#include "trailer_manipulation.hpp"

#include <algorithm>
#include <atomic>

#include "imgui.h"

#include "core.hpp"
#include "hooks/function_hook.hpp"
#include "memory/memory_utils.hpp"
#include "prism/common.hpp"
#include "prism/offsets.hpp"
#include "settings/settings.hpp"

namespace ts_extra_utilities
{
    constexpr uint32_t max_trailers = 20;

    // Shared between the render thread (UI) and the game thread (the hook). The hook only
    // compares the pointer, it never dereferences it, so a stale entry is harmless.
    struct LockedSteering
    {
        std::atomic< prism::vehicle_wheel_steering_data_t* > data{ nullptr };
        std::atomic< float > angle{ 0.f };
    };

    LockedSteering locked_steering[ max_trailers ];

    // UI-only state, render thread
    bool locked_ui[ max_trailers ] = {};
    float angle_ui[ max_trailers ] = {};
    bool centering_ui[ max_trailers ] = {}; // easing back to 0 at steering_speed

    std::shared_ptr< CFunctionHook > set_individual_steering_hook = nullptr;
    std::shared_ptr< CFunctionHook > crashes_when_disconnected_hook = nullptr;
    std::shared_ptr< CFunctionHook > connect_slave_hook = nullptr;
    std::shared_ptr< CFunctionHook > brake_chain_setter_hook = nullptr;
    std::shared_ptr< CFunctionHook > brake_setter_hook = nullptr;
    prism::physics_trailer_u_get_slave_hook_position_fn* get_slave_hook_position_fn = nullptr;

    // native helpers replayed around connect/disconnect so a manually handled
    // trailer gets the same treatment as the game's own T-key flow
    prism::vehicle_set_towed_trailer_fn* set_towed_trailer_fn = nullptr;
    prism::vehicle_clear_detached_flag_fn* clear_detached_flag_fn = nullptr;
    prism::vehicle_rebuild_lines_fn* rebuild_lines_fn = nullptr;
    prism::vehicle_distribute_towed_mass_fn* distribute_towed_mass_fn = nullptr;
    prism::game_actor_refresh_axle_lift_fn* refresh_axle_lift_fn = nullptr;
    prism::console_execute_line_fn* execute_line_fn = nullptr;

    // trailer currently jointed straight to the truck through "connect to truck";
    // used to refuse a second trailer on the fifth wheel
    void* truck_towed = nullptr;

    // The current trailer chain, refreshed by update() every frame on the render thread.
    void* trailers[ max_trailers ] = {};
    uint32_t trailer_count = 0;

    // trailers intentionally detached through this plugin. The game's own
    // attach/detach flows call connect_slave on the parent, which would re-attach
    // them right back — so the hook stays active and skips only those trailers
    void* detached_trailers[ max_trailers ] = {};

    bool is_detached( const void* trailer )
    {
        for ( const void* d : detached_trailers ) if ( d == trailer ) return true;
        return false;
    }

    void mark_detached( void* trailer )
    {
        for ( void*& d : detached_trailers ) if ( d == nullptr ) { d = trailer; return; }
    }

    void unmark_detached( const void* trailer )
    {
        for ( void*& d : detached_trailers ) if ( d == trailer ) d = nullptr;
    }

    void connect_now( void* trailer, void* parent, bool parent_is_truck );
    void disconnect_now( void* trailer, void* parent, bool parent_is_truck );

    // Connect/disconnect calls the game's own vfuncs, which must run on the
    // simulation thread — but update()/the UI run on the render thread, where
    // calling them races the physics update (pure virtual call crashes). The
    // buttons/keys only queue the operation; it is executed inside the hooked
    // per-frame trailer call below, which the game itself runs on the sim thread
    enum class pending_kind { none, connect, disconnect };

    struct pending_op
    {
        pending_kind kind = pending_kind::none;
        void* trailer = nullptr;
        void* parent = nullptr;
        bool parent_is_truck = false;
    };

    pending_op pending;

    // console lines have the same thread rule: execute_line must run on the sim
    // thread, calling it from update() raced the command machinery and crashed
    char pending_cmd[ 64 ] = {};
    std::atomic< bool > pending_cmd_queued{ false };

    bool queue_op( const pending_kind kind, void* trailer, void* parent, const bool parent_is_truck )
    {
        if ( pending.kind != pending_kind::none )
        {
            CCore::g_instance->warning( "Another trailer connect/disconnect is still in progress" );
            return false;
        }
        pending = { kind, trailer, parent, parent_is_truck };
        return true;
    }

    /**
     * \brief The original dereferences the trailer's physics joint, which is null on a
     * detached trailer. Skip only that case and run the original otherwise.
     */
    void hk_crashes_when_disconnected( prism::physics_trailer_u* self, prism::game_trailer_actor_u* trailer )
    {
        if ( pending.kind != pending_kind::none )
        {
            const pending_op op = pending;
            pending = {};
            if ( op.kind == pending_kind::connect )
            {
                connect_now( op.trailer, op.parent, op.parent_is_truck );
            }
            else
            {
                disconnect_now( op.trailer, op.parent, op.parent_is_truck );
            }
        }
        if ( pending_cmd_queued.load( std::memory_order_acquire ) && execute_line_fn != nullptr )
        {
            pending_cmd_queued.store( false, std::memory_order_release );
            char* text = pending_cmd;
            execute_line_fn( &text, -1 );
        }
        if ( trailer != nullptr &&
            prism::offsets::field< void* >( trailer, prism::offsets::trailer_physics_joint ) == nullptr )
        {
            return;
        }
        crashes_when_disconnected_hook->get_original< prism::crashes_when_disconnected_fn >()( self, trailer );
    }

    // the game calls this to auto-attach a parent's slave trailer; refuse only when
    // that slave was detached through this plugin, so native attach flows keep working
    void hk_connect_slave( prism::physics_trailer_u* self )
    {
        void* slave = self != nullptr
            ? prism::offsets::field< void* >( self, prism::offsets::trailer_slave )
            : nullptr;
        if ( slave != nullptr && is_detached( slave ) )
        {
            return;
        }
        connect_slave_hook->get_original< prism::physics_trailer_u_connect_slave_fn >()( self );
    }

    using brake_setter_fn = void __fastcall( void* /*vehicle*/, float /*value*/ );

    // The parked set's per-frame update pushes spring brake (1.0) onto its chain
    // head and the setter tail-propagates it down every trailer_slave link. A
    // trailer moved to the truck stays linked under the parked set, so it keeps
    // getting re-braked no matter what the truck broadcasts. Swallow positive
    // writes aimed at the trailer on the fifth wheel: nothing else may apply its
    // parking brake while it is physically coupled to the truck.
    void __fastcall hk_brake_chain_setter( void* self, float v )
    {
        if ( self == truck_towed && v > 0.f ) return;
        brake_chain_setter_hook->get_original< brake_setter_fn >()( self, v );
    }

    void __fastcall hk_brake_setter( void* self, float v )
    {
        if ( self == truck_towed && v > 0.f ) return;
        brake_setter_hook->get_original< brake_setter_fn >()( self, v );
    }

    template < typename F, typename... A >
    auto vcall( void* object, const uint32_t slot, A&&... args )
    {
        auto* vtbl = *reinterpret_cast< void*** >( object );
        return reinterpret_cast< F* >( vtbl[ slot ] )( object, std::forward< A >( args )... );
    }

    void* joint( void* trailer )
    {
        return prism::offsets::field< void* >( trailer, prism::offsets::trailer_physics_joint );
    }



    /**
     * \brief Hook for set_individual_steering, which the game calls every steering advance with the
     * angle it computed from the joint. For a trailer the user took control of, swap in their angle.
     *
     * Before 1.61 this feature skipped physics_trailer_u::steering_advance entirely, but that function
     * now also runs a per-wheel update after setting the angle, so only the angle is replaced here.
     */
    void hk_set_individual_steering( prism::vehicle_wheel_steering_data_t* data, float angle )
    {
        // diagnostic: log each distinct steering-data pointer once, so the game log shows
        // which objects the game steers and whether a trailer ever shows up here
        static void* seen_data[ 16 ] = {};
        static uint32_t seen_count = 0;
        bool known = false;
        for ( uint32_t i = 0; i < seen_count; ++i ) known |= seen_data[ i ] == data;
        if ( !known && seen_count < 16 )
        {
            seen_data[ seen_count++ ] = data;
            CCore::g_instance->info( "set_individual_steering sees data={:p} angle={:.3f}", static_cast< void* >( data ), angle );
        }

        for ( auto& locked : locked_steering )
        {
            if ( data != nullptr && locked.data.load( std::memory_order_relaxed ) == data )
            {
                angle = locked.angle.load( std::memory_order_relaxed );
                break;
            }
        }
        set_individual_steering_hook->get_original< prism::set_individual_steering_fn >()( data, angle );
    }

    void release_all()
    {
        for ( uint32_t i = 0; i < max_trailers; ++i )
        {
            locked_steering[ i ].data.store( nullptr );
            locked_ui[ i ] = false;
        }
    }

    CTrailerManipulation::CTrailerManipulation() = default;

    CTrailerManipulation::~CTrailerManipulation()
    {
        release_all();
        set_individual_steering_hook.reset();
        crashes_when_disconnected_hook.reset();
        connect_slave_hook.reset();
    }

    bool CTrailerManipulation::init()
    {
        const auto address = memory::get_address_for_pattern( "48 89 5c 24 08 48 89 74 24 10 57 48 83 ec ? 8b 41 ? 48 8b d9 0f 29 74" );

        if ( address == 0 )
        {
            CCore::g_instance->error( "Could not find 'set_individual_steering' function" );
            return false;
        }
        CCore::g_instance->debug( "Found set_individual_steering function @ +{:x}", memory::as_offset( address ) );

        set_individual_steering_hook = CCore::g_instance->get_hooks_manager()->register_function_hook(
            "set_individual_steering",
            address,
            reinterpret_cast< uint64_t >( &hk_set_individual_steering ) );

        if ( set_individual_steering_hook->hook() != CHook::HOOKED )
        {
            CCore::g_instance->error( "Could not hook 'set_individual_steering'" );
            return false;
        }

        // Individually detachable trailers. Two extra guards are needed: a null-joint
        // check the game crashes on while a trailer is detached, and connect_slave,
        // which would otherwise re-attach the trailers behind the one being connected.
        const auto crash_address = memory::get_address_for_pattern( "48 85 d2 0f 84 ? ? ? ? 48 89 74 24 18 57 48 83 ec 40" );
        const auto connect_slave_address = memory::get_address_for_pattern( "40 53 48 83 ec 60 48 83 b9 ? ? ? ? 00 48 8b d9 0f 84 ? ? ? ? 48 8d 54 24 ? e8" );

        if ( crash_address == 0 || connect_slave_address == 0 )
        {
            CCore::g_instance->warning( "Trailer connect/disconnect is unavailable for this game version" );
        }
        else
        {
            // get_slave_hook_position is the tail call inside connect_slave
            get_slave_hook_position_fn = reinterpret_cast< prism::physics_trailer_u_get_slave_hook_position_fn* >(
                connect_slave_address + 29 + *reinterpret_cast< int32_t* >( connect_slave_address + 29 ) + 4 );

            // the rest of the native attach/detach sequence, located next to the
            // connect/disconnect vfunc calls in the game's own T-key flow
            const auto set_towed_address = memory::get_address_for_pattern( "48 89 5c 24 08 48 89 6c 24 10 48 89 74 24 18 57 48 81 ec 80 00 00 00 48 8b f1 49 8b d8 48 8b 89 a8 15" );
            const auto clear_detached_address = memory::get_address_for_pattern( "40 57 41 56 48 83 ec 58 44 0f b6 f2 48 8b f9 3a 91 8d 0e 00 00 75" );
            const auto rebuild_lines_address = memory::get_address_for_pattern( "48 89 5c 24 08 48 89 74 24 10 57 48 83 ec 40 48 8b fa 48 8b f1 e8 ? ? ? ? 48 8b ce e8 ? ? ? ? 48 8d 8e c0 0e 00 00 e8" );
            const auto distribute_mass_address = memory::get_address_for_pattern( "48 89 6c 24 18 57 48 83 ec 50 0f 29 74 24 40 48 8b f9 0f 57" );
            const auto refresh_lift_address = memory::get_address_for_pattern( "48 8b 91 c8 00 00 00 45 32 c0 48 85 d2 74 2c 4c 8b 8a c8 06" );
            const auto exec_line_address = memory::get_address_for_pattern( "40 55 41 54 41 55 48 8d ac 24 00 f4 ff ff 48 81 ec 00 0d 00 00 44 8b ea 4c 8b e1" );

            if ( exec_line_address != 0 )
            {
                execute_line_fn = reinterpret_cast< prism::console_execute_line_fn* >( exec_line_address );
            }

            if ( set_towed_address != 0 && clear_detached_address != 0 && rebuild_lines_address != 0 &&
                distribute_mass_address != 0 && refresh_lift_address != 0 )
            {
                set_towed_trailer_fn = reinterpret_cast< prism::vehicle_set_towed_trailer_fn* >( set_towed_address );
                clear_detached_flag_fn = reinterpret_cast< prism::vehicle_clear_detached_flag_fn* >( clear_detached_address );
                rebuild_lines_fn = reinterpret_cast< prism::vehicle_rebuild_lines_fn* >( rebuild_lines_address );
                distribute_towed_mass_fn = reinterpret_cast< prism::vehicle_distribute_towed_mass_fn* >( distribute_mass_address );
                refresh_axle_lift_fn = reinterpret_cast< prism::game_actor_refresh_axle_lift_fn* >( refresh_lift_address );
            }
            else
            {
                CCore::g_instance->warning( "Native attach helpers not found; a connected trailer may stay braked" );
            }

            crashes_when_disconnected_hook = CCore::g_instance->get_hooks_manager()->register_function_hook(
                "crashes_when_disconnected",
                crash_address,
                reinterpret_cast< uint64_t >( &hk_crashes_when_disconnected ) );

            connect_slave_hook = CCore::g_instance->get_hooks_manager()->register_function_hook(
                "connect_slave",
                connect_slave_address,
                reinterpret_cast< uint64_t >( &hk_connect_slave ) );

            // hooks on the trailer+0x760 brake setters: they swallow positive
            // writes aimed at the trailer on the fifth wheel so the parked set
            // cannot re-brake it through the shared trailer chain
            const auto brake_chain_setter_address = memory::get_address_for_pattern(
                "f3 0f 11 89 60 07 00 00 48 8b 89 88 10 00 00" );
            const auto brake_setter_address = memory::get_address_for_pattern(
                "f3 0f 11 89 60 07 00 00 c3" );
            if ( brake_chain_setter_address != 0 )
            {
                brake_chain_setter_hook = CCore::g_instance->get_hooks_manager()->register_function_hook(
                    "brake_chain_setter", brake_chain_setter_address,
                    reinterpret_cast< uint64_t >( &hk_brake_chain_setter ) );
                brake_chain_setter_hook->hook();
            }
            if ( brake_setter_address != 0 )
            {
                brake_setter_hook = CCore::g_instance->get_hooks_manager()->register_function_hook(
                    "brake_setter", brake_setter_address,
                    reinterpret_cast< uint64_t >( &hk_brake_setter ) );
                brake_setter_hook->hook();
            }

            if ( !CCore::g_instance->is_truckersmp() &&
                crashes_when_disconnected_hook->hook() == CHook::HOOKED &&
                connect_slave_hook->hook() == CHook::HOOKED )
            {
                this->connect_valid_ = true;
            }
            else
            {
                CCore::g_instance->warning( "Could not set up trailer connect/disconnect hooks" );
            }
        }

        this->valid_ = true;
        return this->valid_;
    }


    bool held[ settings::action_count ] = {};

    prism::vehicle_wheel_steering_data_t* steering_data( void* trailer )
    {
        return prism::offsets::field< prism::vehicle_wheel_steering_data_t* >( trailer, prism::offsets::trailer_steering_data );
    }

    void set_locked( const uint32_t i, const bool lock )
    {
        if ( locked_ui[ i ] == lock ) return;
        locked_ui[ i ] = lock;
        CCore::g_instance->info( "{} steering for {}", lock ? "Locking" : "Unlocking", i );
        if ( lock )
        {
            // start from where the game had the wheels so they do not jump
            angle_ui[ i ] = prism::offsets::field< float >( trailers[ i ], prism::offsets::trailer_steering );
            locked_steering[ i ].angle.store( angle_ui[ i ] );
        }
    }

    bool game_has_focus()
    {
        DWORD pid = 0;
        GetWindowThreadProcessId( GetForegroundWindow(), &pid );
        return pid == GetCurrentProcessId();
    }

    // true while the bound key is down; first_press only on the frame it went down
    bool key_down( const settings::Action action, bool* first_press = nullptr )
    {
        const auto vk = settings::key( action );
        const bool down = vk != 0 && ( GetAsyncKeyState( static_cast< int >( vk ) ) & 0x8000 ) != 0;
        bool& was = held[ static_cast< int >( action ) ];
        if ( first_press != nullptr ) *first_press = down && !was;
        was = down;
        return down;
    }

    void CTrailerManipulation::update( const float dt )
    {
        if ( !this->valid_ ) return;

        auto* game_actor = CCore::g_instance->get_game_actor();

        void* trailer = nullptr;
        if ( game_actor != nullptr )
        {
            trailer = prism::offsets::field< void* >( game_actor, prism::offsets::game_actor_trailer );
        }

        trailer_count = 0;
        for ( ; trailer != nullptr && trailer_count < max_trailers; ++trailer_count )
        {
            trailers[ trailer_count ] = trailer;
            trailer = prism::offsets::field< void* >( trailer, prism::offsets::trailer_slave );
        }

        // the trailer parked on the truck through "connect to truck" is free again once its joint is gone
        if ( truck_towed != nullptr && joint( truck_towed ) == nullptr ) truck_towed = nullptr;

        // a spliced trailer no longer appears in the chain walk; keep it listed so
        // the UI and hotkeys can still reach it while it sits on the truck
        if ( truck_towed != nullptr )
        {
            bool listed = false;
            for ( uint32_t i = 0; i < trailer_count; ++i ) listed |= trailers[ i ] == truck_towed;
            if ( !listed && trailer_count < max_trailers ) trailers[ trailer_count++ ] = truck_towed;
        }

        // trailer hose/cable visuals: g_trailer_cables_mode 1 shows the cables on
        // the first trailer only, 2+ shows them on the whole set. The user wants
        // 1 while the first trailer is coupled, 2 once the LAST trailer is coupled
        // (a dolly alone keeps 1), and 0 with nothing coupled. A trailer counts
        // only while actually coupled to the truck's convoy — trailer 1 through
        // the fifth wheel, the rest through the chain or straight on the truck.
        // Written only on change, so the console stays free for the user
        {
            bool in_convoy[ max_trailers ] = {};
            bool parent_in_convoy = true;
            for ( uint32_t i = 0; i < trailer_count; ++i )
            {
                const bool attached = joint( trailers[ i ] ) != nullptr;
                in_convoy[ i ] = trailers[ i ] == truck_towed ? attached : attached && parent_in_convoy;
                parent_in_convoy = in_convoy[ i ];
            }
            uint32_t mode = 0;
            if ( trailer_count > 0 )
            {
                if ( in_convoy[ 0 ] ) mode = 1;
                if ( trailer_count > 1 && in_convoy[ trailer_count - 1 ] ) mode = 2;
            }
            static uint32_t sent_cables_mode = UINT32_MAX;
            if ( mode != sent_cables_mode && execute_line_fn != nullptr )
            {
                sent_cables_mode = mode;
                snprintf( pending_cmd, sizeof( pending_cmd ), "g_trailer_cables_mode %u", mode );
                pending_cmd_queued.store( true, std::memory_order_release );
                CCore::g_instance->info( "g_trailer_cables_mode = {}", mode );
            }
        }

        // a detach mark only protects a trailer while it is jointless or sitting on
        // the truck; back on its chain parent the game may manage it again. Entries
        // no longer in the chain (respawned trailers) are dropped without touching them
        for ( void*& d : detached_trailers )
        {
            if ( d == nullptr || d == truck_towed ) continue;
            bool live = false;
            for ( uint32_t i = 0; i < trailer_count; ++i ) live |= trailers[ i ] == d;
            if ( !live || joint( d ) != nullptr ) d = nullptr;
        }

        // diagnostic: log trailer chain / steering-data transitions instead of spamming every frame
        static uint32_t logged_trailer_count = UINT32_MAX;
        static void* logged_data[ max_trailers ] = {};
        static bool logged_locked[ max_trailers ] = {};
        if ( trailer_count != logged_trailer_count )
        {
            logged_trailer_count = trailer_count;
            CCore::g_instance->info( "trailer_count = {}", trailer_count );
        }
        for ( uint32_t i = 0; i < trailer_count; ++i )
        {
            auto* sd = steering_data( trailers[ i ] );
            if ( sd != logged_data[ i ] )
            {
                logged_data[ i ] = sd;
                CCore::g_instance->info( "trailer {} @ {:p}: steering_data={:p} steering={:.3f}",
                    i, trailers[ i ], static_cast< void* >( sd ),
                    prism::offsets::field< float >( trailers[ i ], prism::offsets::trailer_steering ) );
            }
            if ( locked_ui[ i ] != logged_locked[ i ] )
            {
                logged_locked[ i ] = locked_ui[ i ];
                CCore::g_instance->info( "trailer {} locked={}", i, locked_ui[ i ] );
            }
        }

        // nothing to steer on a trailer without steerable wheels, or one that is gone
        for ( uint32_t i = 0; i < max_trailers; ++i )
        {
            if ( i >= trailer_count || steering_data( trailers[ i ] ) == nullptr )
            {
                locked_ui[ i ] = false;
                centering_ui[ i ] = false;
            }
        }

        // diagnostic: watch every bound key's physical state, outside the focus/capture gate,
        // so the log shows whether a press arrived and why it would have been ignored
        static bool diag_was[ settings::action_count ] = {};
        for ( int a = 0; a < settings::action_count; ++a )
        {
            const auto vk = settings::key( static_cast< settings::Action >( a ) );
            const bool down = vk != 0 && ( GetAsyncKeyState( static_cast< int >( vk ) ) & 0x8000 ) != 0;
            if ( down && !diag_was[ a ] )
            {
                CCore::g_instance->info( "key down: {} (vk={}) focus={} capturing={}",
                    settings::action_name( static_cast< settings::Action >( a ) ), vk,
                    game_has_focus(), settings::is_capturing() );
            }
            diag_was[ a ] = down;
        }

        // hotkeys, ignored while the game is in the background or a key is being rebound
        if ( game_has_focus() && !settings::is_capturing() )
        {
            bool lock_pressed = false, center_pressed = false, left_press = false, right_press = false;
            key_down( settings::Action::TOGGLE_STEERING_LOCK, &lock_pressed );
            key_down( settings::Action::STEER_CENTER, &center_pressed );
            const bool left = key_down( settings::Action::STEER_LEFT, &left_press );
            const bool right = key_down( settings::Action::STEER_RIGHT, &right_press );
            if ( lock_pressed ) CCore::g_instance->info( "key: steering lock toggled" );

            // attach/detach hotkeys: one key toggles a trailer against its chain parent,
            // the truck variants connect that trailer straight to the fifth wheel.
            // Trailer 1 (index 0) is left to the game's own T key
            if ( connect_valid_ )
            {
                bool ad2 = false, ad3 = false, tk2 = false, tk3 = false;
                key_down( settings::Action::ATTACH_DETACH_2, &ad2 );
                key_down( settings::Action::ATTACH_DETACH_3, &ad3 );
                key_down( settings::Action::ATTACH_TRUCK_2, &tk2 );
                key_down( settings::Action::ATTACH_TRUCK_3, &tk3 );
                if ( ad2 ) this->toggle_attach( 1 );
                if ( ad3 ) this->toggle_attach( 2 );
                if ( tk2 ) this->connect_to_truck( 1 );
                if ( tk3 ) this->connect_to_truck( 2 );
            }
            if ( left_press ) CCore::g_instance->info( "key: steer left down" );
            if ( right_press ) CCore::g_instance->info( "key: steer right down" );
            if ( center_pressed ) CCore::g_instance->info( "key: steer center" );

            bool any_locked = false;
            for ( uint32_t i = 0; i < trailer_count; ++i ) any_locked |= locked_ui[ i ];

            for ( uint32_t i = 0; i < trailer_count; ++i )
            {
                if ( steering_data( trailers[ i ] ) == nullptr ) continue;

                if ( lock_pressed )
                {
                    set_locked( i, !any_locked );
                }
                else if ( left || right || center_pressed )
                {
                    set_locked( i, true ); // steering a trailer means taking control of it
                }

                if ( !locked_ui[ i ] ) continue;

                const float step = settings::g_settings.steering_speed * dt;
                if ( center_pressed ) centering_ui[ i ] = true;
                if ( left || right ) centering_ui[ i ] = false;
                if ( centering_ui[ i ] )
                {
                    // ease back to straight instead of snapping
                    if ( angle_ui[ i ] > step ) angle_ui[ i ] -= step;
                    else if ( angle_ui[ i ] < -step ) angle_ui[ i ] += step;
                    else
                    {
                        angle_ui[ i ] = 0.f;
                        centering_ui[ i ] = false;
                    }
                }
                if ( left ) angle_ui[ i ] = ( std::max )( angle_ui[ i ] - step, -1.f );
                if ( right ) angle_ui[ i ] = ( std::min )( angle_ui[ i ] + step, 1.f );
                locked_steering[ i ].angle.store( angle_ui[ i ] );
            }
        }

        // Published every frame: the data pointer changes when trailers are swapped.
        for ( uint32_t i = 0; i < max_trailers; ++i )
        {
            locked_steering[ i ].data.store( locked_ui[ i ] ? steering_data( trailers[ i ] ) : nullptr );
            if ( i < trailer_count && !locked_ui[ i ] )
            {
                angle_ui[ i ] = prism::offsets::field< float >( trailers[ i ], prism::offsets::trailer_steering );
            }
        }

    }

    void CTrailerManipulation::disconnect_trailer( const uint32_t i ) const
    {
        void* trailer = trailers[ i ];
        const bool on_truck = trailer == truck_towed;
        void* parent = nullptr;
        if ( auto* game_actor = CCore::g_instance->get_game_actor() )
        {
            parent = i == 0 || on_truck
                ? prism::offsets::field< void* >( game_actor, prism::offsets::game_actor_truck )
                : trailers[ i - 1 ];
        }

        // a trailer can only leave while nothing is still jointed behind it —
        // detaching a middle link leaves the rear trailer hanging on a detached
        // parent, a state the physics solver cannot initialize (purecall crash).
        // A trailer parked on the truck does not count: its joint is the truck's
        void* next = i + 1 < trailer_count ? trailers[ i + 1 ] : nullptr;
        if ( next != nullptr && next != truck_towed && joint( next ) != nullptr )
        {
            CCore::g_instance->warning( "Detach trailer {} first", i + 1 );
            return;
        }

        CCore::g_instance->info( "Disconnecting trailer {}", i );
        queue_op( pending_kind::disconnect, trailer, parent, i == 0 || on_truck );
    }

    void disconnect_now( void* trailer, void* parent, const bool parent_is_truck )
    {
        // brace down first, while the joint still exists, then drop the joint
        vcall< prism::physics_trailer_u_set_trailer_brace_fn >( trailer, prism::offsets::vfunc_set_brace, true );
        vcall< prism::physics_trailer_u_disconnect_fn >( trailer, prism::offsets::vfunc_disconnect );

        // registered as intentionally detached: native flows must not re-attach it
        mark_detached( trailer );

        // rest of the native detach sequence: flag the trailer as parked, drop the
        // parent's towed-trailer bookkeeping and wheel load, and rebuild the lines
        // so the hoses between the pair come down
        if ( prism::offsets::field< void* >( trailer, prism::offsets::trailer_parked_state ) != nullptr )
        {
            prism::offsets::field< uint8_t >( trailer, prism::offsets::trailer_parked ) = 1;
        }
        // the game applies spring brakes the same way on a jointless trailer
        prism::offsets::field< float >( trailer, prism::offsets::trailer_brake ) = 1.f;
        prism::offsets::field< float >( trailer, prism::offsets::trailer_service_brake ) = 0.f;
        if ( parent != nullptr )
        {
            if ( parent_is_truck )
            {
                // truck-side bookkeeping only; on a trailer parent those fields are
                // something else and the refcounted store would corrupt them
                if ( set_towed_trailer_fn != nullptr )
                {
                    set_towed_trailer_fn( parent, false, nullptr );
                }
                if ( distribute_towed_mass_fn != nullptr )
                {
                    distribute_towed_mass_fn( parent, 0.f, 0.f );
                }
                // nothing tows anymore: back to the uncoupled state or jointless
                // trailers would stop being spring-braked by the per-frame loop
                // (skipped while trailer 1 still sits on the fifth wheel)
                if ( joint( trailers[ 0 ] ) == nullptr )
                {
                    prism::offsets::field< uint32_t >( parent, prism::offsets::truck_coupling_state ) = 0;
                }
            }
            if ( rebuild_lines_fn != nullptr )
            {
                void* slave = prism::offsets::field< void* >( trailer, prism::offsets::trailer_slave );
                prism::offsets::field< void* >( trailer, prism::offsets::trailer_slave ) = nullptr;
                rebuild_lines_fn( parent, trailer );
                prism::offsets::field< void* >( trailer, prism::offsets::trailer_slave ) = slave;
            }
        }
        if ( trailer == truck_towed ) truck_towed = nullptr;
    }

    void connect_now( void* trailer, void* parent, const bool parent_is_truck )
    {
        float3_t vec{};
        if ( parent_is_truck )
        {
            // hooks to the truck: chassis hook position - truck hook locator
            void* chassis = prism::offsets::field< void* >( parent, prism::offsets::vehicle_chassis_data );
            const float3_t hook = chassis != nullptr
                ? prism::offsets::field< float3_t >( chassis, prism::offsets::chassis_hook_position )
                : float3_t{};
            const float3_t loc = prism::offsets::field< float3_t >( parent, prism::offsets::vehicle_hook_locator );
            vec = float3_t{ hook.x - loc.x, hook.y - loc.y, hook.z - loc.z };
        }
        else
        {
            // slave trailer: rear hook of the trailer ahead - its own hook locator
            float3_t hook{};
            get_slave_hook_position_fn( static_cast< prism::physics_trailer_u* >( parent ), &hook );
            const float3_t loc = prism::offsets::field< float3_t >( parent, prism::offsets::vehicle_hook_locator );
            vec = float3_t{ hook.x - loc.x, hook.y - loc.y, hook.z - loc.z };
        }

        // connect_slave also propagates the parent's coupling byte to the slave and
        // passes the parent's chassis-type flag to connect(); replay both since the
        // permanent hook suppresses connect_slave for plugin-detached trailers
        const auto parent_coupling = prism::offsets::field< uint8_t >( parent, prism::offsets::vehicle_coupling_flag );
        prism::offsets::field< uint8_t >( trailer, prism::offsets::vehicle_coupling_flag ) = parent_coupling;
        void* parent_chassis = prism::offsets::field< void* >( parent, prism::offsets::vehicle_chassis_data );
        const bool slave_hook = parent_chassis != nullptr &&
            prism::offsets::field< uint32_t >( parent_chassis, prism::offsets::chassis_slave_hook_type ) == 2;

        CCore::g_instance->info( "Connecting trailer {:p} to {:p}", trailer, parent );
        vcall< prism::physics_trailer_u_connect_fn >( trailer, prism::offsets::vfunc_connect,
            parent, vec, 0, parent_is_truck || parent_coupling != 0, slave_hook );

        // rest of the native attach sequence (T-key flow): register the trailer on
        // the truck, clear the detached flag on both vehicles, unpark it, raise the
        // brace and rebuild the air/electric lines between the pair.
        // The truck-side bookkeeping (towed pointer, wheel load) is truck-only — the
        // fields it writes belong to the truck's own layout, running them on a
        // trailer parent corrupts whatever lives at those offsets.
        if ( parent_is_truck )
        {
            if ( set_towed_trailer_fn != nullptr )
            {
                set_towed_trailer_fn( parent, true, trailer );
            }
            // mark the truck's coupling state machine as fully coupled: the
            // per-frame trailer loop spring-brakes jointed trailers while it is
            // below 5, and the truck only broadcasts its brake/light state down
            // the towed chain at 5
            prism::offsets::field< uint32_t >( parent, prism::offsets::truck_coupling_state ) = 5;
        }
        if ( clear_detached_flag_fn != nullptr )
        {
            clear_detached_flag_fn( parent, 0, 0 );
            clear_detached_flag_fn( trailer, 0, 0 );
        }
        if ( prism::offsets::field< void* >( trailer, prism::offsets::trailer_parked_state ) != nullptr )
        {
            prism::offsets::field< uint8_t >( trailer, prism::offsets::trailer_parked ) = 0;
        }
        // a jointless trailer sits on spring brakes (trailer_brake = 1.0), applied by
        // the game's own update; now that the joint exists, release it or the trailer
        // keeps dragging along with locked wheels
        prism::offsets::field< float >( trailer, prism::offsets::trailer_brake ) = 0.f;
        prism::offsets::field< uint8_t >( trailer, prism::offsets::trailer_brake_forced ) = 0;
        vcall< prism::physics_trailer_u_set_trailer_brace_fn >( trailer, prism::offsets::vfunc_set_brace, false );
        if ( rebuild_lines_fn != nullptr )
        {
            // rebuild_lines walks the trailer's own slave chain; hide it so the rebuild
            // stops at this link instead of reaching a trailer parked somewhere else
            void* slave = prism::offsets::field< void* >( trailer, prism::offsets::trailer_slave );
            prism::offsets::field< void* >( trailer, prism::offsets::trailer_slave ) = nullptr;
            rebuild_lines_fn( parent, trailer );
            prism::offsets::field< void* >( trailer, prism::offsets::trailer_slave ) = slave;
        }
        if ( parent_is_truck )
        {
            if ( distribute_towed_mass_fn != nullptr )
            {
                distribute_towed_mass_fn( parent, prism::offsets::field< float >( trailer, prism::offsets::trailer_total_mass ), 0.f );
            }
            if ( refresh_axle_lift_fn != nullptr )
            {
                if ( auto* game_actor = CCore::g_instance->get_game_actor() ) refresh_axle_lift_fn( game_actor );
            }
        }
    }

    void CTrailerManipulation::connect_trailer( const uint32_t i ) const
    {
        if ( i >= trailer_count || joint( trailers[ i ] ) != nullptr ) return;
        auto* game_actor = CCore::g_instance->get_game_actor();
        void* parent = i == 0
            ? ( game_actor != nullptr ? prism::offsets::field< void* >( game_actor, prism::offsets::game_actor_truck ) : nullptr )
            : trailers[ i - 1 ];
        if ( parent == nullptr )
        {
            CCore::g_instance->warning( "Cannot connect trailer {}: no parent found", i );
            return;
        }
        // the chain parent must be attached itself — the game only ever creates
        // a joint onto an already-jointed parent; hooking onto a free trailer
        // leaves the component half-initialized (purecall crash)
        if ( i > 0 && joint( parent ) == nullptr )
        {
            CCore::g_instance->warning( "Attach trailer {} first", i );
            return;
        }
        // back on its chain parent: the game may manage this trailer again
        unmark_detached( trailers[ i ] );
        queue_op( pending_kind::connect, trailers[ i ], parent, i == 0 );
    }

    void CTrailerManipulation::connect_to_truck( const uint32_t i ) const
    {
        auto* game_actor = CCore::g_instance->get_game_actor();
        if ( game_actor == nullptr || i >= trailer_count ) return;
        if ( joint( trailers[ i ] ) != nullptr ) return; // already attached

        // the fifth wheel must be free: refuse when trailer 1 is still attached to it
        // or another trailer was connected to the truck through this feature
        if ( joint( trailers[ 0 ] ) != nullptr || truck_towed != nullptr )
        {
            CCore::g_instance->warning( "Truck already has a trailer attached" );
            return;
        }

        // cannot lift a trailer that still tows another one onto the fifth wheel
        void* next = i + 1 < trailer_count ? trailers[ i + 1 ] : nullptr;
        if ( next != nullptr && next != truck_towed && joint( next ) != nullptr )
        {
            CCore::g_instance->warning( "Detach trailer {} first", i + 1 );
            return;
        }

        // stays marked as detached: its chain parent still claims it logically, so
        // connect_slave must keep refusing it while it sits on the fifth wheel
        if ( queue_op( pending_kind::connect, trailers[ i ],
            prism::offsets::field< void* >( game_actor, prism::offsets::game_actor_truck ), true ) )
        {
            truck_towed = trailers[ i ];
        }
    }

    void CTrailerManipulation::toggle_attach( const uint32_t i ) const
    {
        // the first trailer stays on the game's own T key: its detach path is not
        // safe to replay manually
        if ( i == 0 || i >= trailer_count ) return;
        if ( joint( trailers[ i ] ) != nullptr )
        {
            this->disconnect_trailer( i );
        }
        else
        {
            this->connect_trailer( i );
        }
    }

    void CTrailerManipulation::render_trailer_connect( const uint32_t i ) const
    {
        // the first trailer belongs to the game's own T key; the plugin only
        // manages the trailers behind it
        if ( i == 0 )
        {
            ImGui::TextDisabled( "Managed by the game (T key)" );
            return;
        }

        ImGui::SeparatorText( "Connect/Disconnect" );
        if ( !this->connect_valid_ )
        {
            ImGui::TextDisabled( "Unavailable for this game version" );
            return;
        }

        const bool attached = joint( trailers[ i ] ) != nullptr;
        ImGui::BeginDisabled( attached );
        if ( ImGui::Button( "Connect##trailer" ) )
        {
            this->connect_trailer( i );
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled( !attached );
        if ( ImGui::Button( "Disconnect##trailer" ) )
        {
            this->disconnect_trailer( i );
        }
        ImGui::EndDisabled();

        // picking up a trailer behind the first one with the truck directly, only
        // possible while the fifth wheel is free (trailer 1 detached or absent)
        if ( !attached )
        {
            const bool wheel_free = truck_towed == nullptr && ( trailer_count == 0 || joint( trailers[ 0 ] ) == nullptr );
            ImGui::BeginDisabled( !wheel_free );
            if ( ImGui::Button( "Connect to truck##trailer" ) )
            {
                this->connect_to_truck( i );
            }
            ImGui::EndDisabled();
            if ( !wheel_free )
            {
                ImGui::SameLine();
                ImGui::TextDisabled( "(fifth wheel occupied)" );
            }
        }
    }

    void CTrailerManipulation::render_trailer_steering( const uint32_t i ) const
    {
        bool lock = locked_ui[ i ];
        if ( ImGui::Checkbox( "Locked##steering", &lock ) )
        {
            set_locked( i, lock );
        }

        ImGui::BeginDisabled( !locked_ui[ i ] );
        bool changed = ImGui::SliderFloat( "Angle", &angle_ui[ i ], -1.0f, 1.0f, "%.3f", ImGuiSliderFlags_AlwaysClamp );

        ImGui::PushItemFlag( ImGuiItemFlags_ButtonRepeat, true );

        if ( ImGui::ArrowButton( "rotate_left", ImGuiDir_Left ) )
        {
            angle_ui[ i ] = ( std::max )( angle_ui[ i ] - 0.02f, -1.f );
            changed = true;
        }
        ImGui::SameLine();
        if ( ImGui::Button( "center" ) )
        {
            centering_ui[ i ] = true; // eased to 0 in update() at steering_speed
        }
        ImGui::SameLine();
        if ( ImGui::ArrowButton( "rotate_right", ImGuiDir_Right ) )
        {
            angle_ui[ i ] = ( std::min )( angle_ui[ i ] + 0.02f, 1.f );
            changed = true;
        }

        ImGui::PopItemFlag();
        ImGui::EndDisabled();

        if ( changed )
        {
            // applied by the game's next steering advance, on its own thread
            locked_steering[ i ].angle.store( angle_ui[ i ] );
        }
    }

    void CTrailerManipulation::render()
    {
        ImGui::Begin( "Trailer Manipulation" );

        if ( !this->valid_ )
        {
            ImGui::TextWrapped( "Trailer steering is unavailable for this game version, see the game log." );
        }
        else if ( trailer_count == 0 )
        {
            ImGui::Text( "No trailers found" );
        }
        else
        {
            ImGui::Text( "%u trailer%s found", trailer_count, trailer_count > 1 ? "s" : "" );
            ImGui::Spacing();
            for ( uint32_t i = 0; i < trailer_count; ++i )
            {
                const auto trailer_name = fmt::format( "Trailer {}", i );
                ImGui::PushID( trailer_name.c_str() );
                if ( ImGui::CollapsingHeader( trailer_name.c_str(), ImGuiTreeNodeFlags_DefaultOpen ) )
                {
                    if ( steering_data( trailers[ i ] ) != nullptr )
                    {
                        ImGui::SeparatorText( "Steering" );
                        this->render_trailer_steering( i );
                    }
                    else
                    {
                        ImGui::TextDisabled( "No steerable wheels" );
                    }
                    this->render_trailer_connect( i );
                }
                ImGui::PopID();
            }
        }

        ImGui::End();
    }
}
