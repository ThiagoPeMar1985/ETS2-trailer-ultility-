#include "trailer_manipulation.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <intrin.h>

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
    std::shared_ptr< CFunctionHook > forced_chain_setter_hook = nullptr;
    std::shared_ptr< CFunctionHook > forced_setter_hook = nullptr;
    prism::physics_trailer_u_get_slave_hook_position_fn* get_slave_hook_position_fn = nullptr;

    // native helpers replayed around connect/disconnect so a manually handled
    // trailer gets the same treatment as the game's own T-key flow
    prism::vehicle_set_towed_trailer_fn* set_towed_trailer_fn = nullptr;
    // actually the vehicle coupled-state component notify (0x140874010), the
    // same function the native T attach calls on both vehicles
    prism::vehicle_clear_detached_flag_fn* clear_detached_flag_fn = nullptr;
    prism::vehicle_notify_decoupled_fn* notify_decoupled_fn = nullptr;
    prism::vehicle_pre_detach_fn* pre_detach_fn = nullptr;
    prism::vehicle_refresh_accessories_fn* refresh_accessories_fn = nullptr;
    prism::traffic_register_anchor_fn* register_anchor_fn = nullptr;
    // the world/vehicle manager singleton (rbx in the native detach): its +0x2B30
    // subsystem gets vfunc+0x138 (pre) and +0x1E0 (post) notifications
    void** traffic_manager_global = nullptr;
    // game-state dword the native flow checks (== 2 while in game) before
    // registering the parked trailer's obstacle anchor
    uint32_t* traffic_game_state = nullptr;
    prism::cvar_get_float_fn* get_cvar_float_fn = nullptr;
    void* g_traffic_cvar = nullptr;
    // anchor re-link the native detach runs between the two traffic-manager
    // notifies: rebuilds type-2 obstacle registrations under the registry at
    // *(game_actor+0x30)+0xD8
    using anchor_relink_fn = void __fastcall( void* /*registry*/, uint32_t /*type*/ );
    anchor_relink_fn* relink_anchors_fn = nullptr;
    // in-flight counter the native register call brackets itself with
    uint32_t* anchor_reg_guard = nullptr;
    // g_traffic value saved before a forced "g_traffic 0" detach; restored once
    // the whole trailer set is jointed again. < 0 means nothing saved
    float saved_traffic = -1.f;
    // trailers the set had when traffic suppression started. The chain shrinks
    // when links leave it, so "all coupled" must compare against this — a
    // partially rebuilt set (e.g. just the dolly on the truck) must not count
    uint32_t expected_trailers = 0;
    // set only after disconnect_now actually dropped a joint. The detach waits
    // ~30 sim ticks after g_traffic 0, and during that window every trailer
    // still reads as coupled — without this flag the restore fires mid-delay
    // and the joint lands with live traffic again
    std::atomic< bool > traffic_detach_done{ false };
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
        // sim ticks to wait before executing — used to let AI traffic despawn
        // after g_traffic 0 before dropping the joint
        uint32_t delay = 0;
    };

    pending_op pending;

    // console lines have the same thread rule: execute_line must run on the sim
    // thread, calling it from update() raced the command machinery and crashed
    char pending_cmd[ 64 ] = {};
    std::atomic< bool > pending_cmd_queued{ false };

    void queue_cmd( const char* text )
    {
        std::strncpy( pending_cmd, text, sizeof( pending_cmd ) - 1 );
        pending_cmd[ sizeof( pending_cmd ) - 1 ] = '\0';
        pending_cmd_queued.store( true, std::memory_order_release );
    }

    bool queue_op( const pending_kind kind, void* trailer, void* parent, const bool parent_is_truck, const uint32_t delay = 0 )
    {
        if ( pending.kind != pending_kind::none )
        {
            CCore::g_instance->warning( "Another trailer connect/disconnect is still in progress" );
            return false;
        }
        pending = { kind, trailer, parent, parent_is_truck, delay };
        return true;
    }

    /**
     * \brief The original dereferences the trailer's physics joint, which is null on a
     * detached trailer. Skip only that case and run the original otherwise.
     */
    void hk_crashes_when_disconnected( prism::physics_trailer_u* self, prism::game_trailer_actor_u* trailer )
    {
        // console commands run first: a queued "g_traffic 0" must land before a
        // delayed disconnect drops the joint
        if ( pending_cmd_queued.load( std::memory_order_acquire ) && execute_line_fn != nullptr )
        {
            pending_cmd_queued.store( false, std::memory_order_release );
            char* text = pending_cmd;
            execute_line_fn( &text, -1 );
        }
        if ( pending.kind != pending_kind::none )
        {
            if ( pending.delay > 0 )
            {
                --pending.delay;
            }
            else
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
    using forced_setter_fn = void __fastcall( void* /*vehicle*/, uint8_t /*value*/ );

    void* joint( void* trailer );

    // Ret sites of the +0x180/+0x188 vfunc calls inside the truck's per-frame
    // update — the only legitimate source of positive brake writes on a coupled
    // trailer: at coupling_state 5 the truck copies its own brake channels onto
    // the towed chain head and the setter tail-propagates down trailer_slave
    // links, so every propagation frame carries this same return address
    uint64_t truck_brake_broadcast_ret = 0;
    uint64_t truck_forced_broadcast_ret = 0;

    // A trailer that leaves a parked set stays linked under it, so the set's
    // per-frame spring-brake push keeps reaching it through the stale slave
    // link and re-locks the wheels — no matter what the truck broadcasts. Any
    // positive write on a JOINTED trailer that did not come from the truck's
    // own broadcast is a parked-set write and gets swallowed
    bool foreign_brake_write( void* self, const void* ret )
    {
        const auto ra = reinterpret_cast< uint64_t >( ret );
        if ( ra == truck_brake_broadcast_ret || ra == truck_forced_broadcast_ret )
        {
            return false;
        }
        return self != nullptr && ( self == truck_towed || joint( self ) != nullptr );
    }

    // The stale parked chain may continue THROUGH a coupled link to trailers
    // still parked behind it: resume the write on the first jointless link so
    // those keep their spring brake
    void* next_jointless_link( void* self )
    {
        void* cur = prism::offsets::field< void* >( self, prism::offsets::trailer_slave );
        for ( int guard = 0; cur != nullptr && joint( cur ) != nullptr; ++guard )
        {
            if ( guard >= 32 ) return nullptr;
            cur = prism::offsets::field< void* >( cur, prism::offsets::trailer_slave );
        }
        return cur;
    }

    void __fastcall hk_brake_chain_setter( void* self, float v )
    {
        if ( v > 0.f && foreign_brake_write( self, _ReturnAddress() ) )
        {
            if ( void* s = next_jointless_link( self ) )
            {
                brake_chain_setter_hook->get_original< brake_setter_fn >()( s, v );
            }
            return;
        }
        brake_chain_setter_hook->get_original< brake_setter_fn >()( self, v );
    }

    void __fastcall hk_brake_setter( void* self, float v )
    {
        if ( v > 0.f && foreign_brake_write( self, _ReturnAddress() ) ) return;
        brake_setter_hook->get_original< brake_setter_fn >()( self, v );
    }

    void __fastcall hk_forced_chain_setter( void* self, uint8_t v )
    {
        if ( v != 0 && foreign_brake_write( self, _ReturnAddress() ) )
        {
            if ( void* s = next_jointless_link( self ) )
            {
                forced_chain_setter_hook->get_original< forced_setter_fn >()( s, v );
            }
            return;
        }
        forced_chain_setter_hook->get_original< forced_setter_fn >()( self, v );
    }

    void __fastcall hk_forced_setter( void* self, uint8_t v )
    {
        if ( v != 0 && foreign_brake_write( self, _ReturnAddress() ) ) return;
        forced_setter_hook->get_original< forced_setter_fn >()( self, v );
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

    // Memory probes for navigating optional object graphs: a slot may hold
    // garbage, so check the pointer is committed and its first qword is a
    // vtable inside the exe image before trusting it
    bool readable_ptr( const void* p )
    {
        if ( p == nullptr ) return false;
        MEMORY_BASIC_INFORMATION mbi{};
        return VirtualQuery( p, &mbi, sizeof( mbi ) ) != 0 && mbi.State == MEM_COMMIT &&
            ( mbi.Protect & ( PAGE_NOACCESS | PAGE_GUARD ) ) == 0;
    }

    bool plausible_object( const void* p )
    {
        if ( !readable_ptr( p ) ) return false;
        const auto base = reinterpret_cast< uint64_t >( GetModuleHandle( nullptr ) );
        const auto vtbl = *reinterpret_cast< const uint64_t* >( p );
        return vtbl >= base && vtbl < base + 0x4000000;
    }

    // The native detach resolves the detached trailer's actor from embedded
    // pointer slots at game_actor+0x1D38 (0x28 stride per chain index). A
    // slot's actor points back at its trailer at +0x18 — verify that instead
    // of trusting the index, and probe each pointer before dereferencing it
    void* find_trailer_actor( void* trailer, void* game_actor )
    {
        if ( trailer == nullptr || game_actor == nullptr ) return nullptr;
        for ( uint32_t s = 0x1D10; s <= 0x1EA0; s += 0x28 )
        {
            void* actor = *reinterpret_cast< void** >( reinterpret_cast< uint8_t* >( game_actor ) + s );
            if ( plausible_object( actor ) && readable_ptr( reinterpret_cast< uint8_t* >( actor ) + 0x20 ) &&
                prism::offsets::field< void* >( actor, 0x18 ) == trailer )
            {
                return actor;
            }
        }
        return nullptr;
    }

    // native attach/detach anchor update: provider -> vfunc+0x28 yields the
    // trailer's traffic-obstacle anchor, which then gets a transform pushed
    // through 0x140284250 (a locked update, not a register), followed by
    // provider vfunc+0x30. Runs only while the world is in state 2.
    void push_anchor_update( void* provider, const void* xform32 )
    {
        if ( provider == nullptr || !plausible_object( provider ) || register_anchor_fn == nullptr )
        {
            return;
        }
        void* anchor = vcall< prism::provider_get_anchor_fn >( provider, 0x28 / 8 );
        if ( anchor != nullptr && traffic_game_state != nullptr && *traffic_game_state == 2 )
        {
            struct { void* p; uint32_t d; } ctx{};
            if ( anchor_reg_guard != nullptr ) ++*anchor_reg_guard;
            register_anchor_fn( anchor, xform32, &ctx );
            if ( anchor_reg_guard != nullptr ) --*anchor_reg_guard;
        }
        vcall< prism::vehicle_notify_decoupled_fn >( provider, 0x30 / 8 );
    }

    // the truck's hook transform, the way the native coupling paths read it
    bool truck_xform( void* game_actor, uint8_t* out32 )
    {
        void* truck = game_actor != nullptr
            ? prism::offsets::field< void* >( game_actor, prism::offsets::game_actor_truck ) : nullptr;
        if ( truck == nullptr ) return false;
        return vcall< prism::vehicle_get_transform_fn >( truck, 0xF8 / 8, out32 ) != nullptr;
    }

    // prism placement-ish { pos@0, pad, quat@0x10 } — what the 32-byte transform
    // getter (vfunc+0xF8) returns for a vehicle's hook point in world space
    struct hook_xform_t
    {
        float3_t pos;
        float pad;
        quat_t quat;
    };

    // world transform of a vehicle's front hook, through the same getter the
    // native flows use (vfunc+0xF8)
    bool hook_world( void* vehicle, hook_xform_t& out )
    {
        alignas( 16 ) uint8_t buf[ 0x20 ]{};
        void* p = vcall< prism::vehicle_get_transform_fn >( vehicle, 0xF8 / 8, buf );
        if ( p == nullptr ) return false;
        std::memcpy( &out, p, sizeof( out ) );
        // diagnostic: dump the raw placement once per call so the log shows
        // which floats actually carry the world position
        const float* f = reinterpret_cast< const float* >( p );
        CCore::g_instance->info( "xform {:p}: {:.1f} {:.1f} {:.1f} {:.1f} | {:.1f} {:.1f} {:.1f} {:.1f}",
            vehicle, f[ 0 ], f[ 1 ], f[ 2 ], f[ 3 ], f[ 4 ], f[ 5 ], f[ 6 ], f[ 7 ] );
        return std::isfinite( out.pos.x ) && std::isfinite( out.pos.y ) && std::isfinite( out.pos.z ) &&
            std::fabs( out.pos.x ) < 1e6f && std::fabs( out.pos.z ) < 1e6f;
    }

    float3_t quat_rot( const quat_t& q, const float3_t& v )
    {
        // t = 2 * cross(q.xyz, v);  v' = v + w*t + cross(q.xyz, t)
        const float tx = 2.f * ( q.y * v.z - q.z * v.y );
        const float ty = 2.f * ( q.z * v.x - q.x * v.z );
        const float tz = 2.f * ( q.x * v.y - q.y * v.x );
        return float3_t{
            v.x + q.w * tx + q.y * tz - q.z * ty,
            v.y + q.w * ty + q.z * tx - q.x * tz,
            v.z + q.w * tz + q.x * ty - q.y * tx };
    }

    // attach offset in the parent's frame — the same vec passed to the connect
    // vfunc: rear hook point minus the parent's own hook locator
    float3_t attach_offset( void* parent, const bool parent_is_truck )
    {
        float3_t hook{};
        if ( parent_is_truck )
        {
            void* chassis = prism::offsets::field< void* >( parent, prism::offsets::vehicle_chassis_data );
            if ( chassis != nullptr )
            {
                hook = prism::offsets::field< float3_t >( chassis, prism::offsets::chassis_hook_position );
            }
        }
        else if ( get_slave_hook_position_fn != nullptr )
        {
            get_slave_hook_position_fn( static_cast< prism::physics_trailer_u* >( parent ), &hook );
        }
        const float3_t loc = prism::offsets::field< float3_t >( parent, prism::offsets::vehicle_hook_locator );
        return float3_t{ hook.x - loc.x, hook.y - loc.y, hook.z - loc.z };
    }

    // the native T only connects when the hooks nearly touch; our connect vfunc
    // teleports a far trailer onto the hook in one step — an instant jump that,
    // over a live traffic lane, leaves a component half-initialized (purecall).
    // Gate on real hook-to-hook distance: trailer front hook vs parent rear
    // hook in world space. The locator frame's sign convention isn't fully
    // nailed down, so both signed offsets are measured and the nearer wins —
    // attached-or-not, one of them is the true rear hook. Returns < 0 when it
    // cannot be measured.
    float hook_gap( void* trailer, void* parent, const bool parent_is_truck )
    {
        hook_xform_t th{}, ph{};
        if ( trailer == nullptr || parent == nullptr ||
            !hook_world( trailer, th ) || !hook_world( parent, ph ) )
        {
            return -1.f;
        }
        const float3_t v = attach_offset( parent, parent_is_truck );
        const float3_t off = quat_rot( ph.quat, v );
        const float dx = th.pos.x - ph.pos.x;
        const float dy = th.pos.y - ph.pos.y;
        const float dz = th.pos.z - ph.pos.z;
        const float g_plus = std::sqrt(
            ( dx - off.x ) * ( dx - off.x ) +
            ( dy - off.y ) * ( dy - off.y ) +
            ( dz - off.z ) * ( dz - off.z ) );
        const float g_minus = std::sqrt(
            ( dx + off.x ) * ( dx + off.x ) +
            ( dy + off.y ) * ( dy + off.y ) +
            ( dz + off.z ) * ( dz + off.z ) );
        const float vlen = std::sqrt( v.x * v.x + v.y * v.y + v.z * v.z );
        const float raw = std::sqrt( dx * dx + dy * dy + dz * dz );
        CCore::g_instance->info( "hook pts raw={:.2f} vec={:.2f} g+={:.2f} g-={:.2f}",
            raw, vlen, g_plus, g_minus );
        return ( std::min )( g_plus, g_minus );
    }

    // refuse the connect when the hooks are farther apart than the native T
    // would allow — no teleporting the trailer across the map onto the hook
    bool hook_far( void* trailer, void* parent, const bool parent_is_truck )
    {
        const float gap = hook_gap( trailer, parent, parent_is_truck );
        if ( gap < 0.f ) return false; // cannot measure: keep the old fail-open
        CCore::g_instance->info( "Hook gap trailer {:p} to {:p}: {:.2f}m", trailer, parent, gap );
        return gap > 4.f;
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
            // the decoupled-state component notify + the truck-side joint release,
            // both from the native T detach flow
            const auto notify_decoupled_address = memory::get_address_for_pattern( "48 83 ec 48 80 b9 8e 0e 00 00 00 0f 84" );
            const auto pre_detach_address = memory::get_address_for_pattern( "48 85 d2 0f 84 ? ? ? ? 48 89 5c 24 08 57 48 83 ec 20 48 8b d9 48 8b fa 48 8b 8a e0 0f 00 00" );
            // accessory event refresh the native attach/detach fire on the trailer:
            // without it the parked trailer's traffic-obstacle anchor stays broken
            // and the traffic lane-modifier sweep dereferences a null owner
            const auto refresh_accessories_address = memory::get_address_for_pattern(
                "48 89 5c 24 08 48 89 74 24 10 57 48 83 ec 40 48 8b fa 48 8b f1 e8 ? ? ? ? 48 8b ce e8 ? ? ? ? 48 8d 8e c0 0e 00 00 e8" );
            if ( notify_decoupled_address != 0 ) notify_decoupled_fn = reinterpret_cast< prism::vehicle_notify_decoupled_fn* >( notify_decoupled_address );
            if ( pre_detach_address != 0 ) pre_detach_fn = reinterpret_cast< prism::vehicle_pre_detach_fn* >( pre_detach_address );
            if ( refresh_accessories_address != 0 ) refresh_accessories_fn = reinterpret_cast< prism::vehicle_refresh_accessories_fn* >( refresh_accessories_address );

            // the world/vehicle manager singleton + the parked-obstacle anchor
            // registration + the in-game state flag, all read out of the native
            // detach body (fixed image offsets for this build)
            get_cvar_float_fn = reinterpret_cast< prism::cvar_get_float_fn* >( memory::get_address_from_offset( 0x1401DB560 ) );
            g_traffic_cvar = reinterpret_cast< void* >( memory::get_address_from_offset( 0x142D45500 ) );
            traffic_manager_global = reinterpret_cast< void** >( memory::get_address_from_offset( 0x1436AF7C8 ) );
            register_anchor_fn = reinterpret_cast< prism::traffic_register_anchor_fn* >( memory::get_address_from_offset( 0x140284250 ) );
            relink_anchors_fn = reinterpret_cast< anchor_relink_fn* >( memory::get_address_from_offset( 0x140623C90 ) );
            anchor_reg_guard = reinterpret_cast< uint32_t* >( memory::get_address_from_offset( 0x143AF1824 ) );
            traffic_game_state = reinterpret_cast< uint32_t* >( memory::get_address_from_offset( 0x143AF1820 ) );

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

            // hooks on the trailer+0x760/+0x764 brake setters: they swallow
            // positive writes that did not come from the truck's own broadcast
            // when aimed at a coupled trailer, so a stale parked-set chain
            // cannot re-brake it through the shared trailer_slave link
            truck_brake_broadcast_ret = memory::get_address_from_offset( 0x14064B7EF );
            truck_forced_broadcast_ret = memory::get_address_from_offset( 0x14064B806 );
            const auto brake_chain_setter_address = memory::get_address_for_pattern(
                "f3 0f 11 89 60 07 00 00 48 8b 89 88 10 00 00" );
            const auto brake_setter_address = memory::get_address_for_pattern(
                "f3 0f 11 89 60 07 00 00 c3" );
            const auto forced_chain_setter_address = memory::get_address_for_pattern(
                "88 91 64 07 00 00 48 8b 89 88 10 00 00" );
            const auto forced_setter_address = memory::get_address_for_pattern(
                "88 91 64 07 00 00 c3" );
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
            if ( forced_chain_setter_address != 0 )
            {
                forced_chain_setter_hook = CCore::g_instance->get_hooks_manager()->register_function_hook(
                    "forced_chain_setter", forced_chain_setter_address,
                    reinterpret_cast< uint64_t >( &hk_forced_chain_setter ) );
                forced_chain_setter_hook->hook();
            }
            if ( forced_setter_address != 0 )
            {
                forced_setter_hook = CCore::g_instance->get_hooks_manager()->register_function_hook(
                    "forced_setter", forced_setter_address,
                    reinterpret_cast< uint64_t >( &hk_forced_setter ) );
                forced_setter_hook->hook();
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
            if ( mode != sent_cables_mode && execute_line_fn != nullptr &&
                !pending_cmd_queued.load( std::memory_order_acquire ) )
            {
                sent_cables_mode = mode;
                snprintf( pending_cmd, sizeof( pending_cmd ), "g_trailer_cables_mode %u", mode );
                pending_cmd_queued.store( true, std::memory_order_release );
                CCore::g_instance->info( "g_trailer_cables_mode = {}", mode );
            }

            // traffic was zeroed for a detach: bring the saved level back only
            // once the WHOLE original set is coupled again AND a detach has
            // genuinely completed — during the post-command delay every joint
            // is still alive, so the set reads as coupled while traffic would
            // bounce straight back before the joint drops. A pending console
            // line defers the write by a frame instead of being overwritten
            if ( saved_traffic >= 0.f && !pending_cmd_queued.load( std::memory_order_acquire ) )
            {
                bool all_coupled = expected_trailers > 0 && trailer_count >= expected_trailers &&
                    traffic_detach_done.load( std::memory_order_acquire );
                for ( uint32_t i = 0; all_coupled && i < trailer_count; ++i ) all_coupled &= in_convoy[ i ];
                // a parked detached trailer that fell out of the chain must
                // also be back before traffic returns
                for ( void* d : detached_trailers )
                    if ( d != nullptr && joint( d ) == nullptr ) all_coupled = false;
                if ( all_coupled )
                {
                    snprintf( pending_cmd, sizeof( pending_cmd ), "g_traffic %g", saved_traffic );
                    pending_cmd_queued.store( true, std::memory_order_release );
                    CCore::g_instance->info( "restoring g_traffic = {:.2f}", saved_traffic );
                    saved_traffic = -1.f;
                    expected_trailers = 0;
                    traffic_detach_done.store( false, std::memory_order_release );
                }
                // until the whole set is back, traffic stays pinned at zero —
                // even if it was raised by hand through the console in between
                else if ( get_cvar_float_fn != nullptr && g_traffic_cvar != nullptr &&
                    get_cvar_float_fn( g_traffic_cvar ) > 0.f )
                {
                    queue_cmd( "g_traffic 0" );
                }
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

        // a middle link may leave with its slave still jointed — the detached
        // pair (dolly + rear trailer) parks as a unit; the detach mark keeps
        // the game from auto-re-attaching the freed link to its old parent
        CCore::g_instance->info( "Disconnecting trailer {}", i );

        // active AI traffic is what makes the detach crash: the parked trailer
        // becomes a lane obstacle and the traffic sweep dies on it. Zero the
        // traffic first and give the despawn ~30 sim ticks to clear the lanes
        // before the joint actually drops. The live value is saved once —
        // further detaches while part of the set is already out must not
        // overwrite it with the zero we wrote ourselves
        if ( saved_traffic < 0.f && get_cvar_float_fn != nullptr && g_traffic_cvar != nullptr )
        {
            saved_traffic = get_cvar_float_fn( g_traffic_cvar );
            expected_trailers = trailer_count;
            CCore::g_instance->info( "g_traffic before detach = {:.2f} (set size {})", saved_traffic, expected_trailers );
        }
        queue_cmd( "g_traffic 0" );
        // skip the despawn wait only when traffic is known to be already zero;
        // an unreadable cvar keeps the delay (safer than a stale lane crash).
        // Clearing the done flag here keeps the restore from firing during the
        // delay — the joint is still alive and the set still reads as coupled
        if ( queue_op( pending_kind::disconnect, trailer, parent, i == 0 || on_truck, saved_traffic == 0.f ? 0 : 30 ) )
        {
            traffic_detach_done.store( false, std::memory_order_release );
        }
    }

    // diagnostic: the crash scanner filters objects by a coupling state field
    // (obj+0x40 -> +0x68, states 3-5 get scanned). Log it around our ops to see
    // whether a detached loaded trailer keeps a stale coupling state
    void log_couple_state( const char* tag, void* trailer, void* parent )
    {
        void* sm = prism::offsets::field< void* >( trailer, 0x40 );
        const uint32_t st = sm != nullptr ? prism::offsets::field< uint32_t >( sm, 0x68 ) : 0xFFFFFFFF;
        void* psm = parent != nullptr ? prism::offsets::field< void* >( parent, 0x40 ) : nullptr;
        const uint32_t pst = psm != nullptr ? prism::offsets::field< uint32_t >( psm, 0x68 ) : 0xFFFFFFFF;
        CCore::g_instance->info( "{} trailer {:p} sm={:p} st={} | parent {:p} sm={:p} st={}",
            tag, trailer, sm, st, parent, psm, pst );
    }

    void disconnect_now( void* trailer, void* parent, const bool parent_is_truck )
    {
        log_couple_state( "pre-detach", trailer, parent );
        // native T detach order: while the truck's coupling state is nonzero it
        // releases the joint and fifth-wheel lock first, then drops the joint
        if ( parent_is_truck && parent != nullptr && pre_detach_fn != nullptr &&
            prism::offsets::field< uint32_t >( parent, prism::offsets::truck_coupling_state ) != 0 )
        {
            pre_detach_fn( parent, trailer );
        }
        vcall< prism::physics_trailer_u_set_trailer_brace_fn >( trailer, prism::offsets::vfunc_set_brace, true );

        // the native detach pushes the parked trailer's position into its
        // traffic-obstacle anchor BEFORE the joint drops: actor slot at
        // game_actor+0x1D38 -> provider +0x1F0 -> anchor update at the truck's
        // transform. Skipping it leaves the lane modifier owner stale and the
        // traffic sweep hits it whenever the trailer sits on a road
        if ( register_anchor_fn != nullptr )
        {
            void* game_actor = CCore::g_instance->get_game_actor();
            void* actor = find_trailer_actor( trailer, game_actor );
            void* provider = actor != nullptr && readable_ptr( reinterpret_cast< uint8_t* >( actor ) + 0x1F8 )
                ? prism::offsets::field< void* >( actor, 0x1F0 ) : nullptr;
            CCore::g_instance->info( "anchor: trailer {:p} actor {:p} provider {:p}", trailer, actor, provider );
            alignas( 16 ) uint8_t xf[ 0x20 ]{};
            truck_xform( game_actor, xf );
            push_anchor_update( provider, xf );
        }
        vcall< prism::physics_trailer_u_disconnect_fn >( trailer, prism::offsets::vfunc_disconnect );

        // tell every component on both vehicles the coupling is gone — the
        // native detach notifies the vehicle component list (+0xE98) on both
        // sides; a loaded trailer's cargo constraints corrupt the physics
        // sweep if they are never told
        if ( notify_decoupled_fn != nullptr )
        {
            if ( parent != nullptr ) notify_decoupled_fn( parent );
            notify_decoupled_fn( trailer );
        }

        // registered as intentionally detached: native flows must not re-attach it
        mark_detached( trailer );

        // rest of the native detach sequence: flag the trailer as parked, drop the
        // parent's wheel load, and rebuild the lines so the hoses between the
        // pair come down
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
            // native detach fires coupling events at every accessory on the
            // trailer (cargo, traffic-obstacle anchor) — without it the parked
            // trailer registers a lane modifier with a null owner and the
            // traffic sweep crashes on it near roads/docks
            if ( refresh_accessories_fn != nullptr )
            {
                refresh_accessories_fn( parent, trailer );
            }

            // native detach tail: notify the traffic subsystem and bind the
            // parked trailer's obstacle anchor through its actor's position
            // provider — a lane modifier whose anchor stays null crashes the
            // proximity sweep whenever AI traffic comes near
            if ( traffic_manager_global != nullptr )
            {
                void* mgr = *traffic_manager_global;
                void* sub = mgr != nullptr ? prism::offsets::field< void* >( mgr, 0x2B30 ) : nullptr;
                if ( sub != nullptr ) vcall< prism::vehicle_notify_decoupled_fn >( sub, 0x138 / 8 );
            }
            // native detach tail, part 2: re-link the type-2 obstacle anchors
            // in the registry under *(game_actor+0x30)+0xD8 — this is what
            // rebinds the parked trailer's lane modifier to its new owner
            if ( relink_anchors_fn != nullptr )
            {
                void* game_actor = CCore::g_instance->get_game_actor();
                void* reg = game_actor != nullptr ? prism::offsets::field< void* >( game_actor, 0x30 ) : nullptr;
                if ( reg != nullptr && readable_ptr( reinterpret_cast< uint8_t* >( reg ) + 0xD8 ) )
                {
                    relink_anchors_fn( reinterpret_cast< uint8_t* >( reg ) + 0xD8, 2 );
                }
            }
            if ( traffic_manager_global != nullptr )
            {
                void* mgr = *traffic_manager_global;
                void* sub = mgr != nullptr ? prism::offsets::field< void* >( mgr, 0x2B30 ) : nullptr;
                if ( sub != nullptr ) vcall< prism::vehicle_notify_decoupled_fn >( sub, 0x1E0 / 8 );
            }
        }
        if ( trailer == truck_towed ) truck_towed = nullptr;
        log_couple_state( "post-detach", trailer, parent );
        // joint is dead now — a detach genuinely completed, so a later "whole
        // set coupled" state means the user re-assembled it and traffic may
        // be restored
        traffic_detach_done.store( true, std::memory_order_release );
    }

    void connect_now( void* trailer, void* parent, const bool parent_is_truck )
    {
        log_couple_state( "pre-connect", trailer, parent );

        // re-check hook proximity at execution time: the trailer may have been
        // nudged between queueing and this sim tick — never teleport it
        if ( hook_far( trailer, parent, parent_is_truck ) )
        {
            CCore::g_instance->warning( "Trailer is too far — reverse closer to attach" );
            return;
        }

        float3_t vec = attach_offset( parent, parent_is_truck );

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
        // same accessory event refresh the native attach runs on the trailer
        if ( refresh_accessories_fn != nullptr && parent != nullptr )
        {
            refresh_accessories_fn( parent, trailer );
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
        // the native attach pushes the trailer's new position into its two
        // traffic-obstacle anchors: the actor's +0x1F8 provider and the holder
        // under trailer+0x12D8 -> +0x58. Without this the anchor stays frozen at
        // the parked spot and the traffic sweep hits a stale owner once AI
        // traffic comes back and reaches the area
        if ( register_anchor_fn != nullptr )
        {
            void* game_actor = CCore::g_instance->get_game_actor();
            alignas( 16 ) uint8_t xf[ 0x20 ]{};
            truck_xform( game_actor, xf );

            void* actor = find_trailer_actor( trailer, game_actor );
            if ( actor != nullptr && readable_ptr( reinterpret_cast< uint8_t* >( actor ) + 0x200 ) )
            {
                void* provider = prism::offsets::field< void* >( actor, 0x1F8 );
                CCore::g_instance->info( "attach anchor: trailer {:p} actor {:p} provider {:p}", trailer, actor, provider );
                push_anchor_update( provider, xf );
            }

            void* holder = prism::offsets::field< void* >( trailer, 0x12D8 );
            void* provider2 = holder != nullptr && readable_ptr( reinterpret_cast< uint8_t* >( holder ) + 0x60 )
                ? prism::offsets::field< void* >( holder, 0x58 ) : nullptr;
            CCore::g_instance->info( "attach anchor2: holder {:p} provider {:p}", holder, provider2 );
            push_anchor_update( provider2, xf );
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
        // proximity rule like the native T: hooks must nearly touch — no
        // teleporting the trailer onto the hook (< 2 m hook-to-hook)
        if ( hook_far( trailers[ i ], parent, i == 0 ) )
        {
            CCore::g_instance->warning( "Trailer {} is too far — reverse closer to attach", i + 1 );
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

        // same proximity rule: no teleporting a far trailer onto the fifth wheel
        void* truck = prism::offsets::field< void* >( game_actor, prism::offsets::game_actor_truck );
        if ( hook_far( trailers[ i ], truck, true ) )
        {
            CCore::g_instance->warning( "Trailer {} is too far — reverse closer to attach", i + 1 );
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
