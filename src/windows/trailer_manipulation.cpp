#include "trailer_manipulation.hpp"

#include <algorithm>
#include <atomic>
#include <vector>

#include "imgui.h"

#include "core.hpp"
#include "hooks/function_hook.hpp"
#include "memory/memory_utils.hpp"
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

    // Which trailer the suspension keybinds act on (checkbox in each trailer section)
    uint32_t susp_selected = 0;
    float susp_ui[ max_trailers ] = {};

    std::shared_ptr< CFunctionHook > set_individual_steering_hook = nullptr;

    /**
     * \brief Hook for set_individual_steering, which the game calls every steering advance with the
     * angle it computed from the joint. For a trailer the user took control of, swap in their angle.
     *
     * Before 1.61 this feature skipped physics_trailer_u::steering_advance entirely, but that function
     * now also runs a per-wheel update after setting the angle, so only the angle is replaced here.
     */
    void hk_set_individual_steering( prism::vehicle_wheel_steering_data_t* data, float angle )
    {
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

        this->valid_ = true;
        return this->valid_;
    }


    // The current trailer chain, refreshed by update() every frame on the render thread.
    void* trailers[ max_trailers ] = {};
    uint32_t trailer_count = 0;

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
        void* trailer = game_actor == nullptr ? nullptr : prism::offsets::field< void* >( game_actor, prism::offsets::game_actor_trailer );

        trailer_count = 0;
        for ( ; trailer != nullptr && trailer_count < max_trailers; ++trailer_count )
        {
            trailers[ trailer_count ] = trailer;
            trailer = prism::offsets::field< void* >( trailer, prism::offsets::trailer_slave );
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

        // hotkeys, ignored while the game is in the background or a key is being rebound
        if ( game_has_focus() && !settings::is_capturing() )
        {
            bool lock_pressed = false, center_pressed = false, susp_next = false;
            key_down( settings::Action::TOGGLE_STEERING_LOCK, &lock_pressed );
            key_down( settings::Action::STEER_CENTER, &center_pressed );
            const bool left = key_down( settings::Action::STEER_LEFT );
            const bool right = key_down( settings::Action::STEER_RIGHT );
            const bool susp_up = key_down( settings::Action::SUSP_UP );
            const bool susp_down = key_down( settings::Action::SUSP_DOWN );
            key_down( settings::Action::SUSP_NEXT_TRAILER, &susp_next );

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

            // trailer suspension: the game's own keys only ever move trailer 0,
            // these act on whichever trailer is selected in the window
            if ( susp_next && trailer_count > 0 ) susp_selected = ( susp_selected + 1 ) % trailer_count;
            if ( ( susp_up || susp_down ) && susp_selected < trailer_count &&
                 prism::offsets::trailer_suspension != prism::offsets::invalid )
            {
                auto& h = prism::offsets::field< float >( trailers[ susp_selected ], prism::offsets::trailer_suspension );
                const float step = 0.25f * dt;
                if ( susp_up ) h = ( std::min )( h + step, 1.f );
                if ( susp_down ) h = ( std::max )( h - step, 0.f );
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

        if ( susp_selected >= trailer_count ) susp_selected = 0;
        if ( prism::offsets::trailer_suspension != prism::offsets::invalid )
        {
            for ( uint32_t i = 0; i < trailer_count; ++i )
            {
                susp_ui[ i ] = prism::offsets::field< float >( trailers[ i ], prism::offsets::trailer_suspension );
            }
        }
    }

    void CTrailerManipulation::render_trailer_suspension( const uint32_t i )
    {
        if ( prism::offsets::trailer_suspension == prism::offsets::invalid )
        {
            ImGui::TextDisabled( "Suspension field not located yet (see Memory research below)" );
            return;
        }

        bool sel = susp_selected == i;
        if ( ImGui::Checkbox( "Suspension target", &sel ) && sel ) susp_selected = i;

        auto& h = prism::offsets::field< float >( trailers[ i ], prism::offsets::trailer_suspension );
        ImGui::PushItemFlag( ImGuiItemFlags_ButtonRepeat, true );
        ImGui::BeginDisabled( susp_selected != i );
        if ( ImGui::ArrowButton( "susp_down", ImGuiDir_Down ) ) h = ( std::max )( h - 0.02f, 0.f );
        ImGui::SameLine();
        if ( ImGui::ArrowButton( "susp_up", ImGuiDir_Up ) ) h = ( std::min )( h + 0.02f, 1.f );
        ImGui::EndDisabled();
        ImGui::PopItemFlag();
        ImGui::SameLine();
        ImGui::Text( "%.3f", susp_ui[ i ] );
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

    // ---------------- memory research tool ----------------
    // Snapshot the trailer object plus its child objects, then diff after
    // pressing the game's own suspension keys to locate the suspension field.
    // The target value may live on a wheel/axle child, not the trailer itself.

    constexpr uint32_t kScanBytes = 0x1800;
    constexpr uint32_t kChildBytes = 0x400;
    constexpr uint32_t kMaxChildren = 32;
    constexpr uint32_t kScanTrailers = 4;

    // returns how many bytes are readable starting at p (0 when unreadable)
    size_t readable_size( const void* p, const size_t want )
    {
        MEMORY_BASIC_INFORMATION mbi{};
        if ( VirtualQuery( p, &mbi, sizeof( mbi ) ) == 0 ) return 0;
        if ( mbi.State != MEM_COMMIT ) return 0;
        const DWORD prot = mbi.Protect & 0xff;
        if ( prot == PAGE_NOACCESS || prot == PAGE_EXECUTE ) return 0;
        const auto start = reinterpret_cast< uintptr_t >( p );
        const auto end = reinterpret_cast< uintptr_t >( mbi.BaseAddress ) + mbi.RegionSize;
        if ( start >= end ) return 0;
        return ( std::min )( want, static_cast< size_t >( end - start ) );
    }

    bool readable_mem( const void* p, const size_t n )
    {
        return readable_size( p, n ) >= n;
    }

    // looks like a pointer into process memory (not code, not tiny)
    bool looks_like_ptr( const void* p )
    {
        const auto v = reinterpret_cast< uintptr_t >( p );
        if ( v < 0x10000 ) return false;
        return readable_mem( p, 8 );
    }

    struct ChildSnap
    {
        void* target = nullptr;   // address of the child object
        uint32_t ptr_off = 0;     // offset in the trailer where the pointer lives
        uint8_t bytes[ kChildBytes ] = {};
    };

    struct TrailerSnap
    {
        void* obj = nullptr;
        uint32_t scan_bytes = 0; // clamped to the readable region
        uint8_t bytes[ kScanBytes ] = {};
        ChildSnap children[ kMaxChildren ] = {};
        uint32_t child_count = 0;
    };

    TrailerSnap snaps[ kScanTrailers ];
    bool snaps_valid = false;

    // per-region noise masks
    bool noise_trailer[ kScanBytes / 4 ] = {};
    bool noise_child[ kChildBytes / 4 ] = {};
    bool noise_has = false;

    void snapshot_trailer( TrailerSnap& s, void* obj )
    {
        s.obj = obj;
        s.child_count = 0;
        s.scan_bytes = static_cast< uint32_t >( readable_size( obj, kScanBytes ) );
        memcpy( s.bytes, obj, s.scan_bytes );

        // collect pointers to readable child objects and snapshot them too
        for ( uint32_t off = 0; off + 8 <= s.scan_bytes && s.child_count < kMaxChildren; off += 8 )
        {
            void* target = *reinterpret_cast< void** >( s.bytes + off );
            if ( !looks_like_ptr( target ) ) continue;
            // skip self-references and pointers back into the trailer itself
            const auto tv = reinterpret_cast< uintptr_t >( obj );
            const auto cv = reinterpret_cast< uintptr_t >( target );
            if ( cv >= tv && cv < tv + s.scan_bytes ) continue;

            auto& c = s.children[ s.child_count ];
            c.target = target;
            c.ptr_off = off;
            if ( readable_mem( target, kChildBytes ) )
            {
                memcpy( c.bytes, target, kChildBytes );
            }
            ++s.child_count;
        }
    }

    void take_snapshot()
    {
        for ( uint32_t i = 0; i < kScanTrailers; ++i )
        {
            if ( i < trailer_count && readable_mem( trailers[ i ], 8 ) )
            {
                snapshot_trailer( snaps[ i ], trailers[ i ] );
            }
            else
            {
                snaps[ i ].obj = nullptr;
            }
        }
        snaps_valid = true;
        CCore::g_instance->info( "[research] snapshot: {} trailers", trailer_count );
        for ( uint32_t i = 0; i < kScanTrailers; ++i )
        {
            if ( snaps[ i ].obj != nullptr )
            {
                CCore::g_instance->info( "[research] T{} @ {} scan=0x{:x} children={}",
                                         i, snaps[ i ].obj, snaps[ i ].scan_bytes, snaps[ i ].child_count );
            }
        }
    }

    struct DiffLine
    {
        uint32_t trailer;
        int32_t child;      // -1 = the trailer itself, else index into children[]
        uint32_t ptr_off;   // where the child pointer lives in the trailer
        uint32_t off;
        float old_f, new_f;
        uint32_t old_u, new_u;
    };
    std::vector< DiffLine > diff_lines;

    void diff_block( const uint32_t i, const int32_t child, const uint32_t ptr_off,
                     const uint8_t* old_b, const uint8_t* cur_b, const uint32_t bytes,
                     const bool* mask, const bool mark )
    {
        for ( uint32_t off = 0; off < bytes; off += 4 )
        {
            const uint32_t old_u = *reinterpret_cast< const uint32_t* >( old_b + off );
            const uint32_t new_u = *reinterpret_cast< const uint32_t* >( cur_b + off );
            if ( old_u == new_u ) continue;
            if ( mask[ off / 4 ] ) continue;
            if ( mark ) continue;
            float old_f, new_f;
            memcpy( &old_f, &old_u, 4 );
            memcpy( &new_f, &new_u, 4 );
            diff_lines.push_back( { i, child, ptr_off, off, old_f, new_f, old_u, new_u } );
        }
    }

    // mark=true fills the noise masks instead of producing lines
    void compute_diff( const bool mark_noise )
    {
        if ( !mark_noise ) diff_lines.clear();
        for ( uint32_t i = 0; i < kScanTrailers; ++i )
        {
            const auto& s = snaps[ i ];
            if ( s.obj == nullptr || trailers[ i ] != s.obj )
            {
                if ( !mark_noise && s.obj != nullptr )
                {
                    CCore::g_instance->info( "[research] T{} skipped: ptr changed {} -> {}", i, s.obj, trailers[ i ] );
                }
                continue;
            }

            if ( mark_noise )
            {
                const auto* cur = static_cast< const uint8_t* >( trailers[ i ] );
                for ( uint32_t off = 0; off < s.scan_bytes; off += 4 )
                {
                    if ( memcmp( s.bytes + off, cur + off, 4 ) != 0 ) noise_trailer[ off / 4 ] = true;
                }
                for ( uint32_t c = 0; c < s.child_count; ++c )
                {
                    if ( !readable_mem( s.children[ c ].target, kChildBytes ) ) continue;
                    const auto* cc = static_cast< const uint8_t* >( s.children[ c ].target );
                    for ( uint32_t off = 0; off < kChildBytes; off += 4 )
                    {
                        if ( memcmp( s.children[ c ].bytes + off, cc + off, 4 ) != 0 ) noise_child[ off / 4 ] = true;
                    }
                }
                continue;
            }

            diff_block( i, -1, 0, s.bytes, static_cast< const uint8_t* >( trailers[ i ] ),
                        s.scan_bytes, noise_trailer, false );

            for ( uint32_t c = 0; c < s.child_count; ++c )
            {
                const auto& ch = s.children[ c ];
                // only diff the child if the trailer still points at the same object
                void* cur_target = *reinterpret_cast< void** >( static_cast< uint8_t* >( trailers[ i ] ) + ch.ptr_off );
                if ( cur_target != ch.target || !readable_mem( ch.target, kChildBytes ) ) continue;
                diff_block( i, static_cast< int32_t >( c ), ch.ptr_off, ch.bytes,
                            static_cast< const uint8_t* >( ch.target ), kChildBytes, noise_child, false );
            }
        }
        if ( mark_noise ) noise_has = true;
    }

    void CTrailerManipulation::render_memory_debug()
    {
        ImGui::SeparatorText( "Memory research" );
        ImGui::TextWrapped(
            "1. Snapshot -> wait ~3s -> Ignore diff (marks physics noise)\n"
            "2. Snapshot -> press trailer susp keys -> Diff now" );

        if ( ImGui::Button( "Snapshot" ) ) take_snapshot();
        ImGui::SameLine();
        if ( ImGui::Button( "Ignore diff" ) ) compute_diff( true );
        ImGui::SameLine();
        if ( ImGui::Button( "Diff now" ) )
        {
            compute_diff( false );
            for ( const auto& l : diff_lines )
            {
                if ( l.child < 0 )
                {
                    CCore::g_instance->info( "[research] T{} +0x{:04x}: f {:>12} -> {:<12} h {:08x} -> {:08x}",
                                             l.trailer, l.off, l.old_f, l.new_f, l.old_u, l.new_u );
                }
                else
                {
                    CCore::g_instance->info( "[research] T{} child@+0x{:04x} +0x{:04x}: f {:>12} -> {:<12} h {:08x} -> {:08x}",
                                             l.trailer, l.ptr_off, l.off, l.old_f, l.new_f, l.old_u, l.new_u );
                }
            }
            CCore::g_instance->info( "[research] diff auto-logged, {} lines", diff_lines.size() );
        }
        ImGui::SameLine();
        if ( ImGui::Button( "Clear noise" ) )
        {
            memset( noise_trailer, 0, sizeof( noise_trailer ) );
            memset( noise_child, 0, sizeof( noise_child ) );
            noise_has = false;
        }
        ImGui::SameLine();
        if ( ImGui::Button( "Log diff" ) )
        {
            for ( const auto& l : diff_lines )
            {
                if ( l.child < 0 )
                {
                    CCore::g_instance->info( "[research] T{} +0x{:04x}: f {:>12} -> {:<12} h {:08x} -> {:08x}",
                                             l.trailer, l.off, l.old_f, l.new_f, l.old_u, l.new_u );
                }
                else
                {
                    CCore::g_instance->info( "[research] T{} child@+0x{:04x} +0x{:04x}: f {:>12} -> {:<12} h {:08x} -> {:08x}",
                                             l.trailer, l.ptr_off, l.off, l.old_f, l.new_f, l.old_u, l.new_u );
                }
            }
            CCore::g_instance->info( "[research] diff logged, {} lines", diff_lines.size() );
        }

        // poke: write a float at an arbitrary offset of the selected trailer, for
        // testing candidate suspension fields live
        static uint32_t poke_off = 0;
        static float poke_val = 0.f;
        ImGui::PushItemWidth( 110 );
        ImGui::InputScalar( "off##poke", ImGuiDataType_U32, &poke_off, nullptr, nullptr,
                            "0x%x", ImGuiInputTextFlags_CharsHexadecimal );
        ImGui::SameLine();
        ImGui::InputFloat( "value##poke", &poke_val, 0.f, 0.f, "%.4f" );
        ImGui::PopItemWidth();
        ImGui::SameLine();
        ImGui::BeginDisabled( trailer_count == 0 || poke_off >= kScanBytes - 4 );
        if ( ImGui::Button( "Write" ) )
        {
            prism::offsets::field< float >( trailers[ susp_selected ], poke_off ) = poke_val;
        }
        ImGui::EndDisabled();
        if ( trailer_count > 0 && poke_off < kScanBytes - 4 )
        {
            ImGui::SameLine();
            ImGui::Text( "T%u now: %.4f", susp_selected,
                         prism::offsets::field< float >( trailers[ susp_selected ], poke_off ) );
        }

        if ( diff_lines.empty() ) return;
        ImGui::BeginChild( "diff", ImVec2( 0, 180 ), ImGuiChildFlags_Border );
        for ( const auto& l : diff_lines )
        {
            if ( l.child < 0 )
            {
                ImGui::Text( "T%u +0x%03x  f %10.4f -> %10.4f  h %08x -> %08x",
                             l.trailer, l.off, l.old_f, l.new_f, l.old_u, l.new_u );
            }
            else
            {
                ImGui::Text( "T%u child[+0x%03x]+0x%03x  f %10.4f -> %10.4f  h %08x -> %08x",
                             l.trailer, l.ptr_off, l.off, l.old_f, l.new_f, l.old_u, l.new_u );
            }
        }
        ImGui::EndChild();
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
                    ImGui::SeparatorText( "Suspension" );
                    this->render_trailer_suspension( i );

                    if ( steering_data( trailers[ i ] ) != nullptr )
                    {
                        ImGui::SeparatorText( "Steering" );
                        this->render_trailer_steering( i );
                    }
                    else
                    {
                        ImGui::TextDisabled( "No steerable wheels" );
                    }
                }
                ImGui::PopID();
            }

            this->render_memory_debug();
        }

        ImGui::End();
    }
}
