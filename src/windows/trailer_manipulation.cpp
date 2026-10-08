#include "trailer_manipulation.hpp"

#include <algorithm>
#include <atomic>

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

    // Trailer that receives the game's native suspension controls. While a trailer
    // other than 0 is selected, game_actor->first_trailer points at it, so the
    // game's own susp up/down keys act on the selected trailer.
    uint32_t susp_selected = 0;
    void* swap_actor = nullptr;      // game_actor whose trailer slot we patched
    void* swap_written = nullptr;    // the trailer pointer we placed in the slot
    void* swap_real_first = nullptr; // genuine first trailer while the swap is active

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

        // leave the game's trailer slot as we found it
        if ( swap_actor != nullptr )
        {
            prism::offsets::field< void* >( swap_actor, prism::offsets::game_actor_trailer ) = swap_real_first;
            swap_actor = nullptr;
        }
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

        // If we patched game_actor->first_trailer, recover the genuine first trailer
        // for enumeration; a slot holding something else means the game rewrote it
        // (attach/detach), which becomes the new real first.
        void* trailer = nullptr;
        if ( game_actor != nullptr )
        {
            trailer = prism::offsets::field< void* >( game_actor, prism::offsets::game_actor_trailer );
            if ( swap_actor == game_actor )
            {
                // slot still holds our trailer -> real first unchanged; anything else
                // is a fresh write by the game (attach/detach) and becomes the real first
                if ( trailer != swap_written ) swap_real_first = trailer;
                trailer = swap_real_first;
            }
            else
            {
                // actor changed since the swap (new vehicle/session); nothing to restore
                swap_actor = nullptr;
                swap_written = nullptr;
                swap_real_first = nullptr;
            }
        }

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

            // cycle which trailer receives the game's native suspension control
            if ( susp_next && trailer_count > 0 ) susp_selected = ( susp_selected + 1 ) % trailer_count;
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

        // ---- trailer suspension: point the game's "first trailer" slot at the
        // selected trailer so the native susp keys act on it ----
        if ( susp_selected >= trailer_count ) susp_selected = 0;
        if ( game_actor != nullptr )
        {
            void* desired = susp_selected == 0 ? nullptr : trailers[ susp_selected ];
            if ( desired != nullptr )
            {
                if ( swap_actor != game_actor )
                {
                    swap_actor = game_actor;
                    swap_real_first = trailers[ 0 ];
                    CCore::g_instance->info( "Routing trailer suspension controls to trailer {}", susp_selected );
                }
                prism::offsets::field< void* >( game_actor, prism::offsets::game_actor_trailer ) = desired;
                swap_written = desired;
            }
            else if ( swap_actor == game_actor )
            {
                prism::offsets::field< void* >( game_actor, prism::offsets::game_actor_trailer ) = swap_real_first;
                swap_actor = nullptr;
                swap_written = nullptr;
                swap_real_first = nullptr;
                CCore::g_instance->info( "Trailer suspension controls back to trailer 0" );
            }
        }
    }

    void CTrailerManipulation::render_trailer_suspension( const uint32_t i )
    {
        if ( i == 0 )
        {
            ImGui::TextDisabled( "Game susp keys control this trailer natively" );
            ImGui::SameLine();
            if ( susp_selected != 0 )
            {
                ImGui::TextColored( ImVec4( 1.f, .6f, .1f, 1.f ), "(routed to %u)", susp_selected );
            }
            return;
        }

        bool sel = susp_selected == i;
        if ( ImGui::Checkbox( "Route game susp keys here", &sel ) )
        {
            susp_selected = sel ? i : 0;
        }
        if ( susp_selected == i )
        {
            ImGui::SameLine();
            ImGui::TextColored( ImVec4( .3f, 1.f, .3f, 1.f ), "active" );
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
        }

        ImGui::End();
    }
}
