#pragma once
#include <cstdint>

// Field offsets for the trailer steering feature, re-derived for game version 1.61.1.9
// (ETS2 and ATS share them). The structs in prism/ still describe the 1.53-era layout
// and are NOT valid for these fields anymore, so read them through here instead.
//
// 1.61.1.9 added 0x28 bytes to the trailer object ahead of the steering fields, so every
// offset past that point moved from the 1.61.1.1 values (steering 0x690, data 0x698,
// slave 0x1060). game_actor itself is unchanged.
//
// How each was found (offline, from the exe):
//  - steering / steering data: physics_trailer_u::steering_advance stores the angle to
//    +0x6B8 and passes the pointer at +0x6C0 to set_individual_steering
//  - slave trailer: prism::physics_trailer_u::connect_slave tests +0x1088 against null
//  - first trailer: the `cheat repair` handler walks game_actor +0x18 (truck) and +0xC8
namespace ts_extra_utilities::prism::offsets
{
    constexpr uint32_t game_actor_trailer = 0x00C8; // game_trailer_actor_u*, first trailer
    constexpr uint32_t trailer_slave = 0x1088; // game_trailer_actor_u*, next trailer in the chain
    constexpr uint32_t trailer_steering = 0x06B8; // float, -1..1, written by the game every advance
    constexpr uint32_t trailer_steering_data = 0x06C0; // vehicle_wheel_steering_data_t*, null when not steerable

    template < typename T >
    T& field( void* object, const uint32_t offset )
    {
        return *reinterpret_cast< T* >( reinterpret_cast< uint8_t* >( object ) + offset );
    }
}
