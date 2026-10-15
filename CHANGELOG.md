1.3.7
    - A trailer connected straight to the truck now marks the truck's coupling
      state (truck physics +0x14E0) as fully coupled: without it the per-frame
      trailer loop kept spring-braking the jointed trailer and the truck never
      broadcast its brake state down the chain
    - The parked set keeps re-applying its spring brake through the shared
      trailer chain even to a trailer sitting on the fifth wheel; the brake
      setter is now hooked and positive writes aimed at a truck-towed trailer
      are swallowed, so its wheels stay free while the pedal brake still works
    - g_trailer_cables_mode follows the convoy automatically: 1 while the first
      trailer is coupled, 2 once the last trailer is coupled, 0 with nothing
    - Overlay font slightly enlarged for 4K readability
    - Diagnostic dump per trailer (joint, brake channels, connector, truck towed
      pointer) plus brake setter call tracing in game.log.txt

1.3.6
    - Manual connect now releases the trailer brake channel directly: the game's
      per-frame update writes trailer+0x760 = 1.0 while the joint is missing and
      nothing zeroed it on our attach, leaving the wheels locked. Connect clears
      +0x760 and the force-brake byte +0x764; disconnect writes the same state the
      game applies to a jointless trailer (+0x760 = 1.0, +0x730 = 0)

1.3.5
    - connect_slave is now permanently hooked but conditional: it only refuses to
      auto-attach a slave trailer when that trailer was detached through this
      plugin. Native flows (spawn, T-key attach) keep working, and the game no
      longer rejoins plugin-detached trailers when the head trailer is detached
      with T
    - A trailer connected straight to the truck keeps its detach mark, so its
      chain parent cannot pull it back through connect_slave
    - Connecting to the chain parent clears the mark and returns the trailer to
      native management; stale marks are cleaned up every frame

1.3.4
    - Reverted the overlay font to the stock size
    - Fixed a crash: the truck-side bookkeeping helpers (towed pointer, wheel load)
      were also running with a trailer as the parent, where those offsets hold
      different fields — they now run for truck parents only

1.3.3
    - Manual connect now replays the full native attach sequence: towed-trailer
      bookkeeping on the parent, detached-state flag, parked flag, air/electric
      line rebuild and wheel-load distribution — a manually connected trailer no
      longer keeps its spring brakes applied
    - Manual disconnect mirrors the native detach: parked flag set, parent's
      bookkeeping/wheel load cleared, lines rebuilt

1.3.2
    - Connect now clears the trailer's "parked" flag (+0x1198) like the native attach
      does; a manually connected trailer no longer drags with its wheels locked
    - Overlay font rendered at 2x with matching spacing for 4K readability
    - The cursor now appears centered on screen when released with Insert

1.3.1
    - New keybinds: Attach / detach trailer 1-3 (one key toggles against the chain parent)
      and Attach trailer 2/3 to truck (connects that trailer straight to the fifth wheel)
    - "Connect to truck" button per detached trailer behind the first; refused while the
      fifth wheel is occupied so two trailers can never share one joint point

1.3.0
    - Individually detachable trailers are back, ported to 1.61.1.9: per-trailer
      Connect/Disconnect buttons in the Trailer Manipulation window. Disconnect drops the
      landing gear and the joint; Connect re-attaches the trailer to whatever is ahead of
      it in the chain (truck for the first trailer). Back the truck close to the parked
      trailer before reconnecting. The null-joint guard is only bypassed while a trailer
      is actually detached, so normal play keeps the native checks. Singleplayer only.

1.2.2
    - Trailer steering offsets updated for game version 1.61.1.9 (the trailer object grew by
      0x28 bytes: steering angle 0x690 -> 0x6B8, steering data 0x698 -> 0x6C0, slave trailer
      0x1060 -> 0x1088; first trailer in game_actor is still 0xC8)

1.2.0
    - Configurable keybinds (Keybinds window), saved to %APPDATA%\ts-extra-utilities\settings.ini
    - New hotkeys: steer left / right, center, lock / unlock trailer steering (unbound by default)
    - Adjustable steering speed for the hotkeys

1.1.1
    - DirectX 11 Present is now inline-hooked (plus Present1) instead of vtable-swapped, so the overlay also shows on flip-model swapchains
    - Logs "Present hook is live, overlay initialized" once the overlay is up

1.1.0
    - Updated manual trailer steering for game version 1.61 (ETS2 + ATS)
    - Trailer steering now only overrides the steering angle instead of skipping the whole steering update
    - Detachable trailers and lockable joints are disabled until their offsets are re-verified

1.0.1
    - Fixed DirectX hook not working on some devices

1.0.0
    - Initial Version
