# ts-extra-utilities (ETS2 1.61 fork)

> [!WARNING]
> This is a Proof of Concept / Work In Progress. It is not extensively tested and may crash,
> especially after game updates. Singleplayer only — **NOT** recommended for multiplayer.

An updated fork of [dariowouters/ts-extra-utilities](https://github.com/dariowouters/ts-extra-utilities),
a plugin for ATS/ETS2 that adds experimental extra functionality.

This branch ports the trailer steering feature to **Euro Truck Simulator 2 / ATS 1.61** and adds
configurable keybinds so the steerable trailer axles can be controlled while driving, without
opening the plugin window.

## Current features

- **Manually steerable trailer wheels** (1.61 port)
   - Take control of the steerable wheels on each attached trailer individually
   - Detects all trailers and shows them in the UI
   - Works through `set_individual_steering`, so the game's own physics keeps running

- **Configurable keybinds**
   - Steer left / steer right / center steering / lock-unlock steering
   - Bind any key, including the numpad — press the action in the Keybinds window, then press the key
   - Once bound, everything works while driving: steer, smoothly return to center at the configured
     speed, and lock the axles straight — no need to open the plugin again
   - Bindings are saved to `%APPDATA%\ts-extra-utilities\settings.ini`

- **UI controls**
   - `Delete` opens/closes the plugin window (fixed)
   - `Insert` frees the mouse cursor (centered on screen) to click the UI (fixed)
   - Per-trailer: lock checkbox, angle slider, left / center / right buttons

- **Individually detachable trailers** (multi-trailer / rodotren support)
   - Detach and re-attach each trailer on its own — detach the last trailer at a
     dock, haul the rest, come back and pick it up
   - Per-trailer Connect/Disconnect buttons plus configurable keybinds:
     attach/detach trailer 1, 2 and 3 against its chain parent
   - **Attach trailer to truck**: connect a rear trailer straight to the fifth
     wheel while the rest of the set stays parked
   - Reconnected trailers release their brakes and roll normally — the plugin
     replays the game's full attach sequence (towed bookkeeping, parked flag,
     brace, air line rebuild, wheel load, coupling state)
   - Plugin-detached trailers are never auto-rejoined by the game's own
     attach/detach flow; the native `T` key keeps working for the first trailer
   - `g_trailer_cables_mode` follows the coupled convoy automatically: `1` while
     the first trailer is coupled, `2` once the last trailer is coupled, `0` with
     nothing attached
   - Per-trailer diagnostics (joint, brake channels, connector, towed pointer)
     dumped to `game.log.txt`

> [!NOTE]
> Trailer attach/detach is singleplayer only and assumes the truck is already
> positioned under the trailer's coupling point — the joint is created at the
> fixed hook, so align first.

## How to use

Currently only works with **DirectX 11**.

Copy `ts-extra-utilities.dll` to `<game_install_location>/bin/win_x64/plugins`
(create the `plugins` folder if it does not exist).

In game: press `Delete` to open the window, `Insert` to enable the mouse cursor. Open **Keybinds**
to bind the steering actions to the keys you want.

# ☕ Buy me a coffee

If this plugin helped you, you can support the work with a small donation — even $1 makes a difference and keeps the updates coming:

[![Buy me a coffee](https://img.shields.io/badge/Buy%20me%20a%20coffee-%E2%98%95-orange?style=for-the-badge)](https://wise.com/pay/r/EKyJzvBu89ivs4Q)

👉 **[Click here to donate via Wise](https://wise.com/pay/r/EKyJzvBu89ivs4Q)**

## Credits

- Original plugin: [dariowouters](https://github.com/dariowouters)
- 1.61 port and keybind system: this fork
