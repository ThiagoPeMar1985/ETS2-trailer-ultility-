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
   - `Insert` frees the mouse cursor to click the UI (fixed)
   - Per-trailer: lock checkbox, angle slider, left / center / right buttons

Original 1.53 features (detachable trailers, PhysX joint locks) are **disabled** in this fork —
their offsets have not been re-verified for 1.61.

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
