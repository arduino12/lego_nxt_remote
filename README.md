# NXT Remote

Phone remote for a LEGO MINDSTORMS NXT robot whose LCD is dead. It's a web page (PWA) that talks to the brick
over Bluetooth from **Chrome on Android**. The NXT keeps its stock LEGO firmware: the page sends LEGO *direct
commands*, so nothing has to be installed or started on the brick and no menu is needed at power-up.

- **Drive tab**: a joystick. Up/down runs motor A (forward/back). Left/right does one B+C turn per step, which moves the robot 5 cm sideways.
- **Path tab**: draw a route with your finger. It's snapped to forward/back runs (1 cm resolution) and whole 5 cm side steps, then driven with closed-loop position control.
- **Brick tab**:
  - The brick's screen, read from its display memory, with the 4 front-panel buttons as virtual buttons.
  - Side-wheel home setup and calibration.
  - Brick name, firmware, battery and Bluetooth address.

## Robot assumptions

- Motor **A** drives all 4 wheels forward/back.
- Motors **B + C** turn D-profile wheels mounted perpendicular to the drive direction. One full turn = one
  5 cm sideways step. Between steps they must rest at **home** (flat side down), so every side move ends on a
  multiple of 360° of the motor's rotation counter.
- The NXT zeroes its rotation counters at power-on. Switch it off with the D-wheels at home. Otherwise use
  *Brick → Side wheels home* to jog them and tap *Set home here*.
- Motor A won't drive while B/C are away from home. The page first turns them back to home.

## Why Bluetooth from a web page works

The NXT has Bluetooth 2.0 **Classic** (serial port profile) only: no BLE, no WiFi. Web Bluetooth can't reach it,
but Chrome's **Web Serial over Bluetooth RFCOMM** can. That's on desktop Chrome, and on Android since Chrome 138.
iPhone browsers can't do either.

## One-time setup

1. Turn Bluetooth **on** on the NXT (it's off by default). With a dead LCD, either:
   - USB + `tools/nxt_probe.py`: run `screen` and `press` to see the LCD as text and press buttons remotely. On Windows this needs WinUSB bound to the NXT once with [Zadig](https://zadig.akeo.ie/).
   - Blind, right after power-on (main menu starts at *My Files*): **◀ ◀ ◀** (→ Bluetooth), **●** (Bluetooth menu), **●** (On/Off), **●** (On).
     The main-menu loop is My Files → NXT Program → View → Bluetooth → Settings → Try Me.
2. Pair the phone in Android Bluetooth settings. The NXT shows up as `NXT` (or its brick name), PIN `1234`.
   When the brick beeps for the passkey, press **●** on the brick once.
3. Open the app (GitHub Pages URL) in Chrome, tap **Connect** and choose the NXT. On later visits it reconnects
   automatically.

Bluetooth on/off and pairings survive power cycles, so after this the brick only needs to be switched on.

## Development

```
node --test test/                     # path planner tests
cd app && python -m http.server 8000  # http://localhost:8000 is a secure context, so Web Serial works on the PC
python tools/nxt_probe.py --usb info  # or --port COMx (the brick's outgoing Bluetooth COM port)
```

- `app/nxt.js`: direct-command protocol (BT length-prefixed frames / USB bulk), IO-map screen and buttons.
- `app/motion.js`: absolute-target motor control, B/C home interlock, joystick deadman, soft-limit fallback.
- `app/path.js`: stroke → move list.
- `app/app.js`: UI, Hebrew/English.
