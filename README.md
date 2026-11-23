# CD PRINTS CAR — Lafvin 2WD Smart Robot Car

A complete autonomous and telemetry-driven 2WD Smart Robot Car built on Arduino (ATmega328P) with JDY-16 Bluetooth Low Energy (BLE), dual Web Bluetooth / Web Serial web applications, and an interactive Shelby Cobra / Motorsport Carbon cockpit phone controller.

## Features

- **Firmware (`lafvin_2wd_car_standalone.ino`)**:
  - Automatic AT command rename for onboard JDY-16 BLE module to `"CD PRINTS CAR"`.
  - Strict `'!'`-framed serial remote command protocol that isolates Bluetooth boot noise (`+MAC:...`, `+READY`, `+CONNECTED`).
  - Zero-wiring battery health telemetry (`readBatteryMv()`) using the ATmega328P internal 1.1V bandgap with optional 2S divider on pin A4.
  - Multi-mode autonomous behaviors: Obstacle Avoidance (SG90 servo sweep), Line Tracking with smart edge recovery scan, Object Follow mode, and Light Seeking.
  - Configurable motor direction inversion constants to easily accommodate hardware/wiring adjustments.
  - Dynamic PWM speed scaling across all movements.

- **Motorsport Cockpit Phone Controller (`phone_controller.html`)**:
  - Works directly in browser via Web Bluetooth API (BLE) or Web Serial API (USB OTG).
  - Motorsport carbon-fiber weave and satin anodized crimson dash styling inspired by Shelby Cobra & GT racing cockpits.
  - **Car Mode**: Realistic 360° rotating racing steering wheel (`steering_wheel.png`, continuous $-135^\circ$ to $+135^\circ$), 5-speed gearbox with Reverse & Neutral (`R`, `N`, `1st`–`5th`), and multi-touch Gas & Brake pedals.
  - **Default Mode**: 8-way directional virtual joystick / D-Pad with instant one-tap toggle.
  - Luminous analog cluster: Speedometer with gear indicator, Fuel/Battery health gauge with low-voltage warning LED, and Sonar radar gauge.
  - 4 rotary dash knobs (`OBST`, `SWEEP`, `FLIP`, `INV LN`) with neon status halos.
  - Fullscreen landscape lock and industrial red/black hazard-stripe `STOP / BRAKE` bar.

- **PC Remote Dashboard (`pc_remote (5).html`)**:
  - Desktop-oriented keyboard, mouse, and telemetry interface with live serial monitor, tuning sliders, and sensor tests.

## File Structure

```
.
├── lafvin_2wd_car_standalone.ino   # Arduino firmware
├── phone_controller.html           # Phone cockpit web app (BLE / USB)
├── steering_wheel.png              # Transparent steering wheel asset
├── pc_remote (5).html              # PC remote control web app
├── .vscode/
│   └── c_cpp_properties.json       # C/C++ editor configuration
├── .gitignore                      # Git ignore rules
└── README.md                       # Project documentation
```

## Quick Start

### 1. Upload Firmware
1. Open `lafvin_2wd_car_standalone.ino` in Arduino IDE.
2. Select Board: **Arduino Uno** (or **Nano** w/ ATmega328P Old Bootloader).
3. Connect USB cable and click **Upload**.
4. Disconnect USB when running on battery.

### 2. Connect via Phone Controller
1. Open `phone_controller.html` on your mobile browser (Chrome for Android recommended).
2. Tap **Bluetooth** -> **Connect**.
3. Select **CD PRINTS CAR** from the pairing window.
4. Drive with the steering wheel, shifter, and pedals!
