# Crunchlanbs HackPack Balance Bot Laser Tag Target Hack

Transform your HackPack Balance Bot into an interactive laser tag target! This project enhances the stock balance bot by adding IR laser tag detection, dramatic hit reactions, and game-inspired LED effects—all while preserving its core self-balancing functionality.

## Features

- **Laser Tag Gameplay**: The robot reacts to being shot with standard NEC IR protocol laser tag guns.
- **Hit Reactions**: Each hit triggers a backward "recoil" movement and heartbeat-style red LED effects.
- **Damage Indicators**: 2 LEDs light up on the NeoPixel ring for each hit (up to 6 hits).
- **Progressive Heartbeat**: Heartbeat LED effect speeds up as the bot takes damage, following a realistic "lub-dub-pause" cycle.
- **Death Sequence**:
  - **Phase 1**: 3 seconds of dramatic staggering and spinning.
  - **Phase 2**: 2 seconds of falling over (motors disabled), all LEDs blink red rapidly.
  - After 5 seconds, the bot resets and returns to normal rainbow LED mode.
- **Debounced IR Detection**: Prevents multiple hits from a single shot.
- **Maintains Core Balance Bot Functionality**: PID-based self-balancing remains active throughout.

## Hardware Requirements

- **HackPack Balance Bot** (with stock hardware)
- **IR Receiver (3-wire)**: Connected to Arduino pin 4
- **NeoPixel Ring (12 LEDs, CJMCU-2812B-12)**: Connected to pin 2

## How It Works

1. **Startup**: The bot displays a rainbow LED cycle on both the ring and bar.
2. **Receiving Hits**:
    - Each valid IR laser tag hit triggers:
        - A dramatic backward lean ("recoil").
        - 2 more NeoPixel ring LEDs light up red.
        - Heartbeat LED effect pulses faster with each hit.
    - After 6 hits, the bot enters a dramatic "death sequence":
        - 3 seconds of staggering/spinning, then 2 seconds of falling over.
        - All LEDs blink red rapidly.
        - After 5 seconds, the bot resets to rainbow mode.
        - 
## Getting Started

1. **Clone the Repository**

    ```sh
    git clone https://github.com/multiplexed/HackPackBalanceBotLaserTagTargetHack.git
    ```

2. **Install Arduino Libraries**

    - [Adafruit NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel)
    - [IRremote](https://github.com/Arduino-IRremote/Arduino-IRremote)
    - [NewPing](https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home)
    - [I2Cdevlib](https://github.com/jrowberg/i2cdevlib)
    - [MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
    - [PID_v1_bc](https://playground.arduino.cc/Code/PIDLibrary/)

3. **Wire Your Hardware**

    | Component             | Arduino Pin |
    |-----------------------|-------------|
    | IR Receiver           | 4           |
    | NeoPixel Ring         | 2           |


4. **Upload the Code**

    - Open `BalanceBotLaserTagTarget.ino` in the Arduino IDE.
    - Select the correct board (e.g., Arduino Nano).
    - Upload the sketch.

## Customization

- **PID Tuning**: Adjust `kP`, `kI`, and `kD` in the code for your bot's response.
- **Dynamic Setpoint**: Toggle `useDynamicSetpoint` for different balancing behavior.
- **Serial Output**: Enable `useSerial` for debugging.

## License

This project is licensed under the MIT License. See the code comments for full details.

---

**Credits**:  
- Code and hardware design by Crunchlabs LLC and contributors.  
- Inspired by the HackPack Balance Bot platform.

---

Feel free to further edit or expand this README to fit your project's needs!
