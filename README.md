# MARIO (Minimal Articulated Robotic Intelligent Object) – Non-ROS Implementation

This project is a minimal version of [MARIO](https://github.com/SRA-VJTI/MARIO), a low-cost 5 DoF robotic arm developed by SRA-VJTI. The original version is built on ROS, but this version runs entirely without it. Instead, it uses direct serial communication with an ESP32 microcontroller to control the arm.

The goal was to make MARIO easier to run and test, especially for setups where ROS might be overkill or unavailable.

## What This Project Does

- Accepts input over serial to control MARIO
- Supports both direct joint angle control and inverse kinematics from XYZ coordinates
- Uses ESP32-WROOM-32E as the main controller
- Can be compiled with ESP-IDF or tested quickly through the Arduino IDE and serial monitor

## Hardware Requirements

- ESP32-WROOM-DA module
- MARIO robotic arm (or any compatible 5 DoF robotic arm)
- USB cable to connect to a PC

## Software Requirements

- Arduino IDE
- Alternatively: ESP-IDF v4.1.0 or later
- Serial Monitor (115200 baud rate)
- Correct board selected: `ESP32 WROOM DA Module`
- Correct COM port selected

## How to Use It

### Using Arduino IDE

1. Open Arduino IDE
2. Select `ESP32 WROOM DA Module` as the board
3. Choose the correct COM port
4. Set baud rate to `115200`
5. Upload one of the source files
6. Open the Serial Monitor

### Providing Input

Depending on the uploaded script, you can either send direct joint angles or XYZ coordinates:

#### Input Angles (`input_angles.c`)

Enter space-separated joint angles like:
30 60 90 20

Each number corresponds to the desired angle (in degrees) for a specific servo. The arm will move accordingly.

#### Input Coordinates (`input_coordiantes.c`)

Enter space-separated XYZ coordinates like:
12 25 7


The onboard inverse kinematics algorithm (`inverseKinematics.c`) will convert these coordinates into joint angles and move the arm.

## File Overview

| File | Description |
|------|-------------|
| `input_angles.c` | Accepts direct angle input from serial |
| `input_coordiantes.c` | Accepts (x, y, z) and uses IK to compute angles |
| `inverseKinematics.c` | Contains the inverse kinematics logic |
| `sweep_motion.c` | Sweeps the joints to test motion |
| `CMakeLists.txt` | Build instructions for ESP-IDF |
| `idf_component.yml` | Component manifest for ESP-IDF |

## Notes

- This version was built to simplify testing and control of the MARIO arm without setting up ROS.
- The Arduino Serial Monitor is sufficient for basic testing and input.
- You can also build and flash the code using ESP-IDF for more complex setups.

## Credits

This work is based on [MARIO by SRA-VJTI](https://github.com/SRA-VJTI/MARIO) and reimplemented to make it lighter, easier to run, and more accessible for basic use cases or hardware testing without ROS.

---

If you plan to expand this, add GUI control or a web socket, or reintroduce ROS later, this version can serve as a solid base to build from.


## This is what it looks like
[▶️ Watch the demo video](assets/demo.mp4)

I held the shoulder cause mujhe ek screw nahi mil raha tha 😭