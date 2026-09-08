# Balance Bot

Arduino code for a two-wheel self-balancing robot using an MPU6050, two stepper motors, and PID control.

## Demo

![Balancing robot demo](balancing_demo_vid-ezgif.com-video-to-gif-converter.gif)

## Schematic

![Balance bot schematic](Schematic_Balance-Car_2026-09-08%20(2).png)

## Hardware

- Arduino-compatible board
- MPU6050 accelerometer and gyroscope
- Two stepper motors with stepper drivers
- Two wheels and a robot chassis

## Libraries

- AccelStepper
- AltSoftSerial
- Wire
- I2Cdev
- MPU6050

Upload `balance_bot.ino` to the board after installing the required libraries and wiring the components according to the schematic.