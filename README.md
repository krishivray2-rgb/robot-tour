# Robot Tour

Bare-metal C++ firmware for a Tektite SciOly Robot Tour competition robot
on the RP2350.

## What it does

The robot autonomously executes a programmed path on a grid course, hitting
each target waypoint with precise position and timing.

## Control stack

- **Odometry:** quadrature-encoder dead reckoning with runtime-calibrated
  wheel-radius constants
- **Heading control:** PD loop with gain scheduling; SPI-based IMU for yaw estimate
- **Gyro bias calibration:** runtime auto-calibration on startup
- **Motion profile:** trapezoidal speed profile with separate accel / cruise /
  decel phases
- **Drift correction:** lateral steering-gain interpolation, smoothed near
  end-of-track to eliminate abrupt corrections

## Hardware

- **MCU:** Raspberry Pi Pico 2 (RP2350) on TektiteRotEv platform
- **Sensors:** quadrature encoders (×2), SPI IMU
- **Drive:** differential drive, two DC motors with H-bridge

## License

MIT
