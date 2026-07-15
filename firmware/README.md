# firmware/

Low-level microcontroller firmware and calibration — not part of the ROS 2
graph. This folder lives at the workspace root, outside `src/`, so colcon
never scans it for packages.

## Goes here

- Microcontroller firmware (e.g. Arduino/STM32/ESP32 source) that gets
  flashed directly onto a board, not built or run by colcon
- Board-level calibration files (sensor offsets, PID tuning specific to the
  microcontroller, factory calibration constants)
- Bootloader/flashing configuration and scripts
- Any hardware-level config that doesn't go through a ROS 2 node, topic, or
  parameter


## Why it's separate from `src/`

Firmware here is built with a different toolchain (e.g. PlatformIO, Arduino
CLI, STM32CubeIDE) targeting a microcontroller.
Keeping it out of `src/` means `colcon build` never tries to interpret it as
a ROS package, and firmware changes don't trigger unrelated ROS package
rebuilds.