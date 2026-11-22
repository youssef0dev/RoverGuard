# Firmware core

Had dossier fih low-level C++ li ma-mrbotch b chip mo3ayan.

- `MotorHal`: driver dyal PWM/direction.
- `EncoderHal`: lecture dyal angles dyal roues.
- `RangeSensorHal`: lecture dyal front sensor.
- `DifferentialDriveController`: PID, timeout, w emergency stop.

## Desktop compilation test

```powershell
cmake -S firmware -B firmware\build
cmake --build firmware\build --config Release
```

Mlli nkhtaro ESP32 wla STM32, ghadi nzido implementation dyal had interfaces.
Math dyal kinematics w controller ma-khas-homch ytbdlo.

