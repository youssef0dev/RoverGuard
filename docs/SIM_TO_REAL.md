# Sim-to-Real roadmap

## Taba9a fo9aniya (Linux computer)

Raspberry Pi / Jetson / mini-PC:

- mission management
- A* and future SLAM
- camera and vision
- optional LLM
- logs and monitoring

## Taba9a ta7taniya (microcontroller)

ESP32 wla STM32:

- motor PWM and direction
- quadrature encoders
- wheel PID loop (100-500 Hz)
- range sensors
- emergency stop
- watchdog ila communication t9t3at

## Hardware li ghadi n7tajou

- 2 DC gear motors m3a encoders
- motor driver mnasb l current dyal motors
- ESP32 or STM32 board
- battery + fuse + regulator
- front range sensor (ToF/lidar/ultrasonic)
- real emergency-stop button
- optional IMU
- Linux computer ila bghina vision/LLM

## Deployment order

1. Compile C++ core f PC.
2. Khtar chip w motor driver.
3. Implement `MotorHal`, `EncoderHal`, `RangeSensorHal`.
4. Test one motor bla roues f l-ard.
5. Tune PID b speed sghira.
6. Verify watchdog and emergency stop.
7. Drive straight/rotate tests.
8. Link high-level planner over UART/CAN/USB or micro-ROS.
9. Calibrate wheel radius/separation f real robot.
10. Zid vision/LLM fo9 safety layer, machi blast-ha.

## Chno ma-khasnach ndiro

- LLM ma-ycommandich PWM direct.
- Ma-nrunniwch motors bla fuse/current limit.
- Ma-n3tamdouch 3la simulation constants bla calibration.
- Software emergency stop ma-y3ewedch hardware E-stop 7a9i9i.

