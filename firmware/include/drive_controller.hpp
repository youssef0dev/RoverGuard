#pragma once

#include "hardware_hal.hpp"
#include "pid_controller.hpp"
#include "robot_types.hpp"

namespace roverguard {

class DifferentialDriveController {
 public:
  DifferentialDriveController(
      MotorHal& motors,
      EncoderHal& encoders,
      RangeSensorHal& range_sensor,
      float wheel_radius_m,
      float wheel_separation_m,
      float emergency_distance_m);

  void setCommand(const TwistCommand& command, unsigned long now_ms);
  void update(float dt_seconds, unsigned long now_ms);
  void emergencyStop();

 private:
  static constexpr unsigned long kCommandTimeoutMs = 500UL;

  MotorHal& motors_;
  EncoderHal& encoders_;
  RangeSensorHal& range_sensor_;
  float wheel_radius_m_;
  float wheel_separation_m_;
  float emergency_distance_m_;
  TwistCommand command_{};
  EncoderState previous_encoder_{};
  unsigned long last_command_ms_{0UL};
  bool initialized_{false};
  PidController left_pid_{0.30F, 0.08F, 0.002F, 1.0F};
  PidController right_pid_{0.30F, 0.08F, 0.002F, 1.0F};
};

}  // namespace roverguard

