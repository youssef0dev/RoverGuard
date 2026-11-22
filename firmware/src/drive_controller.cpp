#include "drive_controller.hpp"

namespace roverguard {

DifferentialDriveController::DifferentialDriveController(
    MotorHal& motors,
    EncoderHal& encoders,
    RangeSensorHal& range_sensor,
    float wheel_radius_m,
    float wheel_separation_m,
    float emergency_distance_m)
    : motors_(motors),
      encoders_(encoders),
      range_sensor_(range_sensor),
      wheel_radius_m_(wheel_radius_m),
      wheel_separation_m_(wheel_separation_m),
      emergency_distance_m_(emergency_distance_m) {}

void DifferentialDriveController::setCommand(
    const TwistCommand& command, unsigned long now_ms) {
  command_ = command;
  last_command_ms_ = now_ms;
}

void DifferentialDriveController::update(
    float dt_seconds, unsigned long now_ms) {
  const RangeState range = range_sensor_.readFront();
  const bool command_timed_out = now_ms - last_command_ms_ > kCommandTimeoutMs;
  const bool obstacle_emergency =
      range.valid && range.front_distance_m <= emergency_distance_m_ &&
      command_.linear_m_s > 0.0F;

  if (command_timed_out || obstacle_emergency) {
    emergencyStop();
    return;
  }

  const EncoderState current = encoders_.read();
  if (!initialized_) {
    previous_encoder_ = current;
    initialized_ = true;
    return;
  }

  const float measured_left =
      (current.left_angle_rad - previous_encoder_.left_angle_rad) / dt_seconds;
  const float measured_right =
      (current.right_angle_rad - previous_encoder_.right_angle_rad) / dt_seconds;
  previous_encoder_ = current;

  const WheelVelocityCommand target = twistToWheels(
      command_, wheel_radius_m_, wheel_separation_m_);

  motors_.setLeftPwm(
      left_pid_.update(target.left_rad_s, measured_left, dt_seconds));
  motors_.setRightPwm(
      right_pid_.update(target.right_rad_s, measured_right, dt_seconds));
}

void DifferentialDriveController::emergencyStop() {
  command_ = {};
  left_pid_.reset();
  right_pid_.reset();
  motors_.stop();
}

}  // namespace roverguard

