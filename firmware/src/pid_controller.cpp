#include "pid_controller.hpp"

#include <algorithm>

namespace roverguard {

PidController::PidController(float kp, float ki, float kd, float output_limit)
    : kp_(kp), ki_(ki), kd_(kd), output_limit_(output_limit) {}

float PidController::update(float target, float measured, float dt_seconds) {
  if (dt_seconds <= 0.0F) {
    return 0.0F;
  }

  const float error = target - measured;
  integral_ += error * dt_seconds;
  const float derivative = has_previous_
                               ? (error - previous_error_) / dt_seconds
                               : 0.0F;
  previous_error_ = error;
  has_previous_ = true;

  const float raw = kp_ * error + ki_ * integral_ + kd_ * derivative;
  const float limited = std::clamp(raw, -output_limit_, output_limit_);

  // Basic anti-windup: undo the last integration when output is saturated.
  if (limited != raw) {
    integral_ -= error * dt_seconds;
  }
  return limited;
}

void PidController::reset() {
  integral_ = 0.0F;
  previous_error_ = 0.0F;
  has_previous_ = false;
}

}  // namespace roverguard

