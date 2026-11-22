#include <iostream>

#include "drive_controller.hpp"

namespace {

class MockMotors final : public roverguard::MotorHal {
 public:
  void setLeftPwm(float value) override { left_ = value; }
  void setRightPwm(float value) override { right_ = value; }
  void stop() override {
    left_ = 0.0F;
    right_ = 0.0F;
  }
  float left_{0.0F};
  float right_{0.0F};
};

class MockEncoders final : public roverguard::EncoderHal {
 public:
  roverguard::EncoderState read() override { return state_; }
  roverguard::EncoderState state_{};
};

class MockRange final : public roverguard::RangeSensorHal {
 public:
  roverguard::RangeState readFront() override { return {1.0F, true}; }
};

}  // namespace

int main() {
  MockMotors motors;
  MockEncoders encoders;
  MockRange range;
  roverguard::DifferentialDriveController controller(
      motors, encoders, range, 0.05F, 0.23F, 0.12F);

  controller.setCommand({0.20F, 0.0F}, 0UL);
  controller.update(0.01F, 0UL);  // Initializes encoder history.
  encoders.state_ = {0.02F, 0.02F};
  controller.update(0.01F, 10UL);

  std::cout << "left_pwm=" << motors.left_
            << " right_pwm=" << motors.right_ << '\n';
  return 0;
}

