#pragma once

#include "robot_types.hpp"

namespace roverguard {

class MotorHal {
 public:
  virtual ~MotorHal() = default;
  virtual void setLeftPwm(float normalized_pwm) = 0;
  virtual void setRightPwm(float normalized_pwm) = 0;
  virtual void stop() = 0;
};

class EncoderHal {
 public:
  virtual ~EncoderHal() = default;
  virtual EncoderState read() = 0;
};

class RangeSensorHal {
 public:
  virtual ~RangeSensorHal() = default;
  virtual RangeState readFront() = 0;
};

class ClockHal {
 public:
  virtual ~ClockHal() = default;
  virtual unsigned long milliseconds() const = 0;
};

}  // namespace roverguard

