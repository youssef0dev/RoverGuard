#pragma once

namespace roverguard {

class PidController {
 public:
  PidController(float kp, float ki, float kd, float output_limit);

  float update(float target, float measured, float dt_seconds);
  void reset();

 private:
  float kp_;
  float ki_;
  float kd_;
  float output_limit_;
  float integral_{0.0F};
  float previous_error_{0.0F};
  bool has_previous_{false};
};

}  // namespace roverguard

