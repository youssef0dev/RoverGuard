#pragma once

namespace roverguard {

struct TwistCommand {
  float linear_m_s{0.0F};
  float angular_rad_s{0.0F};
};

struct WheelVelocityCommand {
  float left_rad_s{0.0F};
  float right_rad_s{0.0F};
};

struct EncoderState {
  float left_angle_rad{0.0F};
  float right_angle_rad{0.0F};
};

struct RangeState {
  float front_distance_m{0.0F};
  bool valid{false};
};

inline WheelVelocityCommand twistToWheels(
    const TwistCommand& command,
    float wheel_radius_m,
    float wheel_separation_m) {
  const float half_track = wheel_separation_m * 0.5F;
  return {
      (command.linear_m_s - command.angular_rad_s * half_track) /
          wheel_radius_m,
      (command.linear_m_s + command.angular_rad_s * half_track) /
          wheel_radius_m,
  };
}

}  // namespace roverguard

