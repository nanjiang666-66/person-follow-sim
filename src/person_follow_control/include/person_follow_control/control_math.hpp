#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace person_follow_control
{
constexpr double pi = 3.14159265358979323846;
struct Command { double linear{0.0}; double angular{0.0}; };
struct Config
{
  double desired_distance{1.5};
  double stop_margin{0.02};
  double resume_margin{0.12};
  double turn_enter{0.30};
  double turn_exit{0.12};
  double angle_deadband{0.05};
  double angular_gain{1.6};
  double distance_gain{0.6};
  double max_linear{0.20};
  double max_angular{0.60};
  double max_linear_acceleration{0.35};
  double max_angular_acceleration{1.2};
  double point_timeout{0.5};
  double receipt_timeout{0.5};
  double raw_timeout{0.3};
  double odom_timeout{0.5};
  double future_tolerance{0.05};
  double recovery_duration{0.20};
  int recovery_frames{3};
};

inline void validate(const Config & c)
{
  const double values[] = {c.desired_distance, c.stop_margin, c.resume_margin,
    c.turn_enter, c.turn_exit, c.angle_deadband, c.angular_gain, c.distance_gain,
    c.max_linear, c.max_angular, c.max_linear_acceleration, c.max_angular_acceleration,
    c.point_timeout, c.receipt_timeout, c.raw_timeout, c.odom_timeout,
    c.future_tolerance, c.recovery_duration};
  for (double value : values) {
    if (!std::isfinite(value) || value < 0.0) {throw std::invalid_argument("Invalid follow parameter");}
  }
  if (c.desired_distance <= 0.3 || c.resume_margin <= c.stop_margin ||
    c.turn_enter <= c.turn_exit || c.turn_exit <= c.angle_deadband ||
    c.turn_enter >= pi / 2 || c.max_linear <= 0 || c.max_angular <= 0 ||
    c.angular_gain <= 0 || c.distance_gain <= 0 || c.max_linear_acceleration <= 0 ||
    c.max_angular_acceleration <= 0 || c.point_timeout <= 0 || c.receipt_timeout <= 0 ||
    c.raw_timeout <= 0 || c.odom_timeout <= 0 || c.recovery_frames < 1)
  {
    throw std::invalid_argument("Inconsistent follow thresholds");
  }
}

struct Input
{
  bool emergency_stop{false};
  bool clock_valid{false};
  bool clock_advancing{true};
  bool visible{false};
  bool target_fresh{false};
  bool odom_fresh{false};
  bool pose_valid{false};
  bool confirmed{false};
  bool action_active{false};
  bool raw_fresh{false};
  double x{0};
  double y{0};
  double camera_bearing{0};
  Command raw{};
};
struct Decision
{
  Command command{};
  std::string state;
  bool stop_immediately{false};
};

inline bool fresh_age(double age, double timeout, double future_tolerance = 0.0)
{
  return std::isfinite(age) && age >= -future_tolerance && age <= timeout;
}

inline double slew(double value, double previous, double acceleration, double dt)
{
  const auto bound = acceleration * std::clamp(dt, 0.0, 0.1);
  return std::clamp(value, previous - bound, previous + bound);
}

class Gate
{
public:
  explicit Gate(Config config) : c_(config) {validate(c_);}
  void reset() {forward_enabled_ = false; aligning_ = false; previous_ = {};}
  Decision step(const Input & in, double dt)
  {
    auto stop = [this](const std::string & reason) {
        reset();
        return Decision{{}, reason, true};
      };
    if (in.emergency_stop) {return stop("emergency_stop");}
    if (!in.clock_valid || !in.clock_advancing) {return stop("clock_unavailable_or_paused");}
    if (!in.odom_fresh) {return stop("odom_unavailable");}
    if (!in.visible) {return stop("target_not_reliable");}
    if (!in.target_fresh) {return stop("target_stale");}
    if (!in.pose_valid || !std::isfinite(in.x) || !std::isfinite(in.y) ||
      !std::isfinite(in.camera_bearing) || in.x <= 0.0)
    {
      return stop("invalid_target_or_tf");
    }
    if (!in.confirmed) {return stop("confirming_target");}

    const auto distance = std::hypot(in.x, in.y);
    const auto angle = in.camera_bearing;
    if (distance <= c_.desired_distance + c_.stop_margin) {forward_enabled_ = false;}
    if (distance >= c_.desired_distance + c_.resume_margin) {forward_enabled_ = true;}
    if (std::abs(angle) >= c_.turn_enter) {aligning_ = true;}
    if (std::abs(angle) <= c_.turn_exit) {aligning_ = false;}

    // Optical x points right: negative yaw turns toward a person on the right.
    const auto turn = std::abs(angle) < c_.angle_deadband ? 0.0 :
      std::clamp(-c_.angular_gain * angle, -c_.max_angular, c_.max_angular);
    Command desired{};
    std::string state;
    bool inhibit_forward = false;
    if (!forward_enabled_) {
      desired.angular = turn;
      state = std::abs(turn) > 0 ? "hold_distance_align" : "hold_distance";
      inhibit_forward = true;
    } else if (aligning_) {
      desired.angular = turn;
      state = "align_target";
      inhibit_forward = true;
    } else {
      if (!in.action_active) {return stop("waiting_follow_server");}
      if (!in.raw_fresh || !std::isfinite(in.raw.linear) || !std::isfinite(in.raw.angular)) {
        return stop("controller_stale");
      }
      desired.linear = std::clamp(in.raw.linear, 0.0,
        std::min(c_.max_linear, c_.distance_gain * std::max(0.0, distance - c_.desired_distance)));
      desired.angular = std::clamp(in.raw.angular, -c_.max_angular, c_.max_angular);
      state = "following";
    }
    // Inhibiting forward motion bypasses ramp-down, just like an invalid target.
    previous_.linear = inhibit_forward ? 0.0 :
      slew(desired.linear, previous_.linear, c_.max_linear_acceleration, dt);
    previous_.angular = slew(desired.angular, previous_.angular, c_.max_angular_acceleration, dt);
    return {previous_, state, false};
  }
private:
  Config c_;
  bool forward_enabled_{false};
  bool aligning_{false};
  Command previous_{};
};
}  // namespace person_follow_control
