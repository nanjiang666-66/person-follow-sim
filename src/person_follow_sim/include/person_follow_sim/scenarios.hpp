#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace person_follow_sim
{
constexpr double pi = 3.14159265358979323846;
struct Target {double x{2.5}; double y{0}; double z{1.0}; bool detected{true}; bool depth{true};};
inline Target scenario(const std::string & name, double time)
{
  const auto t = std::max(0.0, time);
  Target p;
  if (name == "stationary") {return p;}
  if (name == "away") {p.x += 0.10 * std::clamp(t-5.0, 0.0, 35.0); return p;}
  if (name == "approach") {
    // After the initial 1.5 m stop, approach by 0.6 m while remaining in view.
    p.x -= 0.10 * std::clamp(t-10.0, 0.0, 6.0);
    p.x += 0.10 * std::clamp(t-30.0, 0.0, 25.0);
    return p;
  }
  if (name == "lateral") {p.y = 0.8*std::sin(0.10*std::max(0.0, t-5.0)); return p;}
  if (name == "turn") {
    const auto a = 0.10*std::clamp(t-5.0, 0.0, 25.0);
    p.x += std::sin(a); p.y = 1.0-std::cos(a); return p;
  }
  if (name == "occlusion" || name == "invalid_depth") {
    // Three distinct tests with one-, two-, and three-second interruptions.
    const auto hidden = (t>=12 && t<13) || (t>=22 && t<24) || (t>=34 && t<37);
    p.x += 0.06*std::clamp(t-10.0, 0.0, 35.0);
    if (name == "occlusion") {p.detected = !hidden;}
    p.depth = !hidden;
    return p;
  }
  if (name == "demo") {
    const auto cycle = t; // Hold the last pose after the demo; never teleport a target behind the robot.
    p.x += 0.10*std::clamp(cycle-5.0, 0.0, 15.0);
    p.y = 0.6*std::sin(0.10*std::clamp(cycle-20.0, 0.0, 30.0));
    p.detected = !(cycle>=35 && cycle<37);
    p.depth = p.detected && !(cycle>=45 && cycle<47);
    // A subsequent launch with scenario:=approach checks close-person stopping.
    return p;
  }
  throw std::invalid_argument("Unknown scenario: " + name);
}

struct Optical {double x; double y; double z;};
inline Optical to_optical(const Target & p, double robot_x, double robot_y,
  double robot_yaw, double camera_x, double camera_height)
{
  const auto dx=p.x-robot_x, dy=p.y-robot_y;
  const auto forward=std::cos(robot_yaw)*dx+std::sin(robot_yaw)*dy-camera_x;
  const auto left=-std::sin(robot_yaw)*dx+std::cos(robot_yaw)*dy;
  return {-left, camera_height-p.z, forward};
}
inline bool in_view(const Optical & p, double horizontal_fov, double vertical_fov)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && p.z>0.15 && p.z<8.0 &&
    std::abs(std::atan2(p.x,p.z)) < horizontal_fov/2 &&
    std::abs(std::atan2(p.y,p.z)) < vertical_fov/2;
}
}  // namespace person_follow_sim
