#include "person_follow_control/control_math.hpp"
#include <cstdlib>
#include <iostream>

using namespace person_follow_control;
int checks = 0;
void expect(bool result, const char * label)
{
  ++checks;
  if (!result) {std::cerr << "FAIL: " << label << '\n'; std::exit(1);}
}
Input valid()
{
  Input in;
  in.clock_valid = in.visible = in.target_fresh = in.odom_fresh = in.pose_valid =
    in.confirmed = in.action_active = in.raw_fresh = true;
  in.x = 2.5;
  in.raw = {0.5, 0.0};
  return in;
}
int main()
{
  Config c;
  Gate gate(c);
  auto in = valid();
  auto out = gate.step(in, 0.02);
  expect(out.command.linear > 0 && out.command.linear <= 0.00701, "startup acceleration");
  for (int i = 0; i < 100; ++i) {out = gate.step(in, 0.02);}
  expect(std::abs(out.command.linear - 0.2) < 1e-9, "maximum speed");
  in.x = 1.0;
  out = gate.step(in, 0.02);
  expect(out.command.linear == 0 && out.state == "hold_distance", "approaching person inhibits forward immediately");
  in.x = 1.58;
  expect(gate.step(in, 0.02).command.linear == 0, "distance hysteresis holds");
  in.x = 1.7;
  expect(gate.step(in, 0.02).command.linear > 0, "resume after moving away");
  in = valid(); in.camera_bearing = 0.4;
  out = gate.step(in, 0.1);
  expect(out.command.linear == 0 && out.command.angular < 0, "right target turns clockwise before driving");
  in.camera_bearing = 0.2;
  expect(gate.step(in, 0.02).command.linear == 0, "angle hysteresis holds");
  in.camera_bearing = 0.1;
  expect(gate.step(in, 0.02).command.linear > 0, "resume after alignment");
  gate.reset(); in = valid(); in.x = 1.2; in.camera_bearing = -0.4;
  out = gate.step(in, 0.1);
  expect(out.command.linear == 0 && out.command.angular > 0, "close target may turn left without forward motion");
  auto expect_stop = [&](Input sample, const char * name) {
      gate.reset(); gate.step(valid(), 0.1);
      const auto result = gate.step(sample, 0.02);
      expect(result.command.linear == 0 && result.command.angular == 0 && result.stop_immediately, name);
    };
  in = valid(); in.visible = false; expect_stop(in, "invalid depth stop");
  in = valid(); in.target_fresh = false; expect_stop(in, "target timeout stop");
  in = valid(); in.odom_fresh = false; expect_stop(in, "odometry timeout stop");
  in = valid(); in.clock_advancing = false; expect_stop(in, "paused clock stop");
  in = valid(); in.clock_valid = false; expect_stop(in, "uninitialized clock stop");
  in = valid(); in.confirmed = false; expect_stop(in, "recovery confirmation stop");
  in = valid(); in.emergency_stop = true; expect_stop(in, "emergency stop");
  in = valid(); in.x = -1; expect_stop(in, "target behind robot stop");
  in = valid(); in.x = std::numeric_limits<double>::quiet_NaN(); expect_stop(in, "invalid coordinates stop");
  in = valid(); in.raw_fresh = false; expect_stop(in, "controller timeout stop");
  in = valid(); in.action_active = false; expect_stop(in, "server unavailable stop");
  in = valid(); in.raw.angular = std::numeric_limits<double>::infinity(); expect_stop(in, "invalid velocity stop");
  gate.reset(); in = valid(); in.raw.linear = -1;
  expect(gate.step(in, 0.02).command.linear == 0, "reverse prohibited");
  gate.reset(); in.raw = {0.1, 5};
  for (int i = 0; i < 100; ++i) {out = gate.step(in, 0.02);}
  expect(out.command.angular <= c.max_angular, "angular speed clamp");
  expect(fresh_age(0.4, 0.5), "fresh sample");
  expect(!fresh_age(0.6, 0.5), "old sample rejected");
  expect(!fresh_age(-0.1, 0.5, 0.05), "future timestamp rejected");
  expect(!fresh_age(std::numeric_limits<double>::quiet_NaN(), 0.5), "invalid timestamp rejected");
  Config bad = c; bad.resume_margin = bad.stop_margin;
  bool threw = false; try {validate(bad);} catch (const std::invalid_argument &) {threw = true;}
  expect(threw, "invalid configuration rejected");
  std::cout << checks << " control checks passed\n";
}
