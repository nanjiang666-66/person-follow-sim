#include "person_follow_control/control_math.hpp"

#include <chrono>
#include <csignal>
#include <memory>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "opennav_following_msgs/action/follow_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;
namespace pfc = person_follow_control;
using FollowObject = opennav_following_msgs::action::FollowObject;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowObject>;
static volatile std::sig_atomic_t interrupted = 0;
static void handle_signal(int) {interrupted = 1;}

class FollowSupervisor : public rclcpp::Node
{
public:
  FollowSupervisor() : Node("follow_supervisor")
  {
    config_.desired_distance = startup_parameter("desired_distance", 1.5);
    config_.stop_margin = startup_parameter("stop_margin", 0.02);
    config_.resume_margin = startup_parameter("resume_margin", 0.12);
    config_.turn_enter = startup_parameter("turn_enter", 0.30);
    config_.turn_exit = startup_parameter("turn_exit", 0.12);
    config_.angle_deadband = startup_parameter("angle_deadband", 0.05);
    config_.angular_gain = startup_parameter("angular_gain", 1.6);
    config_.distance_gain = startup_parameter("distance_gain", 0.6);
    config_.max_linear = startup_parameter("max_linear", 0.20);
    config_.max_angular = startup_parameter("max_angular", 0.60);
    config_.max_linear_acceleration = startup_parameter("max_linear_acceleration", 0.35);
    config_.max_angular_acceleration = startup_parameter("max_angular_acceleration", 1.2);
    config_.point_timeout = startup_parameter("point_timeout", 0.5);
    config_.receipt_timeout = startup_parameter("receipt_timeout", 0.5);
    config_.raw_timeout = startup_parameter("raw_timeout", 0.3);
    config_.odom_timeout = startup_parameter("odom_timeout", 0.5);
    config_.future_tolerance = startup_parameter("future_tolerance", 0.05);
    config_.recovery_duration = startup_parameter("recovery_duration", 0.20);
    config_.recovery_frames = startup_parameter("recovery_frames", 3);
    base_frame_ = startup_parameter<std::string>("base_frame", "base_footprint");
    fixed_frame_ = startup_parameter<std::string>("fixed_frame", "odom");
    camera_frame_ = startup_parameter<std::string>("camera_frame", "camera_color_optical_frame");
    declare_parameter("emergency_stop", false);
    gate_ = std::make_unique<pfc::Gate>(config_);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    // Humble Buffer::transform() checks for a dedicated TF thread even with its
    // default zero timeout. The listener owns a separate callback group/executor;
    // perception and command callbacks remain in the node's single-thread executor.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/follow/person_pose", 10);
    base_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/follow/target_base", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/follow/state", 10);
    point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/person/target_point", 10,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr point) {receive_point(*point);});
    visible_sub_ = create_subscription<std_msgs::msg::Bool>("/person/visible", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr value) {
        visible_ = value->data; visible_receipt_ = wall_now();
        if (!visible_) {clear_confirmation(); raw_received_ = false;}
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr odom) {
        odom_receipt_ = wall_now();
        odom_stamp_ = rclcpp::Time(odom->header.stamp, get_clock()->get_clock_type()).nanoseconds();
        const auto & q = odom->pose.pose.orientation;
        const auto norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        const auto roll = std::atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y));
        const auto pitch = std::asin(std::clamp(2*(q.w*q.y-q.z*q.x), -1.0, 1.0));
        odom_received_ = std::isfinite(norm) && std::abs(norm - 1.0) < 0.1 &&
          std::isfinite(odom->pose.pose.position.x) && std::isfinite(odom->pose.pose.position.y) &&
          std::abs(roll) < 20*pfc::pi/180 && std::abs(pitch) < 20*pfc::pi/180;
      });
    raw_sub_ = create_subscription<geometry_msgs::msg::Twist>("/follow/cmd_vel_raw", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr command) {
        raw_ = {command->linear.x, command->angular.z};
        raw_receipt_ = wall_now(); raw_received_ = true;
      });
    action_client_ = rclcpp_action::create_client<FollowObject>(this, "/follow_object");
    last_tick_ = clock_progress_wall_ = wall_now();
    // Wall timer keeps publishing STOP if /clock, perception or the controller stops.
    timer_ = create_wall_timer(20ms, [this] {tick();});
    RCLCPP_INFO(get_logger(), "Follow distance %.2f m; final speed topic %s belongs to supervisor",
      config_.desired_distance, command_pub_->get_topic_name());
  }

  void stop()
  {
    stopping_ = true;
    cancel_action();
    command_pub_->publish(geometry_msgs::msg::Twist());
  }

private:
  template<typename T>
  T startup_parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Startup setting: change launch/YAML and restart both following nodes.";
    return declare_parameter<T>(name, value, descriptor);
  }
  static double wall_now()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void clear_confirmation()
  {
    confirmation_count_ = 0; confirmation_start_ = 0; confirmed_ = false;
  }
  void receive_point(const geometry_msgs::msg::PointStamped & point)
  {
    const auto stamp = rclcpp::Time(point.header.stamp, get_clock()->get_clock_type()).nanoseconds();
    if (stamp <= point_stamp_ || stamp <= 0) {return;} // Never count a replayed sample as a new confirmation.
    point_stamp_ = stamp;
    point_receipt_ = wall_now();
    pose_valid_ = false;
    if (point.header.frame_id.empty() || !std::isfinite(point.point.x) ||
      !std::isfinite(point.point.y) || !std::isfinite(point.point.z)) {return;}
    try {
      auto optical = point;
      if (point.header.frame_id != camera_frame_) {optical = tf_buffer_->transform(point, camera_frame_);}
      if (optical.point.z <= 0.15 || optical.point.z > 8.0) {return;}
      target_base_ = tf_buffer_->transform(point, base_frame_);
      const auto fixed_point = tf_buffer_->transform(point, fixed_frame_);
      pose_.header = fixed_point.header;
      pose_.pose.position = fixed_point.point;
      pose_.pose.position.z = 0.0; // Planar stand-off distance measured from base_footprint.
      pose_.pose.orientation.w = 1.0;
      camera_bearing_ = std::atan2(optical.point.x, optical.point.z);
      pose_valid_ = target_base_.point.x > 0.0;
      if (visible_ && pose_valid_) {
        if (confirmation_count_ == 0) {confirmation_start_ = point_receipt_;}
        ++confirmation_count_;
      }
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Target TF unavailable: %s", e.what());
    }
  }
  void start_action(double wall)
  {
    if (goal_pending_ || goal_handle_ || wall < next_goal_wall_ || !action_client_->action_server_is_ready()) {return;}
    FollowObject::Goal goal;
    goal.pose_topic = "/follow/person_pose";
    goal.tracked_frame = "";
    goal.max_duration.sec = 0;
    goal.max_duration.nanosec = 0;
    const auto request = ++request_id_;
    goal_pending_ = true; cancel_requested_ = false;
    rclcpp_action::Client<FollowObject>::SendGoalOptions options;
    options.goal_response_callback = [this, request](GoalHandle::SharedPtr handle) {
        if (request != request_id_) {
          if (handle) {
            try {action_client_->async_cancel_goal(handle);}
            catch (const std::exception & e) {RCLCPP_WARN(get_logger(), "Late goal cancel failed: %s", e.what());}
          }
          return;
        }
        goal_pending_ = false; goal_handle_ = std::move(handle); raw_received_ = false;
        goal_started_wall_ = wall_now();
        if (!goal_handle_) {
          next_goal_wall_ = wall_now() + 1.0;
          RCLCPP_WARN(get_logger(), "Following action rejected; will retry");
        } else if (cancel_requested_ || !healthy_ || stopping_) {
          cancel_requested_ = false;
          cancel_action();
        }
      };
    options.result_callback = [this, request](const GoalHandle::WrappedResult &) {
        if (request != request_id_) {return;}
        goal_handle_.reset(); goal_pending_ = false; cancel_requested_ = false;
        raw_received_ = false; next_goal_wall_ = wall_now() + 0.5;
      };
    goal_sent_wall_ = wall;
    try {action_client_->async_send_goal(goal, options);}
    catch (const std::exception & e) {
      goal_pending_ = false; next_goal_wall_ = wall + 1.0;
      RCLCPP_WARN(get_logger(), "Cannot send following action: %s", e.what());
    }
  }
  void cancel_action()
  {
    if (cancel_requested_) {return;}
    if (goal_pending_ || goal_handle_) {cancel_requested_ = true;}
    if (goal_handle_) {
      try {action_client_->async_cancel_goal(goal_handle_);}
      catch (const std::exception & e) {RCLCPP_WARN(get_logger(), "Cancel failed: %s", e.what());}
    }
  }
  void tick()
  {
    if (stopping_) {command_pub_->publish(geometry_msgs::msg::Twist()); return;}
    const auto wall = wall_now();
    const auto ros = now().nanoseconds();
    const auto dt = wall - last_tick_; last_tick_ = wall;
    if (last_ros_ > 0 && ros < last_ros_) {
      point_stamp_ = 0; pose_valid_ = false; odom_received_ = false;
      raw_received_ = false; visible_ = false; clear_confirmation(); cancel_action(); gate_->reset();
    }
    if (ros != last_ros_) {clock_progress_wall_ = wall;}
    last_ros_ = ros;
    if (goal_pending_ && wall - goal_sent_wall_ > 3.0) {
      ++request_id_; goal_pending_ = false; cancel_requested_ = false; next_goal_wall_ = wall + 1.0;
    }
    if (goal_handle_ && !action_client_->action_server_is_ready()) {
      ++request_id_; goal_handle_.reset(); cancel_requested_ = false; raw_received_ = false;
      next_goal_wall_ = wall + 1.0;
    }
    if (goal_handle_ && wall - goal_started_wall_ > 2.0 &&
      (!raw_received_ || wall - raw_receipt_ > 2.0)) {cancel_action();}
    auto age = [ros](int64_t stamp) {return static_cast<double>(ros - stamp) * 1e-9;};
    pfc::Input input;
    input.emergency_stop = get_parameter("emergency_stop").as_bool();
    input.clock_valid = ros > 0;
    input.clock_advancing = wall - clock_progress_wall_ <= config_.receipt_timeout;
    input.visible = visible_ && pfc::fresh_age(wall-visible_receipt_, config_.receipt_timeout);
    input.odom_fresh = odom_received_ && pfc::fresh_age(wall-odom_receipt_, config_.odom_timeout) &&
      pfc::fresh_age(age(odom_stamp_), config_.odom_timeout, config_.future_tolerance);
    input.target_fresh = point_stamp_ > 0 && pfc::fresh_age(wall-point_receipt_, config_.receipt_timeout) &&
      pfc::fresh_age(age(point_stamp_), config_.point_timeout, config_.future_tolerance);
    input.pose_valid = pose_valid_;
    input.x = target_base_.point.x; input.y = target_base_.point.y;
    input.camera_bearing = camera_bearing_;
    healthy_ = !input.emergency_stop && input.clock_valid && input.clock_advancing &&
      input.visible && input.odom_fresh && input.target_fresh && input.pose_valid;
    if (!healthy_) {
      clear_confirmation(); raw_received_ = false; cancel_action();
    } else {
      if (!confirmed_ && confirmation_count_ >= config_.recovery_frames &&
        wall - confirmation_start_ >= config_.recovery_duration)
      {
        confirmed_ = true; confirmed_wall_ = wall; raw_received_ = false;
      }
      if (confirmed_) {
        // Cached measurements retain their acquisition stamp; never refresh stale data.
        pose_pub_->publish(pose_); base_pub_->publish(target_base_);
        if (std::hypot(input.x, input.y) >= config_.desired_distance + config_.resume_margin &&
          std::abs(camera_bearing_) <= config_.turn_enter) {start_action(wall);}
      }
    }
    input.confirmed = confirmed_;
    input.action_active = static_cast<bool>(goal_handle_) && !cancel_requested_;
    input.raw_fresh = raw_received_ && raw_receipt_ >= confirmed_wall_ &&
      pfc::fresh_age(wall-raw_receipt_, config_.raw_timeout);
    input.raw = raw_;
    const auto decision = gate_->step(input, dt);
    if (decision.state == "odom_unavailable") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Odometry rejected: sample_valid=%s, stamp_age=%.3f s (allowed -%.3f..%.3f), receipt_age=%.3f s",
        odom_received_ ? "true" : "false", age(odom_stamp_), config_.future_tolerance,
        config_.odom_timeout, wall-odom_receipt_);
    }
    geometry_msgs::msg::Twist command;
    command.linear.x = decision.command.linear;
    command.angular.z = decision.command.angular;
    command_pub_->publish(command);
    std_msgs::msg::String status; status.data = decision.state; state_pub_->publish(status);
    if (last_state_ != decision.state) {
      RCLCPP_INFO(get_logger(), "%s | distance %.2f m, bearing %.1f deg", decision.state.c_str(),
        std::hypot(input.x, input.y), camera_bearing_*180/pfc::pi);
      last_state_ = decision.state;
    }
  }
  pfc::Config config_;
  std::unique_ptr<pfc::Gate> gate_;
  std::string base_frame_, fixed_frame_, camera_frame_, last_state_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr base_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visible_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_sub_;
  rclcpp_action::Client<FollowObject>::SharedPtr action_client_;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::PointStamped target_base_;
  pfc::Command raw_{};
  double camera_bearing_{0}, point_receipt_{0}, visible_receipt_{0}, odom_receipt_{0}, raw_receipt_{0};
  double last_tick_{0}, clock_progress_wall_{0}, confirmation_start_{0}, confirmed_wall_{0}, next_goal_wall_{0};
  double goal_started_wall_{0}, goal_sent_wall_{0};
  int64_t point_stamp_{0}, odom_stamp_{0}, last_ros_{0};
  int confirmation_count_{0}, request_id_{0};
  bool visible_{false}, pose_valid_{false}, odom_received_{false}, raw_received_{false};
  bool confirmed_{false}, healthy_{false}, goal_pending_{false}, cancel_requested_{false}, stopping_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, handle_signal); std::signal(SIGTERM, handle_signal);
  std::shared_ptr<FollowSupervisor> node;
  try {
    node = std::make_shared<FollowSupervisor>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !interrupted) {executor.spin_some(); std::this_thread::sleep_for(5ms);}
    for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
      node->stop(); executor.spin_some(); std::this_thread::sleep_for(20ms);
    }
    executor.remove_node(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("follow_supervisor"), "%s", e.what());
    if (node && rclcpp::ok()) {
      try {node->stop(); std::this_thread::sleep_for(50ms);} catch (...) {}
    }
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
