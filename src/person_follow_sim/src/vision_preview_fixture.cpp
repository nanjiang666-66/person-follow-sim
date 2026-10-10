#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

// Stationary test fixture only: these transforms and odometry are not a robot driver.
class VisionPreviewFixture : public rclcpp::Node
{
public:
  VisionPreviewFixture() : Node("vision_preview_fixture")
  {
    rcl_interfaces::msg::ParameterDescriptor startup;
    startup.read_only = true;
    startup.description = "Stationary test setting: change launch arguments and restart.";
    const auto root = declare_parameter<std::string>("camera_root_frame", "camera_link", startup);
    const auto camera_x = declare_parameter<double>("camera_x", 0.0, startup);
    const auto height = declare_parameter<double>("camera_height", 1.0, startup);
    const auto path = declare_parameter<std::string>("log_path", "", startup);
    if (root.empty() || root == base_frame_ || root == fixed_frame_ ||
      !std::isfinite(camera_x) || !std::isfinite(height) || height < 0.0)
    {
      throw std::runtime_error("Invalid preview camera root or test mounting parameters");
    }
    if (get_parameter("use_sim_time").as_bool()) {
      throw std::runtime_error("Real camera preview requires use_sim_time=false");
    }
    if (!path.empty()) {
      const auto parent = std::filesystem::path(path).parent_path();
      if (!parent.empty()) {std::filesystem::create_directories(parent);}
      log_.open(path, std::ios::trunc);
      if (!log_) {throw std::runtime_error("Cannot open preview CSV: " + path);}
      log_ << "wall_s,ros_s,point_received,frame_id,raw_x,raw_y,raw_z,point_stamp_age_s,"
        "point_receipt_age_s,visible_received,visible,visible_age_s,base_received,"
        "base_x,base_y,base_z,base_stamp_age_s,cmd_received,cmd_v,cmd_w,cmd_age_s,"
        "state,depth_status\n";
      log_ << std::fixed << std::setprecision(6);
      RCLCPP_INFO(get_logger(), "Preview CSV: %s", path.c_str());
    }

    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped fixed_to_base;
    fixed_to_base.header.stamp = now();
    fixed_to_base.header.frame_id = fixed_frame_;
    fixed_to_base.child_frame_id = base_frame_;
    fixed_to_base.transform.rotation.w = 1.0;
    auto base_to_camera = fixed_to_base;
    base_to_camera.header.frame_id = base_frame_;
    base_to_camera.child_frame_id = root;
    base_to_camera.transform.translation.x = camera_x;
    base_to_camera.transform.translation.z = height;
    // The RealSense driver supplies camera_link -> color optical frame calibration.
    static_tf_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{
      fixed_to_base, base_to_camera});

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/follow_preview/odom", 10);
    point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>("/person/target_point", 10,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
        point_ = *msg; point_received_ = true; point_wall_ = wall_now();
      });
    visible_sub_ = create_subscription<std_msgs::msg::Bool>("/person/visible", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        visible_ = msg->data; visible_received_ = true; visible_wall_ = wall_now();
      });
    base_sub_ = create_subscription<geometry_msgs::msg::PointStamped>("/follow/target_base", 10,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
        base_ = *msg; base_received_ = true;
      });
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>("/follow_preview/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        command_ = *msg; command_received_ = true; command_wall_ = wall_now();
      });
    state_sub_ = create_subscription<std_msgs::msg::String>("/follow/state", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {state_ = msg->data;});
    depth_sub_ = create_subscription<std_msgs::msg::String>("/person/depth_status", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {depth_status_ = msg->data;});

    start_ = wall_now();
    odom_timer_ = create_wall_timer(20ms, [this] {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = now();
      odom.header.frame_id = fixed_frame_;
      odom.child_frame_id = base_frame_;
      odom.pose.pose.orientation.w = 1.0;
      odom_pub_->publish(odom);
    });
    sample_timer_ = create_wall_timer(50ms, [this] {sample();});
    RCLCPP_INFO(get_logger(),
      "Stationary interface fixture: %s -> %s -> %s; camera test x %.2f m, height %.2f m",
      fixed_frame_.c_str(), base_frame_.c_str(), root.c_str(), camera_x, height);
    RCLCPP_INFO(get_logger(), "Observe /follow_preview/cmd_vel; fixture odometry stays stationary");
  }

private:
  static double wall_now()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  static std::string csv_field(const std::string & value)
  {
    std::string result = "\"";
    for (const char c : value) {
      if (c == '"') {result += '"';}
      result += c;
    }
    return result + '"';
  }
  double stamp_age(const builtin_interfaces::msg::Time & stamp)
  {
    return (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
  }
  void sample()
  {
    const auto wall = wall_now();
    const auto point_age = point_received_ ? stamp_age(point_.header.stamp) : -1.0;
    const auto base_age = base_received_ ? stamp_age(base_.header.stamp) : -1.0;
    if (log_) {
      log_ << wall-start_ << ',' << now().seconds() << ',' << point_received_ << ','
        << csv_field(point_.header.frame_id) << ',' << point_.point.x << ','
        << point_.point.y << ',' << point_.point.z << ',' << point_age << ','
        << (point_received_ ? wall-point_wall_ : -1.0) << ',' << visible_received_ << ','
        << visible_ << ',' << (visible_received_ ? wall-visible_wall_ : -1.0) << ','
        << base_received_ << ',' << base_.point.x << ',' << base_.point.y << ','
        << base_.point.z << ',' << base_age << ',' << command_received_ << ','
        << command_.linear.x << ',' << command_.angular.z << ','
        << (command_received_ ? wall-command_wall_ : -1.0) << ','
        << csv_field(state_) << ',' << csv_field(depth_status_) << '\n';
      log_.flush();
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "preview | point %s age %.3f s | visible %s | base x %.2f y %.2f | cmd %.2f %.2f | %s",
      point_received_ ? point_.header.frame_id.c_str() : "not_received", point_age,
      visible_received_ ? (visible_ ? "true" : "false") : "not_received",
      base_.point.x, base_.point.y, command_.linear.x, command_.angular.z, state_.c_str());
  }

  const std::string fixed_frame_{"follow_preview_odom"}, base_frame_{"follow_preview_base"};
  double start_{0}, point_wall_{0}, visible_wall_{0}, command_wall_{0};
  bool point_received_{false}, visible_received_{false}, visible_{false};
  bool base_received_{false}, command_received_{false};
  std::string state_{"not_received"}, depth_status_{"not_received"};
  std::ofstream log_;
  geometry_msgs::msg::PointStamped point_, base_;
  geometry_msgs::msg::Twist command_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_, base_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visible_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_, depth_sub_;
  rclcpp::TimerBase::SharedPtr odom_timer_, sample_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<VisionPreviewFixture>());}
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("vision_preview_fixture"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
