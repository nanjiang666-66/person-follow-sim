#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
class FollowMonitor : public rclcpp::Node
{
public:
  FollowMonitor() : Node("follow_monitor")
  {
    const auto path=declare_parameter<std::string>("log_path", "");
    if (!path.empty()) {
      const auto parent=std::filesystem::path(path).parent_path();
      if (!parent.empty()) {std::filesystem::create_directories(parent);}
      log_.open(path,std::ios::trunc);
      if (!log_) {throw std::runtime_error("Cannot open log_path: "+path);}
      log_<<"wall_s,ros_s,target_x,target_y,robot_x,robot_y,distance_m,bearing_deg,visible,visible_age_s,point_age_s,cmd_v,cmd_w,cmd_age_s,odom_v,odom_w,odom_age_s,state\n";
      log_<<std::fixed<<std::setprecision(6);
      RCLCPP_INFO(get_logger(),"CSV recording: %s",path.c_str());
    }
    start_=wall_now();
    truth_sub_=create_subscription<geometry_msgs::msg::PointStamped>("/sim/target_ground_truth",10,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr point) {truth_=*point; truth_received_=true;});
    point_sub_=create_subscription<geometry_msgs::msg::PointStamped>("/person/target_point",10,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr) {point_wall_=wall_now();});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr odom) {odom_=*odom; odom_wall_=wall_now(); odom_received_=true;});
    visible_sub_=create_subscription<std_msgs::msg::Bool>("/person/visible",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr flag) {visible_=flag->data; visible_wall_=wall_now();});
    cmd_sub_=create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr cmd) {command_=*cmd; cmd_wall_=wall_now();});
    state_sub_=create_subscription<std_msgs::msg::String>("/follow/state",10,
      [this](std_msgs::msg::String::ConstSharedPtr state) {state_=state->data;});
    timer_=create_wall_timer(50ms,[this] {sample();});
  }
private:
  static double wall_now() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  void sample()
  {
    if (!truth_received_ || !odom_received_) {return;}
    const auto wall=wall_now();
    const auto dx=truth_.point.x-odom_.pose.pose.position.x,dy=truth_.point.y-odom_.pose.pose.position.y;
    const auto distance=std::hypot(dx,dy);
    const auto & q=odom_.pose.pose.orientation;
    const auto yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    const auto bearing=std::atan2(std::sin(std::atan2(dy,dx)-yaw),std::cos(std::atan2(dy,dx)-yaw))*180/3.141592653589793;
    if (log_) {
      log_<<wall-start_<<','<<now().seconds()<<','<<truth_.point.x<<','<<truth_.point.y<<','
        <<odom_.pose.pose.position.x<<','<<odom_.pose.pose.position.y<<','<<distance<<','<<bearing<<','
        <<(visible_ ? 1 : 0)<<','<<wall-visible_wall_<<','<<wall-point_wall_<<','
        <<command_.linear.x<<','<<command_.angular.z<<','<<wall-cmd_wall_<<','
        <<odom_.twist.twist.linear.x<<','<<odom_.twist.twist.angular.z<<','<<wall-odom_wall_<<','<<state_<<'\n';
      log_.flush();
    }
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,
      "distance %.2f m | bearing %.1f deg | cmd %.2f m/s %.2f rad/s | %s",
      distance,bearing,command_.linear.x,command_.angular.z,state_.c_str());
  }
  double start_{0},point_wall_{0},visible_wall_{0},cmd_wall_{0},odom_wall_{0};
  bool visible_{false},truth_received_{false},odom_received_{false};
  std::string state_{"starting"};
  std::ofstream log_;
  geometry_msgs::msg::PointStamped truth_;
  geometry_msgs::msg::Twist command_;
  nav_msgs::msg::Odometry odom_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr truth_sub_,point_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visible_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<FollowMonitor>());}
  catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("follow_monitor"),"%s",e.what()); rclcpp::shutdown(); return 1;}
  rclcpp::shutdown(); return 0;
}
