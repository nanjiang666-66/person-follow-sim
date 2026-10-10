#include "person_follow_sim/scenarios.hpp"
#include <chrono>
#include <cmath>
#include <memory>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;
namespace sim = person_follow_sim;
class TargetSimulator : public rclcpp::Node
{
public:
  TargetSimulator() : Node("simulated_person")
  {
    scenario_ = startup_parameter<std::string>("scenario", "demo");
    sim::scenario(scenario_, 0); // Validate the scenario before starting.
    camera_x_ = startup_parameter("camera_x", 0.25);
    camera_height_ = startup_parameter("camera_height", 1.0);
    horizontal_fov_ = startup_parameter("horizontal_fov", 1.20);
    vertical_fov_ = startup_parameter("vertical_fov", 0.90);
    camera_frame_ = startup_parameter<std::string>("camera_frame", "camera_color_optical_frame");
    declare_parameter("force_lost", false);
    declare_parameter("force_invalid_depth", false);
    declare_parameter("pause_publishing", false);
    point_pub_=create_publisher<geometry_msgs::msg::PointStamped>("/person/target_point",10);
    truth_pub_=create_publisher<geometry_msgs::msg::PointStamped>("/sim/target_ground_truth",10);
    visible_pub_=create_publisher<std_msgs::msg::Bool>("/person/visible",10);
    detected_pub_=create_publisher<std_msgs::msg::Bool>("/person/detected",10);
    id_pub_=create_publisher<std_msgs::msg::Int32>("/person/target_id",10);
    depth_pub_=create_publisher<std_msgs::msg::String>("/person/depth_status",10);
    marker_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("/sim/markers",10);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr odom) {odom_=*odom; received_=true;});
    timer_=create_wall_timer(50ms,[this] {tick();});
  }
private:
  template<typename T>
  T startup_parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Startup setting: edit launch/YAML and restart.";
    return declare_parameter<T>(name, value, descriptor);
  }
  void tick()
  {
    const auto current=now().nanoseconds();
    if (current<=0 || !received_) {return;}
    if (current==last_time_) {return;} // Paused /clock produces no fictitious new frames.
    if (start_time_==0 || current<last_time_) {start_time_=current;}
    last_time_=current;
    const auto elapsed=static_cast<double>(current-start_time_)*1e-9;
    const auto target=sim::scenario(scenario_,elapsed);
    geometry_msgs::msg::PointStamped truth;
    truth.header.frame_id="odom"; truth.header.stamp=now();
    truth.point.x=target.x; truth.point.y=target.y; truth.point.z=target.z;
    truth_pub_->publish(truth);
    const auto & q=odom_.pose.pose.orientation;
    const auto yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    const auto optical=sim::to_optical(target,odom_.pose.pose.position.x,
      odom_.pose.pose.position.y,yaw,camera_x_,camera_height_);
    const auto odom_age=(now()-rclcpp::Time(odom_.header.stamp)).seconds();
    const bool odom_fresh=odom_age>=-0.05 && odom_age<=0.5;
    const bool detected=target.detected && odom_fresh && sim::in_view(optical,horizontal_fov_,vertical_fov_) &&
      !get_parameter("force_lost").as_bool();
    const bool visible=detected && target.depth && !get_parameter("force_invalid_depth").as_bool();
    publish_markers(truth,visible,yaw);
    if (get_parameter("pause_publishing").as_bool()) {return;}
    if (visible) {
      geometry_msgs::msg::PointStamped point;
      point.header.frame_id=camera_frame_;
      point.header.stamp=odom_.header.stamp; // Geometry uses this odometry sample.
      point.point.x=optical.x; point.point.y=optical.y; point.point.z=optical.z;
      point_pub_->publish(point);
    }
    std_msgs::msg::Bool flag; flag.data=detected; detected_pub_->publish(flag);
    flag.data=visible; visible_pub_->publish(flag);
    std_msgs::msg::Int32 id; id.data=1; id_pub_->publish(id);
    std_msgs::msg::String reason;
    reason.data=visible ? "reliable_depth" : (detected ? "no_reliable_depth" : "target_not_detected");
    depth_pub_->publish(reason);
  }
  void publish_markers(const geometry_msgs::msg::PointStamped & truth,bool visible,double yaw)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker marker;
    marker.header=truth.header; marker.ns="target"; marker.id=0;
    marker.type=visualization_msgs::msg::Marker::CYLINDER;
    marker.action=visualization_msgs::msg::Marker::ADD;
    marker.pose.position=truth.point; marker.pose.position.z=0.8; marker.pose.orientation.w=1;
    marker.scale.x=marker.scale.y=0.35; marker.scale.z=1.6;
    marker.color.a=0.9; marker.color.g=visible ? 0.9F : 0.3F; marker.color.r=visible ? 0.1F : 0.9F;
    array.markers.push_back(marker);
    marker.id=1; marker.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.pose.position.z=1.9; marker.scale.z=0.16;
    marker.color.r=marker.color.g=marker.color.b=1;
    marker.text=scenario_+(visible ? " | reliable" : " | invalid / hidden");
    array.markers.push_back(marker);
    marker.ns="camera_fov"; marker.id=0;
    marker.type=visualization_msgs::msg::Marker::LINE_STRIP;
    marker.pose.position.x=marker.pose.position.y=marker.pose.position.z=0;
    marker.scale.x=0.025; marker.color.r=0.2F; marker.color.g=0.6F; marker.color.b=1;
    geometry_msgs::msg::Point origin;
    origin.x=odom_.pose.pose.position.x+camera_x_*std::cos(yaw);
    origin.y=odom_.pose.pose.position.y+camera_x_*std::sin(yaw); origin.z=0.05;
    marker.points.push_back(origin);
    for (const double angle : {yaw-horizontal_fov_/2,yaw+horizontal_fov_/2}) {
      auto edge=origin; edge.x+=3*std::cos(angle); edge.y+=3*std::sin(angle); marker.points.push_back(edge);
    }
    marker.points.push_back(origin); array.markers.push_back(marker);
    marker_pub_->publish(array);
  }
  std::string scenario_,camera_frame_;
  double camera_x_{0},camera_height_{0},horizontal_fov_{0},vertical_fov_{0};
  int64_t start_time_{0},last_time_{0};
  bool received_{false};
  nav_msgs::msg::Odometry odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_,truth_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visible_pub_,detected_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr id_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr depth_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<TargetSimulator>());}
  catch(const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("simulated_person"),"%s",e.what()); rclcpp::shutdown(); return 1;}
  rclcpp::shutdown(); return 0;
}
