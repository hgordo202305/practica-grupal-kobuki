#ifndef BUTTON_HPP_
#define BUTTON_HPP_

#include "kobuki_ros_interfaces/msg/button_event.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <string>

namespace practica
{

using namespace std::chrono_literals;

class ButtonNode : public rclcpp::Node
{
public:
  ButtonNode();

private:
  void button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg);
  void control_cycle();
  void printWaypoint(const geometry_msgs::msg::PoseStamped & wp);
  void printLista(const std::vector<geometry_msgs::msg::PoseStamped>& lista);
  geometry_msgs::msg::PoseStamped create_pose(const std::string& frame_id, double x, double y);

  rclcpp::Subscription<kobuki_ros_interfaces::msg::ButtonEvent>::SharedPtr button_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;


  kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr last_button_;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  geometry_msgs::msg::PoseStamped waypoint;
  
  bool show = true;
  int i = 0;
};

}  // namespace practica

#endif  // BUTTON_HPP_



