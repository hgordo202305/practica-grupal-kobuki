#ifndef BUTTON_HPP_
#define BUTTON_HPP_

#include "kobuki_ros_interfaces/msg/button_event.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"


#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>

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

  rclcpp::Subscription<kobuki_ros_interfaces::msg::ButtonEvent>::SharedPtr button_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;


  kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr last_button_;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  geometry_msgs::msg::PoseStamped wp;


};

}  // namespace practica

#endif  // BUTTON_HPP_


