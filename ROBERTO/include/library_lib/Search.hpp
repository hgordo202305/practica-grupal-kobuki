#ifndef LIBRARY_LIB__SEARCH_HPP_
#define LIBRARY_LIB__SEARCH_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "kobuki_ros_interfaces/msg/button_event.hpp"
namespace library_lib
{

class Search : public BT::ActionNodeBase
{
public:
  explicit Search(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("waypoint")
      });
  }

private:
  void print_interface();
  void button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg);

  geometry_msgs::msg::PoseStamped wp_;
  int idx_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::ButtonEvent>::SharedPtr button_sub_;
  std::map<std::string,std::map<std::string,double>> waypoints_;
  int size_;
  std::vector<std::string> arr_;
};

}

#endif
