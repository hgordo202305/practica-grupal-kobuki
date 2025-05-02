#ifndef LIBRARY_LIB__ISREGISTERED_HPP_
#define LIBRARY_LIB__ISREGISTERED_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace library_lib
{

using namespace std::chrono_literals;

class IsRegistered : public library_lib::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  explicit IsRegistered(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("waypoint")});
  }

private:

  geometry_msgs::msg::PoseStamped wp_;
};

}

#endif