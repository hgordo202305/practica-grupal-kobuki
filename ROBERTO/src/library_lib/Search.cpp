#include <string>
#include <iostream>
#include <vector>

#include "library_lib/Search.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

namespace library_lib
{

Search::Search(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf)
{


    rclcpp::Node::SharedPtr node;
    config().blackboard->get("node", node);

    node->declare_parameter("waypoints",waypoints_);
    node->declare_parameter("arr",arr_);
    node->declare_parameter("size",size_);
    
    node->get_parameter("waypoints", waypoints_);
    node->get_parameter("arr", arr_);
    node->get_parameter("size", size_);
    button_sub_ = node->create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
     "/events/button", 10, std::bind(&Search::timer_callback,this, _1));
}

void
Search::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg)
{
  last_button_ = std::move(msg);
}

void
Search::on_tick()
{

  if(last_button_ == NULL)
  {
      return BT::NodeStatus::FAILURE;
  }
   switch(last_button_->button)
        {
          case 0:
            if(last_button_->state == kobuki_ros_interfaces::msg::ButtonEvent::PRESSED && i < size_)
            {
              idx_++;
            }
            break;
          case 1:
            if(last_button_->state == kobuki_ros_interfaces::msg::ButtonEvent::PRESSED && i > 0)
              {
                idx_--;
              }
            break;
          case 2:
            last_button_ = NULL;
            std::string result = arr[idx_];
            auto wp_param = waypoints.find(result);
            wp_.header.frame_id = "map";
            wp_.pose.orientation = wp_param->second.at("orientation_w");
            wp_.pose.position.x = wp_param->second.at("position_x");
            wp_.pose.position.y = wp_param->second.at("position_y");

            setOutput("waypoint", wp_);
            idx_ = 0;
            return BT::NodeStatus::SUCCESS;
      }
  return BT::NodeStatus::FAILURE;

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_nav::Search>("Search");
}
