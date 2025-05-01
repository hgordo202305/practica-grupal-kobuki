#include "library_lib/Search.hpp"

namespace library_lib
{

Search::Search(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf){


  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

   bumper_sub_ = create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
    "/events/button", 10, std::bind(&Search::timer_callback,node, _1));
}

void
Search::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg)
{
  last_button_ = std::move(msg);
}

void
Search::on_tick()
{

  timer_ = create_wall_timer(50ms, std::bind(&Search::timer_callback, this));
  wp_.header.frame_id = "map";
  wp_.pose.orientation.w = 1.0;

  wp_.pose.position.x = 3.67;
  wp_.pose.position.y = -0.24;
}

void
Search::timer_callback(){
  
}
    switch(last_button_->button)
      {
        case 0:
          if(last_button_->state == kobuki_ros_interfaces::msg::ButtonEvent::PRESSED && i < size)
          {
            i++;
          }
        case 1:
          if(last_button_->state == kobuki_ros_interfaces::msg::ButtonEvent::PRESSED && i > 0)
            {
              i--;
            }
        case 3:
          timer_ = NULL;
          result_ = BT::NodeStatus::SUCCESS;
      }
}

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<library_lib::Search>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<library_lib::Search>(
    "Search", builder);
}
