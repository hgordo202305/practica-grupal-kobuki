#include "library_lib/Search.hpp"

namespace library_lib
{

Search::Search(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf) : BT::ActionNodeBase(xml_tag_name, conf){


  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

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
          case 1:
            if(last_button_->state == kobuki_ros_interfaces::msg::ButtonEvent::PRESSED && i > 0)
              {
                idx_--;
              }
          case 3:
            timer_ = NULL;
            //hacer waypoint aqui
            return BT::NodeStatus::SUCCESS;
      }
  return BT::NodeStatus::FAILURE;

}

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<library_lib::Search>(
        name, "search_wp", config);
    };

  factory.registerBuilder<library_lib::Search>(
    "Search", builder);
}
