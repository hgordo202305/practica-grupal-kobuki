// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "library_lib/NavOdom.hpp"

namespace library_lib
{

NavOdom::NavOdom(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: library_lib::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
}

void
NavOdom::on_tick()
{
  geometry_msgs::msg::PoseStamped odom;
  getInput("odom", odom);

  goal_.pose = odom;

}

BT::NodeStatus
NavOdom::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "** NAVIGATION SUCCEEDED **");

  return BT::NodeStatus::SUCCESS;
}


}

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<library_lib::NavOdom>(
        name, "navigate_to_odom", config);
    };

  factory.registerBuilder<library_lib::NavOdom>(
    "NavOdom", builder);
}
