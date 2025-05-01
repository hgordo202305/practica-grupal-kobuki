// Copyright 2021 Intelligent Robotics Lab
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

#include "kobuki_ros_interfaces/msg/button_event.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "practica/ButtonNode.hpp"
#include <vector>

namespace practica
{

using namespace std::chrono_literals;
using std::placeholders::_1;

ButtonNode::ButtonNode()
: Node("button_node")
{
  button_sub_ = create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
    "/events/button", 10, std::bind(&ButtonNode::button_callback, this, _1));

  waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/waypoint", 10);
  timer_ = create_wall_timer(50ms, std::bind(&ButtonNode::control_cycle, this));

  last_button_ = std::make_unique<kobuki_ros_interfaces::msg::ButtonEvent>();


  wp.header.frame_id = "lab";
  wp.pose.position.x = 1.0;
  wp.pose.position.y = 2.0;
  wp.pose.orientation.w = 1.0;

  poses.push_back(wp);

  wp.header.frame_id = "mesa_profesor";
  wp.pose.position.x = 1.0;
  wp.pose.position.y = 2.0;
  wp.pose.orientation.w = 1.0;

  poses.push_back(wp);

  wp.header.frame_id = "baño_hombres";
  wp.pose.position.x = 1.0;
  wp.pose.position.y = 2.0;
  wp.pose.orientation.w = 1.0;

  poses.push_back(wp);

  wp.header.frame_id = "clase_probabilidad";
  wp.pose.position.x = 1.0;
  wp.pose.position.y = 2.0;
  wp.pose.orientation.w = 1.0;

  poses.push_back(wp);

  
  wp.header.frame_id = "escaleras";
  wp.pose.position.x = 1.0;
  wp.pose.position.y = 2.0;
  wp.pose.orientation.w = 1.0;

  poses.push_back(wp);
}

void
ButtonNode::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg)
{
  last_button_ = std::move(msg);
}

void
ButtonNode::control_cycle()
{

for (size_t i = 0; i < poses.size(); ++i) {
  auto& wp = poses[i];
  RCLCPP_INFO(this->get_logger(), "Pose #%zu - x: %.2f, y: %.2f", 
              i, wp.pose.position.x, wp.pose.position.y);
}
  

  waypoint_pub_->publish(wp);
}


}