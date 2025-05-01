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
#include <iostream>

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

  poses.push_back(create_pose("lab", 1.0, 2.0));
  poses.push_back(create_pose("mesa profesor", 3.0, 1.0));
  poses.push_back(create_pose("baño hombres", 1.0, 2.0));
  poses.push_back(create_pose("escalera", 3.0, 1.0));
  poses.push_back(create_pose("clase probabilidad", 1.0, 2.0));
  poses.push_back(create_pose("clase enrique", 3.0, 1.0));
}


geometry_msgs::msg::PoseStamped 
ButtonNode::create_pose(const std::string& frame_id, double x, double y) {

  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame_id;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1.0;
  return p;
}

void
ButtonNode::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg)
{
  last_button_ = std::move(msg);
}

void 
ButtonNode::printWaypoint(const geometry_msgs::msg::PoseStamped & wp) 
{
    std::cout << wp.header.frame_id << ":\n";
    std::cout << "  Position -> x: " << wp.pose.position.x
      << ", y: " << wp.pose.position.y << "\n";
}

void 
ButtonNode::printLista(const std::vector<geometry_msgs::msg::PoseStamped>& lista) {

  std::cout << "Botón 0 para navegar. 1 para bajar posicion en la lista, 2 para subir" << ":\n";

  for (size_t i = 0; i < lista.size(); ++i) {
    const auto& pose = lista[i];
    printWaypoint(pose);

  }
}


void
ButtonNode::control_cycle()
{

  if (show) {
    printLista(poses);

    show = false;
  }

  if (last_button_->button == 0 && last_button_->state == 1) {
     waypoint_pub_->publish(waypoint);
    
  }

  if (last_button_->button == 1 && last_button_->state == 1) {
    if (i < 4)
      i++;
    else
      i = 0;
    
    waypoint = poses[i];
    printWaypoint(waypoint);

  }
  if (last_button_->button == 2 && last_button_->state == 1) {
    if (i >= 0)
      i--;
    else
      i = 4;
    
    waypoint = poses[i];
    printWaypoint(waypoint);
  }

  }


}