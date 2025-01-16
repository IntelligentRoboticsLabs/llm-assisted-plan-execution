// Copyright 2025 Intelligent Robotics Lab
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

#ifndef PLANSYS2_REPLAN_EXAMPLE__TYPE_ADAPTER_HPP_
#define PLANSYS2_REPLAN_EXAMPLE__TYPE_ADAPTER_HPP_

#include "plansys2_examples_msgs/msg/goal_info.hpp"

#include "rclcpp/type_adapter.hpp"
#include "rclcpp/rclcpp.hpp"


namespace plansys2_replan_example
{

struct GoalInfo
{
  int id;
  std::string instance_id;
  std::string predicate;
  std::string goal;
  rclcpp::Time start_time;
  rclcpp::Time end_time;
  bool achieved;
  bool active;
};

}  // namespace plansys2_replan_example

template<>
struct rclcpp::TypeAdapter<plansys2_replan_example::GoalInfo, plansys2_examples_msgs::msg::GoalInfo>
{

  using is_specialized = std::true_type;
  using custom_type = plansys2_replan_example::GoalInfo;
  using ros_message_type = plansys2_examples_msgs::msg::GoalInfo;


  static
  void
  convert_to_ros_message(const custom_type & source,
    ros_message_type & destination)
  {
    destination.id = source.id;
    destination.instance_id = source.instance_id;
    destination.predicate = source.predicate;
    destination.goal = source.goal;
    destination.start_time.sec = source.start_time.seconds();
    destination.start_time.nanosec = source.start_time.nanoseconds();
    destination.end_time.sec = source.end_time.seconds();
    destination.end_time.nanosec = source.end_time.nanoseconds();
    destination.active = source.active;
    destination.achieved = source.achieved;
  }

  static
  void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination.id = source.id;
    destination.instance_id = source.instance_id;
    destination.predicate = source.predicate;
    destination.goal = source.goal;
    destination.start_time = rclcpp::Time(source.start_time.sec, source.start_time.nanosec);
    destination.end_time = rclcpp::Time(source.end_time.sec, source.end_time.nanosec);
    destination.active = source.active;
    destination.achieved = source.achieved;
  }

};


#endif  // PLANSYS2_REPLAN_EXAMPLE__TYPE_ADAPTER_HPP_