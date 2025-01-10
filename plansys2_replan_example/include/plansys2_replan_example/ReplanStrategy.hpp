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

#ifndef PLANSYS2_REPLAN_EXAMPLE__REPLANSTRATEGY_HPP_
#define PLANSYS2_REPLAN_EXAMPLE__REPLANSTRATEGY_HPP_

#include <string>

#include "plansys2_msgs/msg/plan.hpp"

#include "rclcpp/rclcpp.hpp"

namespace plansys2_replan_example
{

class ReplanStrategy
{
public:
  ReplanStrategy() {}

  void set_node(rclcpp::Node::SharedPtr node) {
    node_ = node;
  }

  virtual void init() {}

  virtual bool should_replan(
    const plansys2_msgs::msg::Plan & new_plan,
    const plansys2_msgs::msg::Plan & remaining_plan,
    const std::string problem) = 0;

protected:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace plansys2_replan_example

#endif  // PLANSYS2_REPLAN_EXAMPLE__REPLANSTRATEGY_HPP_
