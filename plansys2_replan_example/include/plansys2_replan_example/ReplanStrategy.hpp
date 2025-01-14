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

#include <optional>
#include <string>

#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/msg/plan_array.hpp"

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

  virtual std::optional<plansys2_msgs::msg::Plan> get_better_replan(
    const plansys2_msgs::msg::PlanArray & new_plans,
    const plansys2_msgs::msg::Plan & remaining_plan,
    const std::string problem) = 0;

  template<typename... Tools>
  void add_tools(Tools... tools)
  {
    add_tools_impl(tools...);
  }

  virtual void add_tools_impl() {}

  template<typename FirstTool, typename... RemainingTools>
  void add_tools_impl(FirstTool first_tool, RemainingTools... remaining_tools)
  {
    add_tool(first_tool);
    add_tools_impl(remaining_tools...);
  }
  virtual void add_tool(std::shared_ptr<void> tool) {}


protected:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace plansys2_replan_example

#endif  // PLANSYS2_REPLAN_EXAMPLE__REPLANSTRATEGY_HPP_
