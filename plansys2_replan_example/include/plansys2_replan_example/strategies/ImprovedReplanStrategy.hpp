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

#ifndef PLANSYS2_REPLAN_EXAMPLE__IMPROVEDREPLANSTRATEGY_HPP_
#define PLANSYS2_REPLAN_EXAMPLE__IMPROVEDREPLANSTRATEGY_HPP_


#include "plansys2_replan_example/ReplanStrategy.hpp"
#include "plansys2_replan_example/utils.hpp"

#include "rclcpp/rclcpp.hpp"


namespace plansys2_replan_example
{

class ImprovedReplanStrategy : public ReplanStrategy
{
public:
  ImprovedReplanStrategy()
  : ReplanStrategy()
  {}

  virtual bool should_replan(
    const plansys2_msgs::msg::Plan & new_plan,
    const plansys2_msgs::msg::Plan & remaining_plan,
    const std::string problem) override
  {
    (void) problem;

    RCLCPP_INFO(
      node_->get_logger(), "New plan actions: %zu \t remaining plan actions: %zu",
      new_plan.items.size(), remaining_plan.items.size());

    // If a the new plan has less actions, it is better.
    // If a the new plan has the same actions, it isn't worth to change.
    // If the new plan has more actions is because the current plan is probably to fail
    // soon because new conditions in the map impose more actions (probably)
    // So, the decision is using '<' of '!='
  
    if (new_plan.items.size() != remaining_plan.items.size()) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "New plan: ");
      print_plan(node_->get_logger(), new_plan);
      RCLCPP_INFO_STREAM(node_->get_logger(), "Remaining plan: ");
      print_plan(node_->get_logger(), remaining_plan);

      return true;
    } else {
      return false;
    }
  }
};

}  // namespace plansys2_replan_example

#endif  // PLANSYS2_REPLAN_EXAMPLE__IMPROVEDREPLANSTRATEGY_HPP_
