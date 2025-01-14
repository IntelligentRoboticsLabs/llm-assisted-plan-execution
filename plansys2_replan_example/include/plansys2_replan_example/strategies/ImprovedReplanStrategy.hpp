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

  virtual std::optional<plansys2_msgs::msg::Plan> get_better_replan(
    const plansys2_msgs::msg::PlanArray & new_plans,
    const plansys2_msgs::msg::Plan & remaining_plan,
    const std::string problem) override
  {
    (void) problem;

    if (new_plans.plan_array.empty()) {return {};}

    bool changed = false;
    plansys2_msgs::msg::Plan ret = remaining_plan;
    for (const auto & plan : new_plans.plan_array) {
      std::cerr << "Candidate --------------------------------->" << changed << " " <<
        plan.items.size() << " vs " << ret.items.size() << std::endl;
      print_plan(node_->get_logger(), plan);
      if ((!changed && plan.items.size() != ret.items.size()) ||
        (changed && plan.items.size() < ret.items.size()))
      {  // implement stability
        std::cerr << "Seleccionamos este " << std::endl;
        ret = plan;
        changed = true;
      }
    }

    if (changed) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "New plan: ");
      print_plan(node_->get_logger(), ret);
      RCLCPP_INFO_STREAM(node_->get_logger(), "Remaining plan: ");
      print_plan(node_->get_logger(), remaining_plan);
    }

    if (ret != remaining_plan) {
      return ret;
    } else {
      return {};
    }
  }
};

}  // namespace plansys2_replan_example

#endif  // PLANSYS2_REPLAN_EXAMPLE__IMPROVEDREPLANSTRATEGY_HPP_
