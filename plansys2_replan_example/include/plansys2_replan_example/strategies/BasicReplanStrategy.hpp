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

#ifndef PLANSYS2_REPLAN_EXAMPLE__BASICREPLANSTRATEGY_HPP_
#define PLANSYS2_REPLAN_EXAMPLE__BASICREPLANSTRATEGY_HPP_


#include "plansys2_replan_example/ReplanStrategy.hpp"

#include "rclcpp/rclcpp.hpp"


namespace plansys2_replan_example
{

class BasicReplanStrategy : public ReplanStrategy
{
public:
  BasicReplanStrategy()
  : ReplanStrategy()
  {}

  virtual std::optional<plansys2_msgs::msg::Plan> get_better_replan(
    const plansys2_msgs::msg::PlanArray & new_plans,
    const plansys2_msgs::msg::Plan & remaining_plan,
    const std::string problem)
  {
    (void) new_plans;
    (void) remaining_plan;
    (void) problem;

    return std::nullopt;
    // return new_plans.plan_array.empty() ? std::nullopt : std::make_optional(new_plans.plan_array[0]);
  }
};

}  // namespace plansys2_replan_example

#endif  // PLANSYS2_REPLAN_EXAMPLE__BASICREPLANSTRATEGY_HPP_
