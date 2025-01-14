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


#include "plansys2_msgs/msg/plan.hpp"
#include "rclcpp/rclcpp.hpp"

#include "plansys2_replan_example/utils.hpp"


namespace plansys2_replan_example
{

void
print_plan(const rclcpp::Logger & logger, const plansys2_msgs::msg::Plan & plan)
{
  for (const auto & plan_item : plan.items) {
    RCLCPP_INFO_STREAM(logger, plan_item.time << ":\t" <<
      plan_item.action << "\t[" <<
      plan_item.duration << "]");
  }
}


int plan_difference(const plansys2_msgs::msg::Plan & baseline,
  const plansys2_msgs::msg::Plan & new_plan)
{
  int plan_diff = 0;

  for (const auto & item_base : baseline.items) {
    bool exist = false;
    for (const auto & item_new : new_plan.items) {
      if (item_base.action == item_new.action) {
        exist = true;
        break;
      }
    }

    if (!exist) {plan_diff++;}
  }

  for (const auto & item_new : new_plan.items) {
    bool exist = false;
    for (const auto & item_base : baseline.items) {
      if (item_base.action == item_new.action) {
        exist = true;
        break;
      }
    }

    if (!exist) {plan_diff++;}
  }

  return plan_diff;
}

float plan_continuity(const plansys2_msgs::msg::Plan & baseline,
  const plansys2_msgs::msg::Plan & new_plan)
{
  // Normalize items to mark as time 0 the currently executing actions
  plansys2_msgs::msg::Plan base_normalized = baseline;
  for (auto & item : base_normalized.items) {
    item.time = item.time - base_normalized.items.front().time;
  }

  int executing = 0;
  int continuation = 0;
  for (auto & base_item : base_normalized.items) {
    if (base_item.time > 0.1) {
      continue;
    }
    executing++;
    for (const auto & item_new : new_plan.items) {
      if (item_new.time < 0.1 && item_new.action == base_item.action) {
        continuation++;
      }
    }
  }

  return static_cast<float>(continuation) /  static_cast<float>(executing);

}
}  // namespace plansys2_replan_example
