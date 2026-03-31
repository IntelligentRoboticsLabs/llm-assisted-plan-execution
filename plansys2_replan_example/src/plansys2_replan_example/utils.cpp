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

std::string
get_plan_str(const plansys2_msgs::msg::Plan & plan)
{
    std::string ret = "";
    for (const auto & plan_item : plan.items) {
        ret += std::to_string(plan_item.time) + ":\t" +
              plan_item.action + "\t[" +
              std::to_string(plan_item.duration) + "]\n";
    }
    return ret;
}

std::string
sanitize_json(const std::string& raw_input) {
    size_t start = raw_input.find('{');
    if (start == std::string::npos) {
        throw std::invalid_argument("Invalid JSON: Missing opening brace.");
    }
    
    // Find the position of the last closing brace '}'
    size_t end = raw_input.find_last_of('}');
    if (end == std::string::npos || end < start) {
        throw std::invalid_argument("Invalid JSON: Missing closing brace.");
    }

    std::string sanitized = raw_input.substr(start, end - start + 1);
    
    // Replace literal newlines and control characters with spaces
    // to avoid nlohmann::json parse error (U+000A, etc.)
    for (char & c : sanitized) {
        if (static_cast<unsigned char>(c) < 32) {
            c = ' ';
        }
    }

    // Replace Markdown-style double single quotes with double quotes if LLM produced them
    sanitized = std::regex_replace(sanitized, std::regex("''"), "\"");
    
    return sanitized;
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

std::vector<plansys2_msgs::msg::Plan> keeps_uniques(const std::vector<plansys2_msgs::msg::Plan>& plans) {
  std::vector<plansys2_msgs::msg::Plan> unique_plans;

  for (const auto& plan : plans) {
      bool is_unique = true;

      // Check if the plan is already in the unique_plans vector
      for (const auto& uniquePlan : unique_plans) {
          if (plan == uniquePlan) {
              is_unique = false;
              break;
          }
      }

      // If it's unique, add it to the unique_plans vector
      if (is_unique) {
          unique_plans.push_back(plan);
      }
  }

  return unique_plans;
}

}  // namespace plansys2_replan_example
