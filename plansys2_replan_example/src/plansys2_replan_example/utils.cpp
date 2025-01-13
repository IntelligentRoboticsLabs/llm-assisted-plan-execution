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
    
    // Find the position of the closing brace '}'
    size_t end = raw_input.find('}', start);
    if (end == std::string::npos) {
        throw std::invalid_argument("Invalid JSON: Missing closing brace.");
    }

    // Extract the substring between the braces
    return raw_input.substr(start, end - start + 1);
}


}  // namespace plansys2_replan_example
