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


#include <nlohmann/json.hpp>

#include "plansys2_replan_example/ReplanStrategy.hpp"
#include "plansys2_replan_example/utils.hpp"

#include "plansys2_examples_msgs/action/query_model.hpp"

#include "plansys2_replan_example/strategies/LLMReplanStrategy.hpp"


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2_replan_example
{

void
LLMReplanStrategy::init()
{
  ReplanStrategy::init();

  node_->declare_parameter("llm_context", "");
  node_->declare_parameter("prompt_template", "");

  custom_cb_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  llm_cb = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  llm_context_ = node_->get_parameter("llm_context").as_string();
  prompt_template_ = node_->get_parameter("prompt_template").as_string();


  llm_exe_.add_callback_group(llm_cb, this->get_node_base_interface());
    llm_client_ = rclcpp_action::create_client<QueryLLM>(
      node_->get_node_base_interface(),
      "/query_model",
      llm_cb);
}

bool
LLMReplanStrategy::should_replan(
  const plansys2_msgs::msg::Plan & new_plan,
  const plansys2_msgs::msg::Plan & remaining_plan)
{
  // To finish
}

}  // namespace plansys2_replan_example
