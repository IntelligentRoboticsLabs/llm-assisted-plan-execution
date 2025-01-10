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

#ifndef PLANSYS2_REPLAN_EXAMPLE__LLMREPLANSTRATEGY_HPP_
#define PLANSYS2_REPLAN_EXAMPLE__LLMREPLANSTRATEGY_HPP_


#include <nlohmann/json.hpp>

#include "plansys2_replan_example/ReplanStrategy.hpp"
#include "plansys2_replan_example/utils.hpp"

#include "plansys2_examples_msgs/action/query_model.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2_replan_example
{

class LLMReplanStrategy : public ReplanStrategy
{
public:
  LLMReplanStrategy()
  : ReplanStrategy()
  {}

  void init() override;

  virtual bool should_replan(
    const plansys2_msgs::msg::Plan & new_plan,
    const plansys2_msgs::msg::Plan & remaining_plan,
    const std::string problem) override;

private:
  rclcpp::CallbackGroup::SharedPtr llm_cb;
  rclcpp_action::Client<QueryLLM>::SharedPtr llm_client_;
  rclcpp::executors::SingleThreadedExecutor llm_exe_;
  rclcpp_action::Client<QueryLLM>::SendGoalOptions send_goal_options_;

  std::string llm_context_;
  std::string prompt_template_;
};

}  // namespace plansys2_replan_example

#endif  // PLANSYS2_REPLAN_EXAMPLE__LLMREPLANSTRATEGY_HPP_
