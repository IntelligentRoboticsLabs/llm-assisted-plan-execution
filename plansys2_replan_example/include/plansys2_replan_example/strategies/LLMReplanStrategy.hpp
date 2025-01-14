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

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "plansys2_replan_example/ReplanStrategy.hpp"
#include "plansys2_replan_example/utils.hpp"

#include "plansys2_examples_msgs/action/query_model.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using QueryLLM = plansys2_examples_msgs::action::QueryModel;
using GoalHandleQueryLLM = rclcpp_action::ClientGoalHandle<QueryLLM>;
using json = nlohmann::json;

namespace plansys2_replan_example
{

class LLMReplanStrategy : public ReplanStrategy
{
public:
  LLMReplanStrategy(); 

  void init() override;

  bool should_replan(
    const plansys2_msgs::msg::Plan & new_plan,
    const plansys2_msgs::msg::Plan & remaining_plan,
    const std::string problem) override;

<<<<<<< HEAD
  void add_domain_expert(std::shared_ptr<plansys2::DomainExpertClient> domain_expert) override;
  void add_problem_expert(std::shared_ptr<plansys2::ProblemExpertClient> problem_expert) override; 
  void add_planner_client(std::shared_ptr<plansys2::PlannerClient> planner_client) override;
  void add_executor_client(std::shared_ptr<plansys2::ExecutorClient> executor_client) override;
=======
  void add_tool(std::shared_ptr<void> tool) override;
>>>>>>> refactoring

  void init_llm();

private:
  rclcpp::CallbackGroup::SharedPtr llm_cb;
  rclcpp_action::Client<QueryLLM>::SharedPtr llm_client_;
  rclcpp::executors::SingleThreadedExecutor llm_exe_;
  rclcpp_action::Client<QueryLLM>::SendGoalOptions send_goal_options_reflector_;
  rclcpp_action::Client<QueryLLM>::SendGoalOptions send_goal_options_replanner_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  const std::string to_find_problem_{"{problem}"};
  const std::string to_find_goal_{"{goal}"};
  const std::string to_find_plan_{"{current_plan}"};
  const std::string to_find_new_plan_{"{new_plan}"};
  const std::string to_find_feedback_{"{feedback}"};
  const std::string to_find_domain_{"{domain}"};
  std::string self_reflector_context_;
  std::string self_reflector_information_input_;
  std::string replanner_expert_context_;
  std::string replanner_expert_information_input_;
  std::string last_replanner_result_;
  std::string last_reflector_result_;
  uint8_t goal_id_for_replanner_{0};
  uint8_t goal_id_for_reflector_{0};
  
  void replace_placeholder(std::string &context, const std::string &placeholder, const std::string &value);
  void send_goal(const QueryLLM::Goal &goal_msg, rclcpp_action::Client<QueryLLM>::SendGoalOptions &send_goal_options);
  void goal_response_callback(const GoalHandleQueryLLM::SharedPtr &goal_handle);
  void configure_client_callbacks();
};

}  // namespace plansys2_replan_example

#endif  // PLANSYS2_REPLAN_EXAMPLE__LLMREPLANSTRATEGY_HPP_
