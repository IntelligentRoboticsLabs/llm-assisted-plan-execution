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

LLMReplanStrategy::LLMReplanStrategy()
: ReplanStrategy()
{

}

void 
LLMReplanStrategy::add_domain_expert(std::shared_ptr<plansys2::DomainExpertClient> domain_expert)
{
  domain_expert_ = domain_expert;
}
void
LLMReplanStrategy::add_problem_expert(std::shared_ptr<plansys2::ProblemExpertClient> problem_expert)
{
  problem_expert_ = problem_expert;
}
void
LLMReplanStrategy::add_planner_client(std::shared_ptr<plansys2::PlannerClient> planner_client)
{
  planner_client_ = planner_client;
}
void
LLMReplanStrategy::add_executor_client(std::shared_ptr<plansys2::ExecutorClient> executor_client)
{
  executor_client_ = executor_client;
}


void
LLMReplanStrategy::init()
{
  ReplanStrategy::init();

  node_->declare_parameter("self_reflector_context", "");
  node_->declare_parameter("self_reflector_information_input", "");
  node_->declare_parameter("replanner_expert_context", "");
  node_->declare_parameter("replanner_expert_information_input", "");

  self_reflector_context_ = node_->get_parameter("self_reflector_context").as_string();
  self_reflector_information_input_ = node_->get_parameter("self_reflector_information_input").as_string();  
  replanner_expert_context_ = node_->get_parameter("replanner_expert_context").as_string();
  replanner_expert_information_input_ = node_->get_parameter("replanner_expert_information_input").as_string();

  node_->declare_parameter("self_reflector_context", "");
  node_->declare_parameter("self_reflector_information_input", "");
  node_->declare_parameter("replanner_expert_context", "");
  node_->declare_parameter("replanner_expert_information_input", "");


  self_reflector_context_ = node_->get_parameter("self_reflector_context").as_string();
  self_reflector_information_input_ = node_->get_parameter("self_reflector_information_input").as_string();  
  replanner_expert_context_ = node_->get_parameter("replanner_expert_context").as_string();
  replanner_expert_information_input_ = node_->get_parameter("replanner_expert_information_input").as_string();

  llm_cb = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  llm_exe_.add_callback_group(llm_cb, node_->get_node_base_interface());
    llm_client_ = rclcpp_action::create_client<QueryLLM>(
      node_,
      "/query_model",
      llm_cb);

  configure_client_callbacks();
  init_llm();

}

void
LLMReplanStrategy::init_llm()
{
  auto domain = domain_expert_->getDomain();
  auto problem = problem_expert_->getProblem();
  auto current_goal = problem_expert_->getGoal();

  replace_placeholder(self_reflector_context_, to_find_domain_, domain);
  replace_placeholder(replanner_expert_context_, to_find_domain_, domain);
  replace_placeholder(replanner_expert_context_, to_find_goal_, parser::pddl::toString(current_goal));
  replace_placeholder(replanner_expert_context_, to_find_problem_, problem);

  auto goal_msg = QueryLLM::Goal();

  goal_msg.query_text = self_reflector_context_;
  send_goal(goal_msg, send_goal_options_reflector_);

  goal_msg.query_text = replanner_expert_context_;
  send_goal(goal_msg, send_goal_options_replanner_);  
}

std::optional<plansys2_msgs::msg::Plan>
LLMReplanStrategy::get_better_replan(
    const plansys2_msgs::msg::PlanArray & new_plans,
    const plansys2_msgs::msg::Plan & remaining_plan,
    const std::string problem)
{
  if (new_plans.plan_array.empty()) {
    return {};
    }

  auto new_reflector_prompt = self_reflector_information_input_;
  auto new_replan_prompt = replanner_expert_information_input_;
  std::map<int, plansys2_msgs::msg::Plan> all_plans_map;
  auto goal_msg = QueryLLM::Goal();

  all_plans_map[0] = remaining_plan;

  auto i = 1;
  for (const auto & plan : new_plans.plan_array) {
    all_plans_map[i] = plan;
    i++;
  }

  replace_placeholder(new_reflector_prompt, to_find_problem_, problem);
  replace_placeholder(new_reflector_prompt, to_find_plan_, get_plan_str(remaining_plan));

  goal_msg.query_text = new_reflector_prompt;
  goal_msg.chat_id = goal_id_for_reflector_;
  send_goal(goal_msg, send_goal_options_reflector_);

  json reflector_response = json::parse(sanitize_json(last_reflector_result_));

  replace_placeholder(new_replan_prompt, to_find_plan_, get_plan_str(remaining_plan));
  std::string all_plans_str = "{";
  for (const auto & [key, value] : all_plans_map) {
    if (key == 0) {
      continue;
    }
    all_plans_str += std::to_string(key) + ":" + get_plan_str(value) + ",\n";
  }
  all_plans_str += std::string("}");
  replace_placeholder(new_replan_prompt, to_find_new_plan_, all_plans_str);
  replace_placeholder(new_replan_prompt, to_find_feedback_, reflector_response["improvement_feedback"]);

  goal_msg.query_text = new_replan_prompt;
  goal_msg.chat_id = goal_id_for_replanner_;
  send_goal(goal_msg, send_goal_options_replanner_);

  json replan_response = json::parse(sanitize_json(last_replanner_result_));

  if (replan_response["selected_plan"] != 0) {
    RCLCPP_INFO(node_->get_logger(), "********************************************");
    RCLCPP_INFO(node_->get_logger(), "New selected plan!: %d", replan_response["selected_plan"]);
    RCLCPP_INFO(node_->get_logger(), "********************************************");
    return all_plans_map[replan_response["selected_alternate_plan"]];
  } else {
    return {};
  }
}

bool
LLMReplanStrategy::should_replan(
  const plansys2_msgs::msg::Plan & new_plan,
  const plansys2_msgs::msg::Plan & remaining_plan,
  const std::string problem)
{
  auto new_reflector_prompt = self_reflector_information_input_;
  auto new_replan_prompt = replanner_expert_information_input_;
  auto goal_msg = QueryLLM::Goal();

  replace_placeholder(new_reflector_prompt, to_find_problem_, problem);
  replace_placeholder(new_reflector_prompt, to_find_plan_, get_plan_str(remaining_plan));

  goal_msg.query_text = new_reflector_prompt;
  goal_msg.chat_id = goal_id_for_reflector_;
  send_goal(goal_msg, send_goal_options_reflector_);

  json reflector_response = json::parse(last_reflector_result_);

  replace_placeholder(new_replan_prompt, to_find_plan_, get_plan_str(remaining_plan));
  replace_placeholder(new_replan_prompt, to_find_new_plan_, get_plan_str(new_plan));
  replace_placeholder(new_replan_prompt, to_find_feedback_, reflector_response["improvement_feedback"]);

  goal_msg.query_text = new_replan_prompt;
  goal_msg.chat_id = goal_id_for_replanner_;
  send_goal(goal_msg, send_goal_options_replanner_);

  json replan_response = json::parse(last_replanner_result_);
  
  if (replan_response["should_use_alternate_plan"]) {
    return true;
  } else {
    return false;
  }  

}

void LLMReplanStrategy::replace_placeholder(
  std::string & context, const std::string & placeholder, const std::string & value)
{
  auto pos = context.find(placeholder);
  if (pos != std::string::npos) {
    context.replace(pos, placeholder.length(), value);
  }
}

void LLMReplanStrategy::send_goal(
  const QueryLLM::Goal & goal_msg,
  rclcpp_action::Client<QueryLLM>::SendGoalOptions & send_goal_options)
{
  auto future_server_response = llm_client_->async_send_goal(goal_msg, send_goal_options);
  llm_exe_.spin_until_future_complete(future_server_response);
  auto future_result = llm_client_->async_get_result(future_server_response.get());
  llm_exe_.spin_until_future_complete(future_result);
}
void LLMReplanStrategy::goal_response_callback(const GoalHandleQueryLLM::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void LLMReplanStrategy::configure_client_callbacks()
{
  send_goal_options_reflector_ = rclcpp_action::Client<QueryLLM>::SendGoalOptions();
  send_goal_options_replanner_ = rclcpp_action::Client<QueryLLM>::SendGoalOptions();

  send_goal_options_reflector_.goal_response_callback = std::bind(
    &LLMReplanStrategy::goal_response_callback, this, std::placeholders::_1);
  send_goal_options_replanner_.goal_response_callback = std::bind(
    &LLMReplanStrategy::goal_response_callback, this, std::placeholders::_1);

  send_goal_options_reflector_.result_callback = [this](const GoalHandleQueryLLM::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
          return;
      }
      goal_id_for_reflector_ = result.result->chat_id;
      last_reflector_result_ = result.result->result_text;
    };
  send_goal_options_replanner_.result_callback = [this](const GoalHandleQueryLLM::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
          return;
      }
      goal_id_for_replanner_ = result.result->chat_id;
      last_replanner_result_ = result.result->result_text;
    };
  RCLCPP_INFO(node_->get_logger(), "Clients callbacks configured!");
}

}  // namespace plansys2_replan_example
