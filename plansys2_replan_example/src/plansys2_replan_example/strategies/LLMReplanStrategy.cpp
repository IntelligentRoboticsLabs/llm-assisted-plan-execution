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

  node_->declare_parameter("world_model", "");
  node_->declare_parameter("enable_forecaster", true);
  enable_forecaster_ = node_->get_parameter("enable_forecaster").as_bool();
  
  node_->declare_parameter("discontinued.self_reflector_context", "");
  node_->declare_parameter("discontinued.self_reflector_information_input", "");
  node_->declare_parameter("discontinued.replanner_expert_context", "");
  node_->declare_parameter("discontinued.replanner_expert_information_input", "");
  node_->declare_parameter("goal.self_reflector_context", "");
  node_->declare_parameter("goal.self_reflector_information_input", "");
  node_->declare_parameter("goal.replanner_expert_context", "");
  node_->declare_parameter("goal.replanner_expert_information_input", "");

  auto world_model = node_->get_parameter("world_model").as_string();
  if (world_model == "goal") {
    RCLCPP_INFO(node_->get_logger(), "Using goal world model");
    self_reflector_context_ = node_->get_parameter("goal.self_reflector_context").as_string();
    self_reflector_information_input_ = node_->get_parameter("goal.self_reflector_information_input").as_string();  
    replanner_expert_context_ = node_->get_parameter("goal.replanner_expert_context").as_string();
    replanner_expert_information_input_ = node_->get_parameter("goal.replanner_expert_information_input").as_string();
  } else if (world_model == "discontinued") {
    self_reflector_context_ = node_->get_parameter("discontinued.self_reflector_context").as_string();
    self_reflector_information_input_ = node_->get_parameter("discontinued.self_reflector_information_input").as_string();  
    replanner_expert_context_ = node_->get_parameter("discontinued.replanner_expert_context").as_string();
    replanner_expert_information_input_ = node_->get_parameter("discontinued.replanner_expert_information_input").as_string();

  }

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
  auto [problem, stamp] = problem_expert_->getProblemWithTimestamp();
  auto current_goal = problem_expert_->getGoal();

  replace_placeholder(self_reflector_context_, to_find_domain_, domain);
  replace_placeholder(replanner_expert_context_, to_find_domain_, domain);
  replace_placeholder(replanner_expert_context_, to_find_goal_, parser::pddl::toString(current_goal));
  replace_placeholder(replanner_expert_context_, to_find_problem_, problem);

  auto goal_msg = QueryLLM::Goal();

  goal_msg.query_text = self_reflector_context_;
  if (enable_forecaster_) {
    send_goal(goal_msg, send_goal_options_reflector_);
  } else {
    last_reflector_result_ = "{\"improvement_feedback\": \"No environmental forecast available. Rely strictly on the current PDDL state.\"}";
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  goal_msg.query_text = replanner_expert_context_;
  send_goal(goal_msg, send_goal_options_replanner_);  
}

void
LLMReplanStrategy::update_knowledge(const std::unordered_map<std::string, std::string> & knowledge)
{
  auto problem = knowledge.at("problem");
  auto remaining_plan = knowledge.at("remaining_plan");
  auto new_reflector_prompt = self_reflector_information_input_;
  auto goal_msg = QueryLLM::Goal();


  replace_placeholder(new_reflector_prompt, to_find_problem_, problem);
  replace_placeholder(new_reflector_prompt, to_find_plan_, remaining_plan);

  RCLCPP_INFO(node_->get_logger(), "********************************************");
  RCLCPP_INFO(node_->get_logger(), "New reflector prompt: %s", new_reflector_prompt.c_str());
  RCLCPP_INFO(node_->get_logger(), "********************************************");
  goal_msg.query_text = new_reflector_prompt;
  goal_msg.chat_id = goal_id_for_reflector_;
  
  if (enable_forecaster_) {
    send_goal(goal_msg, send_goal_options_reflector_);
  } else {
    last_reflector_result_ = "{\"improvement_feedback\": \"No environmental forecast available. Rely strictly on the current PDDL state.\"}";
  }
  
  RCLCPP_INFO(node_->get_logger(), "********************************************");
  RCLCPP_INFO(node_->get_logger(), "reflector answer: %s", last_reflector_result_.c_str());
  RCLCPP_INFO(node_->get_logger(), "********************************************");
  is_feedback_updated_ = true;
}

std::optional<plansys2_msgs::msg::Plan>
LLMReplanStrategy::get_better_replan(
    const plansys2_msgs::msg::PlanArray & new_plans,
    const plansys2_msgs::msg::Plan & remaining_plan,
    const std::string problem)
{
  // auto now = node_->now();
  if (new_plans.plan_array.empty()) {
    return {};
    }

  auto new_reflector_prompt = self_reflector_information_input_;
  auto new_replan_prompt = replanner_expert_information_input_;
  std::map<int, plansys2_msgs::msg::Plan> all_plans_map;
  auto goal_msg = QueryLLM::Goal();

  auto unique_plans = keeps_uniques(new_plans.plan_array);

  all_plans_map[0] = remaining_plan;
  int i = 1;
  for (const auto & plan : unique_plans) {
    all_plans_map[i] = plan;
    i++;
  }
  replace_placeholder(new_replan_prompt, to_find_plan_, get_plan_str(remaining_plan));
  std::string all_plans_str = "{";
  for (const auto & [key, value] : all_plans_map) {
    if (key == 0) {
      continue;
    }
    all_plans_str += "plan_";
    all_plans_str += std::to_string(key) + ":" + get_plan_str(value) + ",\n";
  }
  all_plans_str += std::string("}");
  
  std::string feedback_ = "";
  json reflector_response;

  try
  {
    reflector_response = json::parse(sanitize_json(last_reflector_result_));
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "[LLM_METRICS] Parse error in LLM response: %s", e.what());
    RCLCPP_ERROR(node_->get_logger(), e.what());
    return {};
  }

  replace_placeholder(new_replan_prompt, to_find_new_plan_, all_plans_str);
  if (is_feedback_updated_) {
    is_feedback_updated_ = false;
    feedback_ = reflector_response["improvement_feedback"];
  } else {
    feedback_ = "No updated feedback available, you will have to use the current state";
  }

  replace_placeholder(new_replan_prompt, to_find_feedback_, feedback_);
  replace_placeholder(new_replan_prompt, to_find_problem_, problem);

  goal_msg.query_text = new_replan_prompt;
  goal_msg.chat_id = goal_id_for_replanner_;
  RCLCPP_INFO(node_->get_logger(), "********************************************");
  RCLCPP_INFO(node_->get_logger(), "New replanner prompt: %s", new_replan_prompt.c_str());
  RCLCPP_INFO(node_->get_logger(), "********************************************");
  send_goal(goal_msg, send_goal_options_replanner_);

  RCLCPP_INFO(node_->get_logger(), "********************************************");
  RCLCPP_INFO(node_->get_logger(), "Last replanner result: %s", last_replanner_result_.c_str());
  RCLCPP_INFO(node_->get_logger(), "********************************************");
  json replan_response;
  try
  {
    replan_response = json::parse(sanitize_json(last_replanner_result_));
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "[LLM_METRICS] Parse error in LLM response: %s", e.what());
    RCLCPP_ERROR(node_->get_logger(), e.what());
    return {};
  }
  

  // RCLCPP_INFO(node_->get_logger(), "********************************************");
  // RCLCPP_INFO(node_->get_logger(), "Replan response: %s", replan_response.dump().c_str());
  // RCLCPP_INFO(node_->get_logger(), "********************************************");
  if (replan_response["selected_plan"] != 0 && all_plans_map[replan_response["selected_plan"].template get<int>()] != remaining_plan) {
    RCLCPP_INFO(node_->get_logger(), "********************************************");
    RCLCPP_INFO(node_->get_logger(), "New selected plan!: %d", replan_response["selected_plan"].template get<int>());
    RCLCPP_INFO(node_->get_logger(), "********************************************");
    // RCLCPP_INFO(node_->get_logger(), "********************************************");
    // RCLCPP_INFO(node_->get_logger(), "Current current problem %s", problem.c_str());
    // RCLCPP_INFO(node_->get_logger(), "********************************************");
    problem_expert_->clearKnowledge();
    problem_expert_->addProblem(problem);
    return all_plans_map[replan_response["selected_plan"].template get<int>()];
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
  
  if (enable_forecaster_) {
    send_goal(goal_msg, send_goal_options_reflector_);
  } else {
    last_reflector_result_ = "{\"improvement_feedback\": \"No environmental forecast available. Rely strictly on the current PDDL state.\"}";
  }

  json reflector_response;
  try {
    reflector_response = json::parse(last_reflector_result_);
  } catch(const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "[LLM_METRICS] Parse error in LLM response: %s", e.what());
    return false;
  }

  replace_placeholder(new_replan_prompt, to_find_plan_, get_plan_str(remaining_plan));
  replace_placeholder(new_replan_prompt, to_find_new_plan_, get_plan_str(new_plan));
  replace_placeholder(new_replan_prompt, to_find_feedback_, reflector_response["improvement_feedback"]);

  goal_msg.query_text = new_replan_prompt;
  goal_msg.chat_id = goal_id_for_replanner_;
  send_goal(goal_msg, send_goal_options_replanner_);

  json replan_response;
  try {
    replan_response = json::parse(last_replanner_result_);
  } catch(const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "[LLM_METRICS] Parse error in LLM response: %s", e.what());
    return false;
  }
  
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
         RCLCPP_INFO(node_->get_logger(), "Got result from reflector");
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
          RCLCPP_INFO(node_->get_logger(), "Got result from planner");
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
