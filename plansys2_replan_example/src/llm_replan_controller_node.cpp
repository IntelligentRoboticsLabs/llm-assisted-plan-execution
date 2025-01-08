// Copyright 2024 Intelligent Robotics Lab
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


#include <iostream>
#include <memory>
#include <random>
#include <optional>
#include <regex>
#include <string>

#include <nlohmann/json.hpp>

#include "plansys2_pddl_parser/Utils.hpp"

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_examples_msgs/action/query_model.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


using namespace std::chrono_literals;

using QueryLLM = plansys2_examples_msgs::action::QueryModel;
using GoalHandleQueryLLM = rclcpp_action::ClientGoalHandle<QueryLLM>;
using json = nlohmann::json;


class ReplanController : public rclcpp::Node
{
public:
  ReplanController()
  : rclcpp::Node("llm_replan_controller"),
    rd_()
  {
    // declare parameters
    declare_parameter("llm_context", "");
    declare_parameter("prompt_template", "");

    custom_cb_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    llm_cb = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    llm_context_ = get_parameter("llm_context").as_string();
    prompt_template_ = get_parameter("prompt_template").as_string();
  }

  bool init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    llm_exe_.add_callback_group(llm_cb, this->get_node_base_interface());
    llm_client_ = rclcpp_action::create_client<QueryLLM>(
      this,
      "/query_model",
      llm_cb);

    init_knowledge();
    generate_new_problem();

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    current_plan_ = planner_client_->getPlan(domain, problem);

    if (!current_plan_.has_value()) {
      std::cout << "Could not find plan to reach goal " <<
        parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return false;
    }

    std::string to_find_domain = "{domain}";
    std::string to_find_goal = "{goal}";
    std::string to_find_problem = "{problem}";

    auto pos = llm_context_.find(to_find_domain);
    if (pos != std::string::npos) {
        llm_context_.replace(pos, to_find_domain.length(), domain);
    }
    pos = llm_context_.find(to_find_goal);
    if (pos != std::string::npos) {
        llm_context_.replace(pos, to_find_goal.length(), new_goal_);
    }
    pos = llm_context_.find(to_find_problem);
    if (pos != std::string::npos) {
        llm_context_.replace(pos, to_find_problem.length(), problem);
    }
    auto goal_msg = QueryLLM::Goal();
    goal_msg.query_text = llm_context_;

    send_goal_options_ = rclcpp_action::Client<QueryLLM>::SendGoalOptions();
    send_goal_options_.goal_response_callback = [this](const GoalHandleQueryLLM::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    send_goal_options_.result_callback = [this](const GoalHandleQueryLLM::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }
      goal_id_ = result.result->chat_id;
      last_result_ = result.result->result_text;
    };
    RCLCPP_INFO(get_logger(), "Sending first prompt, With context: %s", goal_msg.query_text.c_str());
    auto future_server_response = llm_client_->async_send_goal(goal_msg, send_goal_options_);
    llm_exe_.spin_until_future_complete(future_server_response);
    auto future_result = llm_client_->async_get_result(future_server_response.get());
    llm_exe_.spin_until_future_complete(future_result);


    RCLCPP_INFO_STREAM(get_logger(), "New plan: ");
    print_plan(current_plan_.value());

    if (!executor_client_->start_plan_execution(current_plan_.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }

    timer_ = create_wall_timer(
      200ms, std::bind(&ReplanController::step, this), custom_cb_);
    timer_replan_ = create_wall_timer( // the LLM limit is 15 request per minute
      4.5s, std::bind(&ReplanController::do_replan_if_better, this), custom_cb_);


    timer_fsm_ = create_wall_timer(
      25s, std::bind(&ReplanController::change_map_fsm, this), custom_cb_);

    return true;
  } 

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});

    problem_expert_->addInstance(plansys2::Instance{"wp1", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp2", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp3", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp4", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp5", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp6", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp7", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp8", "waypoint"});

    problem_expert_->addInstance(plansys2::Instance{"obj1", "piece"});
    problem_expert_->addInstance(plansys2::Instance{"obj2", "piece"});
    problem_expert_->addInstance(plansys2::Instance{"obj3", "piece"});

    problem_expert_->addPredicate(plansys2::Predicate("(connected wp1 wp2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp2 wp1)")); 
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp1 wp3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp3 wp1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp4 wp2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp2 wp4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp4 wp3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp3 wp4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp5 wp6)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp6 wp5)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp5 wp7)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp7 wp5)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp8 wp6)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp6 wp8)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp8 wp7)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp7 wp8)"));
    
    // For starting in OPEN_25 state
    // problem_expert_->addPredicate(plansys2::Predicate("(connected wp4 wp7)"));
    // problem_expert_->addPredicate(plansys2::Predicate("(connected wp7 wp4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp2 wp5)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp5 wp2)"));

    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp1 wp2) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp2 wp1) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp1 wp3) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp3 wp1) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp4 wp2) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp2 wp4) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp4 wp3) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp3 wp4) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp5 wp6) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp6 wp5) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp5 wp7) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp7 wp5) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp8 wp6) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp6 wp8) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp8 wp7) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp7 wp8) 1)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp4 wp7) 2)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp7 wp4) 2)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp2 wp5) 2)"));
    problem_expert_->addFunction(plansys2::Function("(= (nav_time wp5 wp2) 2)"));

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 wp1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_available r2d2)"));

    std::uniform_int_distribution<int> object_locations(1, 8);

    int from1 = object_locations(rd_);
    int from2 = object_locations(rd_);
    int from3 = object_locations(rd_);

    problem_expert_->addPredicate(plansys2::Predicate("(piece_at obj1 wp" + std::to_string(from1) + ")"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_at obj2 wp" + std::to_string(from2) + ")"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_at obj3 wp" + std::to_string(from3) + ")"));

  }

  void generate_new_problem()
  {
    std::uniform_int_distribution<int> object_locations(1, 8);

    int to1 = object_locations(rd_);
    int to2 = object_locations(rd_);
    int to3 = object_locations(rd_);

    new_goal_ = "(and (piece_at obj1 wp" +
        std::to_string(to1) + ")(piece_at obj2 wp" + 
        std::to_string(to2) + ")(piece_at obj3 wp" +
        std::to_string(to3) + "))";

    RCLCPP_INFO_STREAM(get_logger(), "-----------------------------------------------------------");
    RCLCPP_INFO_STREAM(get_logger(), "New goal set to "<< new_goal_);
    RCLCPP_INFO_STREAM(get_logger(), "-----------------------------------------------------------");

    problem_expert_->setGoal(plansys2::Goal(new_goal_));
  }

  void step()
  {
    if (!executor_client_->execute_and_check_plan()) {  // Plan finished
      switch (executor_client_->getResult().value().result) {
        case plansys2_msgs::action::ExecutePlan::Result::SUCCESS:
          {
            RCLCPP_INFO(get_logger(), "Plan succesfully finished");
            generate_new_problem();

            auto domain = domain_expert_->getDomain();
            auto problem = problem_expert_->getProblem();
            current_plan_ = planner_client_->getPlan(domain, problem);

            if (!current_plan_.has_value()) {
              std::cout << "Could not find plan to reach goal " <<
                parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
              return;
            }

            RCLCPP_INFO_STREAM(get_logger(), "New plan: ");
            print_plan(current_plan_.value());

            if (!executor_client_->start_plan_execution(current_plan_.value())) {
              RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
            }
          }
          break;
        case plansys2_msgs::action::ExecutePlan::Result::PREEMPT:
          RCLCPP_INFO(get_logger(), "Plan preempted");
          problem_expert_->addPredicate(plansys2::Predicate("(robot_available r2d2)"));
          break;
        case plansys2_msgs::action::ExecutePlan::Result::FAILURE:
          {
            RCLCPP_ERROR(get_logger(), "Plan finished with error");
            problem_expert_->addPredicate(plansys2::Predicate("(robot_available r2d2)"));

            auto domain = domain_expert_->getDomain();
            auto problem = problem_expert_->getProblem();
            current_plan_ = planner_client_->getPlan(domain, problem);

            if (!current_plan_.has_value()) {
              std::cout << "Could not find plan to reach goal " <<
                parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
              return;
            }

            RCLCPP_INFO_STREAM(get_logger(), "New plan: ");
            print_plan(current_plan_.value());

            if (!executor_client_->start_plan_execution(current_plan_.value())) {
              RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
            }
          }
          break;
      }
    }
  }

  void print_plan(const plansys2_msgs::msg::Plan & plan)
  {
    for (const auto & plan_item : plan.items) {
      RCLCPP_INFO_STREAM(get_logger(), plan_item.time << ":\t" <<
        plan_item.action << "\t[" <<
        plan_item.duration << "]");
    }
  }

  bool should_replan(
    const plansys2_msgs::msg::Plan & new_plan,
    const plansys2_msgs::msg::Plan & remaining_plan,
    const std::string problem,
    const std::string goal)
  {
    RCLCPP_INFO(
      get_logger(), "New plan actions: %zu \t remaining plan actions: %zu",
      new_plan.items.size(), remaining_plan.items.size());
    
    if (problem_history_.size() >= MAX_SIZE) {
      problem_history_.erase(problem_history_.begin());
    }
    
    auto problem_history_str = get_history_str(problem_history_);
    auto new_prompt = prompt_template_;  

    std::string to_find_problem = "{problem_history}";
    std::string to_find_goal = "{goal}";
    std::string to_find_plan = "{current_plan}";
    std::string to_find_new_plan = "{new_plan}";

    auto pos = new_prompt.find(to_find_problem);
    if (pos != std::string::npos) {
      if (problem_history_.size() == 0) {
      new_prompt.replace(pos, to_find_problem.length(), problem);
      } else {
        new_prompt.replace(pos, to_find_problem.length(), problem_history_str);
      }
    }
    pos = new_prompt.find(to_find_goal);
    if (pos != std::string::npos) {
      auto goal_msg = problem_expert_->getGoal();
      new_prompt.replace(pos, to_find_goal.length(), new_goal_);
    }
    pos = new_prompt.find(to_find_plan);
    if (pos != std::string::npos) {
        new_prompt.replace(pos, to_find_plan.length(), get_plan_str(remaining_plan));
    }
    pos = new_prompt.find(to_find_new_plan);
    if (pos != std::string::npos) {
        new_prompt.replace(pos, to_find_new_plan.length(), get_plan_str(new_plan));
    }

    RCLCPP_INFO(get_logger(), "*******************************************************************");
    RCLCPP_INFO(get_logger(), "INFORMATION PASSED TO THE LLM:");
    RCLCPP_INFO(get_logger(), "Problem history size: %s", std::to_string(problem_history_.size()).c_str());
    RCLCPP_INFO(get_logger(), "Problem history: %s", problem_history_str.c_str());
    RCLCPP_INFO(get_logger(), "Goal: %s", new_goal_.c_str());
    RCLCPP_INFO(get_logger(), "Current plan: %s", get_plan_str(remaining_plan).c_str());
    RCLCPP_INFO(get_logger(), "New plan: %s", get_plan_str(new_plan).c_str());
    RCLCPP_INFO(get_logger(), "*******************************************************************");

    RCLCPP_INFO(get_logger(), "*******************************************************************");
    RCLCPP_INFO(get_logger(), "CURRENT PLANSYS STATE:");
    // RCLCPP_INFO(get_logger(), "Domain: %s", domain_expert_->getDomain().c_str());
    RCLCPP_INFO(get_logger(), "Problem: %s", problem.c_str());
    RCLCPP_INFO(get_logger(), "*******************************************************************");
    auto goal_msg = QueryLLM::Goal();
    goal_msg.query_text = new_prompt;
    goal_msg.chat_id = goal_id_;

    // RCLCPP_INFO_ONCE(get_logger(), "Checking llm, With prompt: %s", goal_msg.query_text.c_str());

    auto future_server_response = llm_client_->async_send_goal(goal_msg, send_goal_options_);
    llm_exe_.spin_until_future_complete(future_server_response);
    auto future_result = llm_client_->async_get_result(future_server_response.get());
    llm_exe_.spin_until_future_complete(future_result);

    RCLCPP_INFO(get_logger(), "*******************************************************************");
    json response = json::parse(sanitize_json(last_result_));
    RCLCPP_INFO(get_logger(), "LLM response: %s", last_result_.c_str());
    RCLCPP_INFO(get_logger(), "*******************************************************************");



    if (response["should_replan"] == "Yes") {
      return true;
    } else {
      return false;
    }

  }

  void do_replan_if_better()
  {
    if (goal_id_ == 0) {
      RCLCPP_INFO(get_logger(), "No goal id, skipping replan");
      return;
    }
    // Getting a new plan with the current state
    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();

    // It is neccessary to add (robot_available r2d2) to the problem
    // to calculate a plan.
    const std::string to_find("( connected wp1 wp2 )");
    const std::string to_replace("( robot_available r2d2 )\n        ( connected wp1 wp2 )");

    size_t start_pos = problem.find(to_find); 
    if (start_pos != std::string::npos) {
      problem.replace(start_pos, to_find.length(), to_replace);
    }

    auto new_plan = planner_client_->getPlan(domain, problem);
    auto remaining_plan = executor_client_->get_remaining_plan();

    if (!new_plan.has_value() || !remaining_plan.has_value()) {
      RCLCPP_WARN_STREAM(
        get_logger(), "No plans : [" << new_plan.has_value() << ", " <<  remaining_plan.has_value());
      return;
    } 

    if (should_replan(new_plan.value(),
                      remaining_plan.value(),
                      problem,
                      new_goal_)) {
      current_plan_ = new_plan;
      problem_expert_->addPredicate(plansys2::Predicate("(robot_available r2d2)"));
      if (!executor_client_->start_plan_execution(current_plan_.value())) {
        RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
      }
    }
  }

  std::string get_plan_str(const plansys2_msgs::msg::Plan & plan)
  {
      std::string ret = "";
      for (const auto & plan_item : plan.items) {
          ret += std::to_string(plan_item.time) + ":\t" +
                plan_item.action + "\t[" +
                std::to_string(plan_item.duration) + "]\n";
      }
      return ret;
  }

  std::string sanitize_json(const std::string& raw_input) {
      size_t start = raw_input.find('{');
      if (start == std::string::npos) {
          throw std::invalid_argument("Invalid JSON: Missing opening brace.");
      }
      std::string sanitized = raw_input.substr(start);

      sanitized = std::regex_replace(sanitized, std::regex("''"), "\"");

      sanitized.erase(std::remove_if(sanitized.begin(), sanitized.end(), [](unsigned char c) {
          return c == '`' || std::isspace(c);
      }), sanitized.end());

      return sanitized;
  }

  std::string get_history_str(std::vector<std::string> problem_history_) {

    
    std::string ret = "[";
    for (const auto & problem : problem_history_) {
      // size_t start_pos = problem.find(":init");
      // if (start_pos != std::string::npos) {
      //     problem.erase(0, start_pos + 5);
      // }
      // start_pos = problem.find("nav_time");
      // if (start_pos != std::string::npos) {
      //     problem.erase(0, start_pos + 8);
      // }
      ret += problem + "," + "\n";
    }
    ret += "]";
    
    // start the string from :init :
    return ret;
  }

  void change_map_fsm() {
    RCLCPP_INFO(get_logger(), "===================================================================");
    RCLCPP_INFO(get_logger(), "Change in the map:");
    
    last_update_ = "";
    switch (state_) {
    case TWO_OPEN:
      {
        problem_expert_->removePredicate(plansys2::Predicate("(connected wp2 wp5)"));
        problem_expert_->removePredicate(plansys2::Predicate("(connected wp5 wp2)"));
        
        problem_expert_->addPredicate(plansys2::Predicate("(disconnected wp2 wp5)"));
        problem_expert_->addPredicate(plansys2::Predicate("(disconnected wp5 wp2)"));

        auto now = this->now().nanoseconds();
        last_update_ = "[";
        last_update_ += std::to_string(now) + "][removed][connection][wp2 <-> wp5] \n";
        last_update_ += "[" + std::to_string(now) + "][added][disconnection][wp2 <-> wp5] \n";
        
        RCLCPP_INFO(get_logger(), "\tRemoved: connection wp2 <-> wp5");

        state_ = OPEN_47;
        break;
      }
    case OPEN_47:
      {
        problem_expert_->addPredicate(plansys2::Predicate("(connected wp2 wp5)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected wp5 wp2)"));

        problem_expert_->removePredicate(plansys2::Predicate("(disconnected wp2 wp5)"));
        problem_expert_->removePredicate(plansys2::Predicate("(disconnected wp5 wp2)"));

        problem_expert_->removePredicate(plansys2::Predicate("(connected wp4 wp7)"));
        problem_expert_->removePredicate(plansys2::Predicate("(connected wp7 wp4)"));

        problem_expert_->addPredicate(plansys2::Predicate("(disconnected wp4 wp7)"));
        problem_expert_->addPredicate(plansys2::Predicate("(disconnected wp7 wp4)"));

        auto now = this->now().nanoseconds();
        last_update_ = "[";
        last_update_ += std::to_string(now) + "][added][connection][wp2 <-> wp5] \n";
        last_update_ += "[" + std::to_string(now) + "][removed][disconnection][wp2 <-> wp5] \n";
        last_update_ += "[" + std::to_string(now) + "][removed][connection][wp4 <-> wp7] \n";
        last_update_ += "[" + std::to_string(now) + "][added][disconnection][wp4 <-> wp7] \n";
  
        RCLCPP_INFO(get_logger(), "\tRestored: connection wp2 <-> wp5");
        RCLCPP_INFO(get_logger(), "\tRemoved: connection wp4 <-> wp7");

        state_ = OPEN_25;
        break;
      }
    case OPEN_25:
      {
        problem_expert_->addPredicate(plansys2::Predicate("(connected wp4 wp7)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected wp7 wp4)"));

        problem_expert_->removePredicate(plansys2::Predicate("(disconnected wp4 wp7)"));
        problem_expert_->removePredicate(plansys2::Predicate("(disconnected wp7 wp4)"));
  
        auto now = this->now().nanoseconds();
        last_update_ = "[";
        last_update_ += std::to_string(now) + "][added][connection][wp4 <-> wp7] \n";
        last_update_ += "[" + std::to_string(now) + "][removed][disconnection][wp4 <-> wp7] \n";

        RCLCPP_INFO(get_logger(), "\tRestored: connection wp4 <-> wp7");
        RCLCPP_INFO(get_logger(), "\tAll GRAPH IS CLOSED");
      
        state_ = TWO_OPEN;
        break;
      }
    }
    problem_history_.push_back(last_update_);
    RCLCPP_INFO(this->get_logger(), "Problem history size after modifing map: %zu", problem_history_.size());
    RCLCPP_INFO(get_logger(), "===================================================================");

  }

  rclcpp::CallbackGroup::SharedPtr custom_cb_;

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;


  std::optional<plansys2_msgs::msg::Plan> current_plan_;

  rclcpp::CallbackGroup::SharedPtr llm_cb;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_replan_;
  rclcpp::TimerBase::SharedPtr timer_fsm_;
  rclcpp_action::Client<QueryLLM>::SharedPtr llm_client_;
  rclcpp::executors::SingleThreadedExecutor llm_exe_;
  rclcpp_action::Client<QueryLLM>::SendGoalOptions send_goal_options_;

  std::string llm_context_;
  std::string prompt_template_;
  static const int TWO_OPEN = 0;
  static const int OPEN_47 = 1;
  static const int OPEN_25 = 2;
  static const size_t MAX_SIZE = 7;
  int state_ {OPEN_25};
  uint8_t goal_id_{0};
  std::string last_result_;
  std::string new_goal_;
  std::string last_update_;
  std::vector<std::string> problem_history_;

  std::random_device rd_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReplanController>();

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);
  exe.add_callback_group(node->custom_cb_, node->get_node_base_interface());

  if (node->init()) {
    exe.spin();
  }

  rclcpp::shutdown();

  return 0;
}
