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


#include <string>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"


#include "plansys2_replan_example/ReplanController.hpp"

#include "rclcpp/rclcpp.hpp"


namespace plansys2_replan_example
{

using namespace std::chrono_literals;

ReplanController::ReplanController(
    std::shared_ptr<ReplanStrategy> replan_strategy,
    const std::string & node_name,
    const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options),
  rd_(),
  replan_strategy_(replan_strategy)
{
  goal_info_pub_ = create_publisher<GoalInfo>(
    "/experiments_goal_info", 10);
}

bool
ReplanController::init()
{
  domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();
  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
  executor_client_ = std::make_shared<plansys2::ExecutorClient>();

  init_knowledge();
  generate_new_problem();

  auto domain = domain_expert_->getDomain();
  auto problem = problem_expert_->getProblem();
  current_plan_ = planner_client_->getPlan(domain, problem);
  
  replan_strategy_->add_domain_expert(domain_expert_);
  replan_strategy_->add_problem_expert(problem_expert_);
  replan_strategy_->add_planner_client(planner_client_);
  replan_strategy_->add_executor_client(executor_client_);
  replan_strategy_->set_node(shared_from_this());
  replan_strategy_->init();

  if (!current_plan_.has_value()) {
    std::cout << "Could not find plan to reach goal " <<
      parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
    return false;
  }

  RCLCPP_INFO_STREAM(get_logger(), "New plan: ");
  print_plan(get_logger(), current_plan_.value());

  if (!executor_client_->start_plan_execution(current_plan_.value())) {
    RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
  }

  timer_ = create_wall_timer(
    200ms, std::bind(&ReplanController::step, this));
  timer_replan_ = create_wall_timer(
    2s, std::bind(&ReplanController::check_replan, this));

  return true;
}

void
ReplanController::init_knowledge()
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
  
  problem_expert_->addPredicate(plansys2::Predicate("(connected wp4 wp7)"));
  problem_expert_->addPredicate(plansys2::Predicate("(connected wp7 wp4)"));
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
}

void
ReplanController::step()
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
          print_plan(get_logger(), current_plan_.value());

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
          print_plan(get_logger(), current_plan_.value());

          if (!executor_client_->start_plan_execution(current_plan_.value())) {
            RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
          }
        }
        break;
    }
  }

  // RCLCPP_INFO(get_logger(), "Goals: %zu", goal_vector_.size());
  for (auto & entry : goal_vector_) {
    if (entry.active) {
      entry.achieved = problem_expert_->existPredicate(plansys2::Predicate(entry.goal));

      // RCLCPP_INFO(
      //   get_logger(), "%s [%7.3lf] achieved %s\t active %s",
      //   entry.goal.c_str(), entry.start_time.seconds(),
      //   entry.achieved? "True": "False", entry.active? "True": "False");
    }
  }

   
  remove_achieved_fluents();

  RCLCPP_INFO(get_logger(), "=======================================");
  double achieved_time = 0.0;
  double achieved_count = 0;

  for (auto & entry : goal_vector_) {
    if (entry.achieved) {
      RCLCPP_INFO(
        get_logger(), "[%7.3lf] %s achieved in %7.3lf",
        entry.start_time.seconds(), entry.goal.c_str(),
        (entry.end_time - entry.start_time).seconds());
      achieved_time += (entry.end_time - entry.start_time).seconds();
      achieved_count++;
    }
  }
  if (achieved_count > 0) {
     RCLCPP_INFO(
        get_logger(), "Mean %7.3lf", achieved_time / achieved_count);
  }
  RCLCPP_INFO(get_logger(), "=======================================");
}

void
ReplanController::check_replan()
{
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

  if (replan_strategy_->should_replan(new_plan.value(), remaining_plan.value(), problem)) {
    current_plan_ = new_plan;
    problem_expert_->addPredicate(plansys2::Predicate("(robot_available r2d2)"));
    if (!executor_client_->start_plan_execution(current_plan_.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }
  }
}

void
ReplanController::add_new_goal()
{
  std::uniform_int_distribution<int> object_locations(1, 8);

  GoalInfo new_goal;
  new_goal.achieved = false;
  
  int from = object_locations(rd_);
  int to = object_locations(rd_);

  new_goal.active = true;
  new_goal.id = goal_vector_.size();
  new_goal.instance_id = "obj" + std::to_string(new_goal.id);
  new_goal.predicate = "(piece_at " +  new_goal.instance_id + " wp" + std::to_string(from) + ")";
  new_goal.goal = "(piece_at " +  new_goal.instance_id + " wp" + std::to_string(to) + ")";
  new_goal.start_time = now();

  goal_vector_.push_back(new_goal);
  goal_info_pub_->publish(new_goal);

  problem_expert_->addInstance(plansys2::Instance{new_goal.instance_id, "piece"});
  problem_expert_->addPredicate(plansys2::Predicate(new_goal.predicate));

  // update_goals;
  std::string current_goal_ = "(and ";
  for (const auto & entry : goal_vector_) {
    if (!entry.achieved && entry.active) {
      current_goal_ = current_goal_ + entry.goal;
    }
  }
  current_goal_ = current_goal_ + ")";

  // RCLCPP_INFO_STREAM(get_logger(), "-----------------------------------------------------------");
  // RCLCPP_INFO_STREAM(get_logger(), "New goal set to "<< current_goal_);
  // RCLCPP_INFO_STREAM(get_logger(), "-----------------------------------------------------------");

  problem_expert_->setGoal(plansys2::Goal(current_goal_));
}


void
ReplanController::remove_achieved_fluents()
{
  std::string goals = "(and ";
  for (const auto & entry : goal_vector_) {
    if (!entry.achieved) {
      goals = goals + entry.goal;
    }
  }
  goals = goals + ")";

  if (goals != current_goal_) {
    current_goal_ = goals;
    // RCLCPP_INFO_STREAM(get_logger(), "-----------------------------------------------------------");
    // RCLCPP_INFO_STREAM(get_logger(), "New goal set to "<< current_goal_);
    // RCLCPP_INFO_STREAM(get_logger(), "-----------------------------------------------------------");
    problem_expert_->setGoal(plansys2::Goal(current_goal_));
  }

  for (auto & entry : goal_vector_) {
    if (entry.achieved && entry.active) {
      // RCLCPP_INFO_STREAM(get_logger(), "-----------------------------------------------------------");
      // RCLCPP_INFO_STREAM(get_logger(), "Removed: "<< entry.goal << "\t" << entry.instance_id);
      // RCLCPP_INFO_STREAM(get_logger(), "-----------------------------------------------------------");

      entry.end_time = now();
      
      RCLCPP_INFO(
        get_logger(), "[Metric] %s [%7.3lf] achieved Goal in %7.3lf ",
        entry.goal.c_str(), entry.start_time.seconds(),
        (now() - entry.start_time).seconds());
  
      problem_expert_->removePredicate(plansys2::Predicate(entry.goal));
      problem_expert_->removeInstance(plansys2::Instance{entry.instance_id, "piece"});
      entry.active = false;
    }
  }
}

}  // namespace plansys2_replan_example
