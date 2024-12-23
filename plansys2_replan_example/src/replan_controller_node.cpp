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


#include <memory>
#include <random>

#include "plansys2_pddl_parser/Utils.hpp"

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


using namespace std::chrono_literals;

class ReplanController : public rclcpp::Node
{
public:
  ReplanController()
  : rclcpp::Node("replan_controller"),
    rd_()
  {
    custom_cb_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }

  bool init()
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

    if (!current_plan_.has_value()) {
      std::cout << "Could not find plan to reach goal " <<
        parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return false;
    }

    RCLCPP_INFO_STREAM(get_logger(), "New plan: ");
    print_plan(current_plan_.value());

    if (!executor_client_->start_plan_execution(current_plan_.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }

    timer_ = create_wall_timer(
      200ms, std::bind(&ReplanController::step, this), custom_cb_);
    timer_replan_ = create_wall_timer(
      2s, std::bind(&ReplanController::do_replan_if_better, this), custom_cb_);

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

    std::string new_goal = "(and (piece_at obj1 wp" +
        std::to_string(to1) + ")(piece_at obj2 wp" + 
        std::to_string(to2) + ")(piece_at obj3 wp" +
        std::to_string(to3) + "))";

    RCLCPP_INFO_STREAM(get_logger(), "-----------------------------------------------------------");
    RCLCPP_INFO_STREAM(get_logger(), "New goal set to "<< new_goal);
    RCLCPP_INFO_STREAM(get_logger(), "-----------------------------------------------------------");

    problem_expert_->setGoal(plansys2::Goal(new_goal));
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
    const plansys2_msgs::msg::Plan & remaining_plan)
  {
    RCLCPP_INFO(
      get_logger(), "New plan actions: %zu \t remaining plan actions: %zu",
      new_plan.items.size(), remaining_plan.items.size());

    // If a the new plan has less actions, it is better.
    // If a the new plan has the same actions, it isn't worth to change.
    // If the new plan has more actions is because the current plan is probably to fail
    // soon because new conditions in the map impose more actions (probably)
    // So, the decision is using '<' of '!='
  
    if (new_plan.items.size() < remaining_plan.items.size()) {
      RCLCPP_INFO_STREAM(get_logger(), "New plan: ");
      print_plan(new_plan);
      RCLCPP_INFO_STREAM(get_logger(), "Remaining plan: ");
      print_plan(remaining_plan);

      return true;
    } else {
      return false;
    }
  }

  void do_replan_if_better()
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

    if (should_replan(new_plan.value(), remaining_plan.value())) {
      current_plan_ = new_plan;
      problem_expert_->addPredicate(plansys2::Predicate("(robot_available r2d2)"));
      if (!executor_client_->start_plan_execution(current_plan_.value())) {
        RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
      }
    }
  }

  void change_map_fsm(){
    RCLCPP_INFO(get_logger(), "===================================================================");
    RCLCPP_INFO(get_logger(), "Change in the map:");

    switch (state_) {
    case TWO_OPEN:
      problem_expert_->removePredicate(plansys2::Predicate("(connected wp2 wp5)"));
      problem_expert_->removePredicate(plansys2::Predicate("(connected wp5 wp2)"));
 
      RCLCPP_INFO(get_logger(), "\tRemoved: connection wp2 <-> wp5");

      state_ = OPEN_47;
      break;
    case OPEN_47:
      problem_expert_->addPredicate(plansys2::Predicate("(connected wp2 wp5)"));
      problem_expert_->addPredicate(plansys2::Predicate("(connected wp5 wp2)"));
      problem_expert_->removePredicate(plansys2::Predicate("(connected wp4 wp7)"));
      problem_expert_->removePredicate(plansys2::Predicate("(connected wp7 wp4)"));
 
      RCLCPP_INFO(get_logger(), "\tRestored: connection wp2 <-> wp5");
      RCLCPP_INFO(get_logger(), "\tRemoved: connection wp4 <-> wp7");

      state_ = OPEN_25;
      break;
    case OPEN_25:
      problem_expert_->addPredicate(plansys2::Predicate("(connected wp4 wp7)"));
      problem_expert_->addPredicate(plansys2::Predicate("(connected wp7 wp4)"));
 
      RCLCPP_INFO(get_logger(), "\tRestored: connection wp4 <-> wp7");
      RCLCPP_INFO(get_logger(), "\tAll connection open");
     
      state_ = TWO_OPEN;
      break;
    }
    RCLCPP_INFO(get_logger(), "===================================================================");

  }

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  std::optional<plansys2_msgs::msg::Plan> current_plan_;

  rclcpp::CallbackGroup::SharedPtr custom_cb_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_replan_;
  rclcpp::TimerBase::SharedPtr timer_fsm_;

  static const int TWO_OPEN = 0;
  static const int OPEN_47 = 1;
  static const int OPEN_25 = 2;
  int state_ {OPEN_25};

  std::random_device rd_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReplanController>();

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);
  exe.add_node(node->get_node_base_interface());

  if (node->init()) {
    exe.spin();
  }

  rclcpp::shutdown();

  return 0;
}
