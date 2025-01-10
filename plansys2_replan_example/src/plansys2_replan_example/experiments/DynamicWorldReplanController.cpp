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

#include "plansys2_replan_example/experiments/DynamicWorldReplanController.hpp"

#include "rclcpp/rclcpp.hpp"


namespace plansys2_replan_example
{

using namespace std::chrono_literals;

DynamicWorldReplanController::DynamicWorldReplanController(
  std::shared_ptr<ReplanStrategy> replan_strategy,
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: ReplanController(replan_strategy, node_name, options)
{
}

bool
DynamicWorldReplanController::init()
{
  if (!ReplanController::init()) {
    return false;
  }

  timer_fsm_ = create_wall_timer(
    25s, std::bind(&DynamicWorldReplanController::change_map_fsm, this));

  return true;
}


void
DynamicWorldReplanController::generate_new_problem()
{
  for (int i = 0; i < 3; i++) {
    add_new_goal();
  }
}

void
DynamicWorldReplanController::change_map_fsm()
{
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


}  // namespace plansys2_replan_example
