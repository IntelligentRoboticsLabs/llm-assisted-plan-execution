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

#include "plansys2_replan_example/experiments/DynamicGoalsReplanController.hpp"

#include "rclcpp/rclcpp.hpp"


namespace plansys2_replan_example
{

using namespace std::chrono_literals;

DynamicGoalsReplanController::DynamicGoalsReplanController(
  std::shared_ptr<ReplanStrategy> replan_strategy,
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: ReplanController(replan_strategy, node_name, options)
{
}

bool
DynamicGoalsReplanController::init()
{
  if (!ReplanController::init()) {
    return false;
  }

  timer_goals_ = create_wall_timer(
    35s, std::bind(&DynamicGoalsReplanController::add_new_goal, this));

  return true;
}


void
DynamicGoalsReplanController::generate_new_problem()
{
  add_new_goal();
}


}  // namespace plansys2_replan_example
