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

#ifndef PLANSYS2_REPLAN_EXAMPLE__REPLANCONTROLLER_HPP_
#define PLANSYS2_REPLAN_EXAMPLE__REPLANCONTROLLER_HPP_

#include <string>
#include <random>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "plansys2_replan_example/ReplanStrategy.hpp"
#include "plansys2_replan_example/utils.hpp"

#include "rclcpp/rclcpp.hpp"


namespace plansys2_replan_example
{

struct GoalInfo
{
  int id;
  std::string instance_id;
  std::string predicate;
  std::string goal;
  rclcpp::Time start_time;
  rclcpp::Time end_time;
  bool achieved;
  bool active;
};

class ReplanController : public rclcpp::Node
{
public:
  ReplanController(
    std::shared_ptr<ReplanStrategy> replan_strategy,
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual bool init();
  virtual void init_knowledge();

  virtual void generate_new_problem() = 0;
  void add_new_goal();
  virtual void check_replan();

  virtual void step();
  void remove_achieved_fluents();

protected:
  std::string get_last_robot_at();

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  std::optional<plansys2_msgs::msg::Plan> current_plan_;
  std::vector<GoalInfo> goal_vector_;
  std::string current_goal_;
  std::string last_robot_at_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_replan_;
  rclcpp::TimerBase::SharedPtr timer_fsm_;

  std::random_device rd_;

  std::shared_ptr<ReplanStrategy> replan_strategy_;
};

}  // namespace plansys2_replan_example

#endif  // PLANSYS2_REPLAN_EXAMPLE__REPLANCONTROLLER_HPP_
