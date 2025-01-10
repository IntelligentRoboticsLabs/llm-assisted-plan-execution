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

#ifndef PLANSYS2_REPLAN_EXAMPLE__DYNAMICGOALSREPLANCONTROLLER_HPP_
#define PLANSYS2_REPLAN_EXAMPLE__DYNAMICGOALSREPLANCONTROLLER_HPP_

#include <string>

#include "plansys2_replan_example/ReplanController.hpp"

#include "rclcpp/rclcpp.hpp"


namespace plansys2_replan_example
{
class DynamicGoalsReplanController : public ReplanController
{
public:
  DynamicGoalsReplanController(
    std::shared_ptr<ReplanStrategy> replan_strategy,
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  bool init() override;
  virtual void generate_new_problem() override;

private:
  rclcpp::TimerBase::SharedPtr timer_goals_;
};

}  // namespace plansys2_replan_example

#endif  // PLANSYS2_REPLAN_EXAMPLE__DYNAMICGOALSREPLANCONTROLLER_HPP_
