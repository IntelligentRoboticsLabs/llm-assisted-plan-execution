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

#include "plansys2_replan_example/experiments/DynamicWorldReplanController.hpp"
#include "plansys2_replan_example/experiments/DynamicGoalsReplanController.hpp"
#include "plansys2_replan_example/strategies/ImprovedReplanStrategy.hpp"
#include "plansys2_replan_example/strategies/BasicReplanStrategy.hpp"
#include "plansys2_replan_example/strategies/LLMReplanStrategy.hpp"

#include "rclcpp/rclcpp.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc != 5) {
    std::cerr << "Usage: " << argv[0] <<
      " --strategy <basic|improved|LLM> --experiment <dynamic_world|dynamic_goals>" << std::endl;
    return 1;
  }

  std::string strategy;
  std::string experiment;

  for (int i = 1; i < argc; i += 2) {
    std::string arg = argv[i];

    if (arg == "--strategy") {
        strategy = argv[i + 1];
    } else if (arg == "--experiment") {
        experiment = argv[i + 1];
    } else {
        std::cerr << "Error: Unknown argument '" << arg << "'." << std::endl;
        return 1;
    }
  }

  if ((strategy != "basic" && strategy != "improved" && strategy != "LLM") ||
      (experiment != "dynamic_world" && experiment != "dynamic_goals")) {
      std::cerr << "Error: Invalid strategy or experiment." << std::endl;
      std::cerr << "Valid strategies: basic, improved, LLM." << std::endl;
      std::cerr << "Valid experiments: dynamic_world, dynamic_goals." << std::endl;
      return 1;
  } 
  

  std::shared_ptr<plansys2_replan_example::ReplanStrategy> replan_strategy;
  std::shared_ptr<plansys2_replan_example::ReplanController> controller;

  if (strategy == "basic") {
    replan_strategy = std::make_shared<plansys2_replan_example::BasicReplanStrategy>();
  } else if (strategy == "improved") {
    replan_strategy = std::make_shared<plansys2_replan_example::ImprovedReplanStrategy>();
  } else if (strategy == "llm") {
    replan_strategy = std::make_shared<plansys2_replan_example::LLMReplanStrategy>();
  }

  if (experiment == "dynamic_world") {
    controller = std::make_shared<plansys2_replan_example::DynamicWorldReplanController>(
      replan_strategy, "dynamic_world");
  } else if (experiment == "dynamic_goals") {
    controller = std::make_shared<plansys2_replan_example::DynamicGoalsReplanController>(
      replan_strategy, "dynamic_goals");
  }

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);
  exe.add_node(controller->get_node_base_interface());

  if (controller->init()) {
    exe.spin();
  }

  rclcpp::shutdown();

  return 0;
}
