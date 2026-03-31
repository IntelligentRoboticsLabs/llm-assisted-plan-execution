cmake_minimum_required(VERSION 3.5)
project(plansys2_replan_example)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(plansys2_examples_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)
find_package(nlohmann_json REQUIRED)



set(dependencies
    rclcpp
    rclcpp_action
    geometry_msgs
    tf2_geometry_msgs
    plansys2_examples_msgs
    plansys2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
    ament_index_cpp
    plansys2_bt_actions
    nlohmann_json
    turtlesim_msgs
)

set(target_dependencies
    rclcpp::rclcpp
    ${turtlesim_msgs_TARGETS}
    rclcpp_action::rclcpp_action
    ${geometry_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    ${plansys2_examples_msgs_TARGETS}
    ${plansys2_msgs_TARGETS}
    plansys2_domain_expert::plansys2_domain_expert
    plansys2_executor::plansys2_executor
    plansys2_planner::plansys2_planner
    plansys2_problem_expert::plansys2_problem_expert
    plansys2_pddl_parser::plansys2_pddl_parser
    ament_index_cpp::ament_index_cpp
    plansys2_bt_actions::plansys2_bt_actions
    nlohmann_json::nlohmann_json
)

include_directories(include)

add_library(plansys2_close_gripper_bt_node SHARED src/behavior_tree_nodes/CloseGripper.cpp)
list(APPEND plugin_libs plansys2_close_gripper_bt_node)

add_library(plansys2_open_gripper_bt_node SHARED src/behavior_tree_nodes/OpenGripper.cpp)
list(APPEND plugin_libs plansys2_open_gripper_bt_node)

add_library(plansys2_move_bt_node SHARED src/behavior_tree_nodes/Move.cpp)
list(APPEND plugin_libs plansys2_move_bt_node)

add_library(plansys2_approach_object_bt_node SHARED src/behavior_tree_nodes/ApproachObject.cpp)
list(APPEND plugin_libs plansys2_approach_object_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_include_directories(${bt_plugin} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  target_link_libraries(${bt_plugin} PUBLIC ${target_dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/plansys2_replan_example/utils.cpp
  src/plansys2_replan_example/ReplanController.cpp
  src/plansys2_replan_example/experiments/DynamicWorldReplanController.cpp
  src/plansys2_replan_example/experiments/DynamicGoalsReplanController.cpp
  src/plansys2_replan_example/strategies/LLMReplanStrategy.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC ${target_dependencies})

add_executable(replan_controller src/replan_controller.cpp)
target_link_libraries(replan_controller PUBLIC ${target_dependencies} ${PROJECT_NAME})

add_executable(llm_replan_controller_node src/llm_replan_controller_node.cpp)
target_link_libraries(llm_replan_controller_node PUBLIC ${target_dependencies} ${PROJECT_NAME})

add_executable(nav2_sim_node src/nav2_sim_node.cpp)
target_link_libraries(nav2_sim_node PUBLIC ${target_dependencies})

install(DIRECTORY launch pddl behavior_trees behavior_trees_xml config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  replan_controller
  llm_replan_controller_node
  nav2_sim_node
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
