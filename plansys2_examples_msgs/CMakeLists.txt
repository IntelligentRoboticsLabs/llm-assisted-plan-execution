cmake_minimum_required(VERSION 3.5)
project(plansys2_examples_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# TODO(jwallace42): This is a work around for https://github.com/ros2/rosidl_typesupport_fastrtps/issues/28
add_compile_options(-Wno-error=deprecated)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToPose.action"
  "action/QueryModel.action"
  "msg/GoalInfo.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
