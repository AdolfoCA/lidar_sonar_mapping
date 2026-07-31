cmake_minimum_required(VERSION 3.5)
project(sonar_map_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/SeabedEstimatorState.msg"
  "msg/ClassifiedReturns.msg"
  "msg/DirichletVoxelMap.msg"
  "msg/PromotedVoxelKeys.msg"
  "msg/LearnedClassStates.msg"
  "msg/Eq24Telemetry.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
