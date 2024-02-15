cmake_minimum_required(VERSION 3.8)
project(roscco_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BrakeCommand.msg"
  "msg/BrakeReport.msg"
  "msg/BrakeReportData.msg"
  "msg/CanFrame.msg"
  "msg/CanFrameData.msg"
  "msg/EnableDisable.msg"
  "msg/FaultReport.msg"
  "msg/FaultReportData.msg"
  "msg/SteeringCommand.msg"
  "msg/SteeringReport.msg"
  "msg/SteeringReportData.msg"
  "msg/ThrottleCommand.msg"
  "msg/ThrottleReport.msg"
  "msg/ThrottleReportData.msg"
  "msg/PIDReport.msg"
  "msg/PIDReport2.msg"
  "msg/PIDReport3.msg"
  "msg/CANInfo.msg"
DEPENDENCIES std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
