cmake_minimum_required(VERSION 3.8)
project(first_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bunker_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TfErrorMsg.msg"
  DEPENDENCIES std_msgs
)

# *ODOMETER SECTION , compile odometer.cpp so we can run it with name odometer
add_executable(odometer src/odometer.cpp)

ament_target_dependencies(odometer #required libraries from  executable
  rclcpp
  bunker_msgs
  nav_msgs
  tf2
  tf2_ros
  geometry_msgs
  std_srvs
)

# *TF_ERROR SECTION
add_executable(tf_error src/tf_error.cpp)

ament_target_dependencies(tf_error
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(tf_error "${cpp_typesupport_target}")
target_link_libraries(odometer "${cpp_typesupport_target}")


install(TARGETS #after build ,putting executable on correct place so we can use ros2 run
  odometer
  tf_error
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
