cmake_minimum_required(VERSION 3.8)
project(model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vex_message REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(simulator STATIC src/simulator.cpp)
ament_target_dependencies(simulator rclcpp std_msgs vex_message)
add_executable(model src/model.cpp)
add_executable(down src/down.cpp)
add_executable(testModel src/testModel.cpp)
add_executable(hco src/hco.cpp)
add_executable(full_controller src/full_controller.cpp)
target_link_libraries(testModel simulator)
target_link_libraries(hco simulator)
target_link_libraries(full_controller simulator)
ament_target_dependencies(model rclcpp std_msgs vex_message)
ament_target_dependencies(down rclcpp std_msgs vex_message)
ament_target_dependencies(testModel rclcpp std_msgs vex_message)
ament_target_dependencies(hco rclcpp std_msgs vex_message)
ament_target_dependencies(full_controller rclcpp std_msgs vex_message)

install(TARGETS
	model
	DESTINATION lib/${PROJECT_NAME})
install(TARGETS
	down
	DESTINATION lib/${PROJECT_NAME})
install(TARGETS
	testModel
	DESTINATION lib/${PROJECT_NAME})
install(TARGETS
	hco
	DESTINATION lib/${PROJECT_NAME})
install(TARGETS
	full_controller
	DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
