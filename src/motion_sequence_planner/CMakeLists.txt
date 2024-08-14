cmake_minimum_required(VERSION 3.8)
project(motion_sequence_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)



#add packages required below
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



#added this following block to add the executable
include_directories(include)
add_executable(motion_seq_planner src/motion_seq_planner.cpp)



#modified block below
ament_target_dependencies(motion_seq_planner
  rclcpp
  moveit_msgs
  geometry_msgs
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




#modify to add install below
install(TARGETS
  motion_seq_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
