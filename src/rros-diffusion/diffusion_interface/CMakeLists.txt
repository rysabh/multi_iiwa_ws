cmake_minimum_required(VERSION 3.8)
project(diffusion_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# add the dependencies to create the custom interfaces
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mocap_optitrack_interfaces REQUIRED)
find_package(ati_sensor_interfaces REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DiffusionAction.srv"
  "msg/PredictedAction.msg"
  "msg/Observation.msg"
  DEPENDENCIES mocap_optitrack_interfaces geometry_msgs std_msgs ati_sensor_interfaces
  # DEPENDENCIES mocap_optitrack_interfaces geometry_msgs std_msgs

 )

ament_package()
