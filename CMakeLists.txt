cmake_minimum_required(VERSION 3.8)
project(dual_arm_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)



add_executable(gravity_comp src/gravity_comp.cpp)


ament_target_dependencies(gravity_comp
  kdl_parser
  urdf
  orocos_kdl
)

target_link_libraries(gravity_comp 
  orocos-kdl
  )

include_directories(include)
install(TARGETS
  gravity_comp
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
