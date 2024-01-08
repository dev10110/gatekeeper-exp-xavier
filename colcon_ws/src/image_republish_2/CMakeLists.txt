cmake_minimum_required(VERSION 3.8)
project(image_republish_2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
	rclcpp
	sensor_msgs
	)

# executable
add_executable(image_republisher src/image_republisher.cpp)
ament_target_dependencies(image_republisher ${dependencies})

# install
install(TARGETS
	image_republisher
	DESTINATION lib/${PROJECT_NAME}
	)

ament_package()
