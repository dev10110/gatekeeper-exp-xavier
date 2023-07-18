cmake_minimum_required(VERSION 3.8)
project(image_republish)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)



include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(node_plugins "")

## 
add_library(image_republish_component SHARED
	src/image_republisher.cpp
)
target_compile_definitions(image_republish_component
    PRIVATE "COMPOSITION_BUILDING_DLL"
)
# target_link_libraries(${PROJECT_NAME}_component
#     ${OpenCV_LIBS}
# )
ament_target_dependencies(image_republish_component
	rclcpp
	rclcpp_components
	sensor_msgs
	image_transport
)

rclcpp_components_register_node(image_republish_component
	PLUGIN "image_republish::ImageRepublisher"
	EXECUTABLE image_republisher_node
	)

set(node_plugins "${node_plugins}image_republish::ImageRepublisher;$<TARGET_FILE:image_republisher>\n")




# add_executable(image_republisher src/image_republisher.cpp)
# target_include_directories(image_republisher PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(image_republisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# target_link_libraries(image_republisher
# 	image_transport::image_transport
# 	rclcpp::rclcpp
# 	)
# 
# install(TARGETS image_republisher
#   DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()


install(TARGETS
	image_republish_component
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib
	)


# # install the launch files
# install(DIRECTORY
# 	launch
# 	DESTINATION share/${PROJECT_NAME}
# 	)

ament_package()
