cmake_minimum_required(VERSION 3.8)
project(purendt_3d_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
add_subdirectory(PureNDT-3D)

ament_auto_find_build_dependencies()

ament_auto_add_library(purendt_3d_ros2 SHARED
	src/purendt_3d_ros2.cpp
)
target_link_libraries(purendt_3d_ros2
	PureNDT3D
)

rclcpp_components_register_node(purendt_3d_ros2
	PLUGIN "PureNDT3D::PureNDT3DNode"
	EXECUTABLE purendt_3d_node
)

ament_auto_package()
