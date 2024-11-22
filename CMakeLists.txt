cmake_minimum_required(VERSION 3.14)
project(dynamixel_interface)
set(CMAKE_CXX_STANDARD 23)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)
add_compile_options(-Wall -Wextra -pedantic -Werror)

set(DYNAMIXEL_INTERFACE_LIB_SRC
  src/dynamixel_interface.cpp
)

set(DYNAMIXEL_INTERFACE_LIB_HEADERS
  include/dynamixel_interface/dynamixel_interface.hpp
  include/dynamixel_interface/visibility_control.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${DYNAMIXEL_INTERFACE_LIB_SRC}
  ${DYNAMIXEL_INTERFACE_LIB_HEADERS}
)

set(DYNAMIXEL_INTERFACE_NODE_SRC
  src/dynamixel_interface_node.cpp
)

set(DYNAMIXEL_INTERFACE_NODE_HEADERS
  include/dynamixel_interface/dynamixel_interface_node.hpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${DYNAMIXEL_INTERFACE_NODE_SRC}
  ${DYNAMIXEL_INTERFACE_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "dynamixel_interface::DynamixelInterfaceNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_dynamixel_interface
    test/test_dynamixel_interface.cpp
  )
  ament_target_dependencies(test_dynamixel_interface ${TEST_SOURCES})
  target_link_libraries(test_dynamixel_interface ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
