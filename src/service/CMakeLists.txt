cmake_minimum_required(VERSION 3.8)
project(service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(client src/client.cpp)
ament_target_dependencies(client 
  rclcpp 
  std_msgs 
  sensor_msgs 
  cv_bridge 
  geometry_msgs
  OpenCV
)

add_executable(server src/server.cpp)
ament_target_dependencies(server 
  rclcpp 
  std_msgs 
  sensor_msgs 
  cv_bridge 
  geometry_msgs
  OpenCV
)

# # 服务端/客户端必须链接类型支持目标（让编译器找到生成的头文件）
# rosidl_get_typesupport_target(cpp_typesupport_target
#   interfaces rosidl_typesupport_cpp)
# target_link_libraries(server "${cpp_typesupport_target}")  # 服务端节点
# target_link_libraries(client "${cpp_typesupport_target}")  # 客户端节点


install(TARGETS server client
  DESTINATION lib/${PROJECT_NAME} 
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
