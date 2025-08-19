cmake_minimum_required(VERSION 3.8)
project(global_to_polar_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(planning_custom_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ===================================================================
# 1. 사용자 정의 메시지 파일
# ===================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolarGrid.msg"
  DEPENDENCIES std_msgs nav_msgs sensor_msgs
)

# 메시지 생성과 타겟 의존성을 위한 설정
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ===================================================================
# 2. 실행 파일 추가
# ===================================================================
# global_to_polar_node 실행 파일
add_executable(global_to_polar_node src/globToPolar.cpp)
ament_target_dependencies(global_to_polar_node
  rclcpp
  std_msgs
  nav_msgs
  tf2
)

# data_logger_node 실행 파일
add_executable(data_logger_node src/dataLoggerNode.cpp)
ament_target_dependencies(data_logger_node
  rclcpp
  std_msgs
  sensor_msgs
  planning_custom_msgs
  ament_index_cpp
)

# 사용자 정의 메시지 타입 링크
target_link_libraries(global_to_polar_node "${cpp_typesupport_target}")
target_link_libraries(data_logger_node "${cpp_typesupport_target}")

# ===================================================================
# 3. 설치 규칙 추가
# ===================================================================
install(TARGETS
  global_to_polar_node
  data_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install data directories if they exist
install(DIRECTORY
  line/
  DESTINATION share/${PROJECT_NAME}/line/
  OPTIONAL
)

install(DIRECTORY
  dataSet/
  DESTINATION share/${PROJECT_NAME}/dataSet/
  OPTIONAL
)

install(DIRECTORY
  auto_save/
  DESTINATION share/${PROJECT_NAME}/auto_save/
  OPTIONAL
)


ament_export_dependencies(rosidl_default_runtime)
ament_package()
