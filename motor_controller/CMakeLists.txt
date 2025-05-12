cmake_minimum_required(VERSION 3.8)
project(motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# libserialport
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

include_directories(
  include
  ${LIBSERIALPORT_INCLUDE_DIRS}
)

link_directories(${LIBSERIALPORT_LIBRARY_DIRS})

# Mensajes personalizados
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorState.msg"
  "msg/MotorCommand.msg"
  DEPENDENCIES builtin_interfaces
)

# Nodo principal con motor.cpp
add_executable(motor_node
  src/motor_node.cpp
  src/motor.cpp
)

ament_target_dependencies(motor_node
  rclcpp
  std_msgs
)

# Soporte de mensajes generados
rosidl_target_interfaces(motor_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Link con libserialport
target_link_libraries(motor_node
  ${LIBSERIALPORT_LIBRARIES}
)

# Instalación
install(TARGETS motor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
