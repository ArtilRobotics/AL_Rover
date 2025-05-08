cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(CommDemo)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

find_package(SFML 2.5 COMPONENTS audio REQUIRED)

include_directories(${LIBSERIALPORT_INCLUDE_DIRS})
link_directories(${LIBSERIALPORT_LIBRARY_DIRS})

include_directories(/usr/include)
link_directories(/usr/lib)

add_executable(block_comm_test block_comm_demo.cpp motor.cpp)


add_executable(menu_motor_move menu_motor_move.cpp motor.cpp)

add_executable(control_mecanum control_mecanum.cpp motor.cpp)

add_executable(arms_control arms_control.cpp motor.cpp)

add_executable(read_motors read_motors.cpp motor.cpp)

add_executable(read_joystick read_joystick.cpp)

add_executable(identifica_u2d2 identifica_u2d2.cpp)

add_executable(control control.cpp motor.cpp) 

add_executable(listar_puertos  listar_puertos.cpp)

add_executable(audio_demo audio_demo.cpp)

target_link_libraries(audio_demo sfml-audio)

target_link_libraries(control sfml-audio)

target_link_libraries(listar_puertos ${LIBSERIALPORT_LIBRARIES})
 
target_link_libraries(control ${LIBSERIALPORT_LIBRARIES})

target_link_libraries(identifica_u2d2 ${LIBSERIALPORT_LIBRARIES})


target_link_libraries(control_mecanum ${LIBSERIALPORT_LIBRARIES})
target_link_libraries(control_mecanum spnav)

target_link_libraries(arms_control ${LIBSERIALPORT_LIBRARIES})

target_link_libraries(read_motors ${LIBSERIALPORT_LIBRARIES})


file(COPY ${CMAKE_SOURCE_DIR}/audio DESTINATION ${CMAKE_BINARY_DIR})
