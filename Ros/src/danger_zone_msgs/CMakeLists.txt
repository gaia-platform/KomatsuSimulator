cmake_minimum_required(VERSION 3.10)
project(danger_zone_msgs)

if(NOT WIN32)
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()
endif()

set(MSG_DEPS std_msgs vision_msgs)

find_package(ament_cmake 		REQUIRED)
find_package(rosidl_default_generators 	REQUIRED)
find_package(std_msgs 			REQUIRED)
find_package(vision_msgs    REQUIRED)

set(MSG_FILES
  msg/Obstacle.msg
  msg/ObstacleArray.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES ${MSG_DEPS}
)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION include/${PROJECT_NAME}
#)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()
