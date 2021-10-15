#############################################
# Copyright (c) Gaia Platform LLC
# All rights reserved.
#############################################

cmake_minimum_required(VERSION 3.5)

project(danger_zone)

include(/opt/gaia/cmake/gaia.cmake)

# --- Default to C++17 ---
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-c -Wall -Wextra -Wpedantic -ggdb)
endif()

# --- Debug build-specific options ---
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-O0 -g3 -ggdb -fno-limit-debug-info -fno-omit-frame-pointer -fno-optimize-sibling-calls -ggnu-pubnames -gsplit-dwarf)
  if(SANITIZER STREQUAL "ASAN")
    add_compile_options(-fsanitize=address -fsanitize=undefined -fno-sanitize-recover=all)
    add_link_options(-fsanitize=address -fsanitize=undefined -fno-sanitize-recover=all)
  endif()
else()
  add_compile_options(-O3)
endif()

# --- find dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
#find_package(rclcpp_server REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(danger_zone_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_snapshot_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(DANGER_ZONE_INCLUDES ${PROJECT_SOURCE_DIR}/inc ${CMAKE_CURRENT_SOURCE_DIR}/inc)
set(DANGER_ZONE_DB_NAME danger_zone)
set(DANGER_ZONE_DDL ${PROJECT_SOURCE_DIR}/src/danger_zone.ddl)
set(DANGER_ZONE_RULESET ${PROJECT_SOURCE_DIR}/src/danger_zone.ruleset)

# --- I dont think this is needed ---
configure_file("${PROJECT_SOURCE_DIR}/src/gaia_conf.toml" "${PROJECT_BINARY_DIR}/gaia_conf.toml")
configure_file("${PROJECT_SOURCE_DIR}/src/log_conf.toml" "${PROJECT_BINARY_DIR}/log_conf.toml")

# --- Generate EDC from DDL---
process_schema(
  DDL_FILE ${DANGER_ZONE_DDL}
  DATABASE_NAME ${DANGER_ZONE_DB_NAME}
)

# -- Translate ruleset into CPP --
translate_ruleset(
  RULESET_FILE ${DANGER_ZONE_RULESET}
  CLANG_PARAMS -I ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# --- add and link ---
add_executable(danger_zone
  src/danger_zone.cpp
  src/snapshot_client.cpp
  src/zones.cpp
)

target_add_gaia_generated_sources(danger_zone)
ament_target_dependencies(danger_zone rclcpp rclcpp_action std_msgs sensor_msgs vision_msgs danger_zone_msgs rosbag2_snapshot_msgs)
target_include_directories(danger_zone PRIVATE ${GAIA_INC} ${DANGER_ZONE_INCLUDES})
target_link_libraries(danger_zone ${GAIA_LIB})

install(TARGETS
  danger_zone
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
