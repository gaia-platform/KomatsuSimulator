#*****************************************************************************
#*
#* Dependencies: Requires Gaia preview (without libc++) or later, Gaia March 
#* 2021 will not work
#*
#*****************************************************************************

cmake_minimum_required(VERSION 3.5)

# {SDK-SETUP:CMAKE} - Do not remove this line

project(danger_zone)

# {SDK-SETUP:CXX_STANDARD} - Do not remove this line

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(danger_zone_msgs REQUIRED)

set(DANGER_ZONE_INCLUDES ${PROJECT_SOURCE_DIR}/inc ${CMAKE_CURRENT_SOURCE_DIR}/inc)

#message(WARNING "------------------- {DANGER_ZONE_INCLUDES}: ${DANGER_ZONE_INCLUDES}")
#message(WARNING "------------------- {GAIA_INC}: ${GAIA_INC}")
#message(WARNING "------------------- {GAIA_DEFAULT_DIRECT_ACCESS_GENERATED_DIR}: ${GAIA_DEFAULT_DIRECT_ACCESS_GENERATED_DIR}")

set(DANGER_ZONE_DB_NAME danger_zone)
set(DANGER_ZONE_DDL ${PROJECT_SOURCE_DIR}/src/danger_zone.ddl)
set(DANGER_ZONE_RULESET ${PROJECT_SOURCE_DIR}/src/danger_zone.ruleset)

# --- I dont think this is needed ---
configure_file("${PROJECT_SOURCE_DIR}/src/gaia_conf.toml" "${PROJECT_BINARY_DIR}/gaia_conf.toml")
configure_file("${PROJECT_SOURCE_DIR}/src/log_conf.toml" "${PROJECT_BINARY_DIR}/log_conf.toml")

# --- with Gaia.Preview, it looks like target_include_directories is not being considered by 
# --- target_add_gaia_generated_sources. I have a header that I want the ruleset to take in, so
# --- I add it to a place where I know the build is looking. This is a blunt force solution which
# --- should be improved sometime soon.
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/inc/danger_zone.hpp"
  DESTINATION "${GAIA_DEFAULT_DIRECT_ACCESS_GENERATED_DIR}/danger_zone")

# --- add and link ---
add_executable(danger_zone src/danger_zone.cpp)
target_include_directories(danger_zone PRIVATE ${GAIA_INC} ${DANGER_ZONE_INCLUDES})

# --- Generate EDC from DDL---
process_schema(
  DDL_FILE ${DANGER_ZONE_DDL}
  DATABASE_NAME ${DANGER_ZONE_DB_NAME}
)

# -- Translate ruleset into CPP --
translate_ruleset(
  RULESET_FILE ${DANGER_ZONE_RULESET}
  DATABASE_NAME ${DANGER_ZONE_DB_NAME}
)

# --- add and link ---
target_add_gaia_generated_sources(danger_zone)
add_dependencies(danger_zone translate_danger_zone_ruleset)
ament_target_dependencies(danger_zone rclcpp std_msgs sensor_msgs vision_msgs danger_zone_msgs)
target_link_libraries(danger_zone ${GAIA_LIB})

install(TARGETS
  danger_zone
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
