cmake_minimum_required(VERSION 3.5)

# {SDK-SETUP:CMAKE} - Do not remove this line

project(danger_zone)

# {SDK-SETUP:CXX_STANDARD} - Do not remove this line

include(/opt/gaia/cmake/gaia.cmake)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-c -Wall -Wextra -Wpedantic -ggdb)
  #add_link_options(-stdlib=libc++)
endif()

# Debug build-specific options.
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-O0 -g3 -ggdb -fno-limit-debug-info -fno-omit-frame-pointer -fno-optimize-sibling-calls -ggnu-pubnames -gsplit-dwarf)
  if(SANITIZER STREQUAL "ASAN")
    add_compile_options(-fsanitize=address -fsanitize=undefined -fno-sanitize-recover=all)
    add_link_options(-fsanitize=address -fsanitize=undefined -fno-sanitize-recover=all)
  endif()
else()
  add_compile_options(-O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(danger_zone_msgs REQUIRED)

set(DANGER_ZONE_DDL ${PROJECT_SOURCE_DIR}/src/danger_zone.ddl)
set(DANGER_ZONE_RULESET ${PROJECT_SOURCE_DIR}/src/danger_zone.ruleset)

configure_file("${PROJECT_SOURCE_DIR}/src/gaia_conf.toml" "${PROJECT_BINARY_DIR}/gaia_conf.toml")
configure_file("${PROJECT_SOURCE_DIR}/src/log_conf.toml" "${PROJECT_BINARY_DIR}/log_conf.toml")

# --- Generate EDC from DDL---
process_schema(DDL_FILE ${DANGER_ZONE_DDL})

# -- Translate ruleset into CPP --
translate_ruleset(RULESET_FILE ${DANGER_ZONE_RULESET})

#add_executable(danger_zone src/danger_zone.cpp ${GENERATED_OUTPUTS}/danger_zone_ruleset.cpp)
add_executable(danger_zone src/danger_zone.cpp)

target_add_gaia_generated_sources(danger_zone)
add_dependencies(danger_zone translate_danger_zone_ruleset)

ament_target_dependencies(danger_zone rclcpp std_msgs sensor_msgs vision_msgs danger_zone_msgs)

#target_include_directories(danger_zone PRIVATE ${DANGER_ZONE_INCLUDES})
target_include_directories(danger_zone PRIVATE ${GAIA_INC})

target_link_libraries(danger_zone gaia)

install(TARGETS
danger_zone
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
