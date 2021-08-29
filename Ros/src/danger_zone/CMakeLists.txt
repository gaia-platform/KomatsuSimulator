cmake_minimum_required(VERSION 3.5)
project(danger_zone)

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
endif()

#TODO : sort this out
#set( CMAKE_PREFIX_PATH "/media/mark/Data1/U20/ros2/install/ament_cmake/share/colcon-core/packages : $(CMAKE_PREFIX_PATH)" ) 
#set( ament_cmake_DIR "/media/mark/Data1/U20/ros2/install/ament_cmake/share/colcon-core/packages" ) 

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(danger_zone_msgs REQUIRED)

set(GAIA_INC "/opt/gaia/include")                     #TODO: remove fixed path
set(GAIA_BIN "/opt/gaia/bin")                         #TODO: remove fixed path
set(GAIA_LIB "/opt/gaia/lib")                         #TODO: remove fixed path

set(GENERATED_OUTPUTS "${PROJECT_SOURCE_DIR}/generated")
file(MAKE_DIRECTORY ${GENERATED_OUTPUTS})

set(DANGER_ZONE_INCLUDES
  ${GAIA_INC}
  ${GAIA_INC}/gaia
  ${GAIA_INC}/gaia/db
  ${GAIA_INC}/gaia/direct_access
  ${GAIA_INC}/gaia/rules
  ${FLATBUFFERS_INC}
  ${GENERATED_OUTPUTS}
  "/media/mark/Data1/U20/develop/Unity/KomatsuSimulator/Ros/install/danger_zone_msgs/include"
)

set(GAIA_GENERATE ${GAIA_BIN}/gaiac)                             
set(GAIA_TRANSLATE ${GAIA_BIN}/gaiat)     

# --- Generate ---

#message( error "------------------- PSD ${PROJECT_SOURCE_DIR}  ------------------------" )
#COMMENT ${PROJECT_SOURCE_DIR}

set(DANGER_ZONE_INC ${PROJECT_SOURCE_DIR}/include)
set(DANGER_ZONE_DDL ${PROJECT_SOURCE_DIR}/src/danger_zone.ddl)
set(DANGER_ZONE_RULESET ${PROJECT_SOURCE_DIR}/src/danger_zone.ruleset)

process_schema(
  DDL_FILE ${DANGER_ZONE_DDL}
  OUTPUT_FOLDER ${GENERATED_OUTPUTS} 
  TARGET_NAME generate_danger_zone_headers
)

#--- Translate ---

translate_ruleset(
  RULESET_FILE ${DANGER_ZONE_RULESET}
  OUTPUT_FOLDER ${GENERATED_OUTPUTS}
  CLANG_PARAMS -I ${GAIA_INC} -I ${DANGER_ZONE_INC} 
  DEPENDS generate_danger_zone_headers)

configure_file("${PROJECT_SOURCE_DIR}/src/gaia_conf.toml" "${PROJECT_BINARY_DIR}/gaia_conf.toml")
configure_file("${PROJECT_SOURCE_DIR}/src/log_conf.toml" "${PROJECT_BINARY_DIR}/log_conf.toml")

add_library(gaia SHARED IMPORTED GLOBAL)
set_target_properties(gaia PROPERTIES IMPORTED_LOCATION ${GAIA_LIB}/libgaia.so) 

add_executable(danger_zone src/danger_zone.cpp ${GENERATED_OUTPUTS}/danger_zone_ruleset.cpp)
#add_executable(danger_zone src/danger_zone.cpp )
ament_target_dependencies(danger_zone rclcpp std_msgs sensor_msgs vision_msgs danger_zone_msgs)
target_link_libraries(danger_zone gaia)
target_include_directories(danger_zone PRIVATE ${DANGER_ZONE_INCLUDES})

install(TARGETS
danger_zone
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
