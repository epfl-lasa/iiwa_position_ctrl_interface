### Flags and standards
cmake_minimum_required(VERSION 3.12)
project(iiwa_postion_ctrl)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()
### Dependencies
list(APPEND CMAKE_PREFIX_PATH /opt/openrobots)

find_package(control_libraries 6.0.0 REQUIRED COMPONENTS
    state_representation
    robot_model)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    actionlib
    iiwa_position_msgs)

find_package(Boost REQUIRED COMPONENTS system)
add_compile_definitions(BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
add_compile_definitions(BOOST_MPL_LIMIT_LIST_SIZE=30)

catkin_package(CATKIN_DEPENDS iiwa_position_msgs
               LIBRARIES iiwa_postion_ctrl
               INCLUDE_DIRS include)

### Code directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    /opt/openrobots/include
)

### Create library
set(SRC_FILES
    src/type_conversions.cpp
    src/position_ctrl.cpp)

add_library(iiwa_postion_ctrl STATIC ${SRC_FILES})

target_link_libraries(iiwa_postion_ctrl
    ${catkin_LIBRARIES}
    ${control_libraries_LIBRARIES}
)

### Install commands
install(TARGETS iiwa_postion_ctrl
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )