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
find_library(state_representation REQUIRED)
find_library(robot_model REQUIERD)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    actionlib
    iiwa_position_msgs)

list(APPEND CMAKE_PREFIX_PATH /opt/openrobots)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(osqp REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
add_compile_definitions(BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
add_compile_definitions(BOOST_MPL_LIMIT_LIST_SIZE=30)

catkin_package(CATKIN_DEPENDS iiwa_position_msgs
               LIBRARIES ctrl_position_interface)


### Code directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${STATE_REPRESENTATION_INCLUDE_DIR}
    ${ROBOT_MODEL_INCLUDE_DIR}
    ${PINOCCHIO_INCLUDE_DIR}
    ${OsqpEigen_INCLUDE_DIR}
    /opt/openrobots/include
)

### Create library
set(SRC_FILES
    src/position_ctrl.cpp
    src/type_conversions.cpp)
add_library(ctrl_position_interface ${SRC_FILES})
target_link_libraries(ctrl_position_interface
    ${catkin_LIBRARIES}
    ${PINOCCHIO_LIBRARIES}
    OsqpEigen::OsqpEigen
    osqp::osqp
    state_representation
    robot_model
)

### Install commands
install(TARGETS ctrl_position_interface
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)