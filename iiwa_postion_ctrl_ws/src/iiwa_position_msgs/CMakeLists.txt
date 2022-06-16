### Flags and standards
cmake_minimum_required(VERSION 3.12)
project(iiwa_position_msgs)

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
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs                
  std_msgs                      
  message_generation            
)

### Source code for action files
add_action_files(
    FILES
    goToJointPos.action)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
)


### Make package
catkin_package(
  CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime 
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)