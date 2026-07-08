cmake_minimum_required(VERSION 3.10)
project(legged_mujoco)

set(CMAKE_CXX_STANDARD 17)  # MuJoCo 3.2 mjspec.h needs std::byte (C++17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  legged_hw
  legged_common
)

# MuJoCo
set(MUJOCO_DIR /usr/local/mujoco)
include_directories(${MUJOCO_DIR}/include)
link_directories(${MUJOCO_DIR}/lib)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp hardware_interface controller_manager legged_hw legged_common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/LeggedHWMujoco.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  mujoco
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# MuJoCo hardware loop node
add_executable(mujoco_hw_node src/mujoco_hw_node.cpp)
target_link_libraries(mujoco_hw_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS mujoco_hw_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
