cmake_minimum_required(VERSION 3.10)
project(solution_part2)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cg_interfaces REQUIRED)

# ============================================================================
# Create robot library
# ============================================================================
add_library(robot_lib src/robot.cpp)
ament_target_dependencies(robot_lib
    rclcpp
    cg_interfaces
)

target_include_directories(robot_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ============================================================================
# Create pathfinding library
# ============================================================================
add_library(pathfinding_lib src/pathfinding.cpp)
ament_target_dependencies(pathfinding_lib
    rclcpp
    cg_interfaces
)

target_include_directories(pathfinding_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ============================================================================
# Create maze_solver executable
# ============================================================================
add_executable(maze_solver src/main_solver.cpp)
ament_target_dependencies(maze_solver
    rclcpp
    cg_interfaces
)

target_include_directories(maze_solver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(maze_solver robot_lib pathfinding_lib)

# ============================================================================
# Install libraries
# ============================================================================
install(TARGETS robot_lib pathfinding_lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib
)

# ============================================================================
# Install executables
# ============================================================================
install(TARGETS maze_solver
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ============================================================================
# Install headers
# ============================================================================
install(DIRECTORY include/
    DESTINATION include
)

ament_package()
