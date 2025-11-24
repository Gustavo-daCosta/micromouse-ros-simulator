cmake_minimum_required(VERSION 3.10)
project(solution)

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
# Create map library from source
# ============================================================================
add_library(map_lib src/map.cpp)
ament_target_dependencies(map_lib
    rclcpp
    cg_interfaces
)

target_include_directories(map_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

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
# Create main_client executable (original test client)
# ============================================================================
add_executable(main_client src/main.cpp)
ament_target_dependencies(main_client
    rclcpp
    cg_interfaces
)

target_include_directories(main_client PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(main_client map_lib)

# ============================================================================
# Create maze_solver executable (PARTE 1 - Navegação com mapa)
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

target_link_libraries(maze_solver map_lib robot_lib pathfinding_lib)

# ============================================================================
# Install library
# ============================================================================
install(TARGETS map_lib robot_lib pathfinding_lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib
)

# ============================================================================
# Install executables
# ============================================================================
install(TARGETS main_client maze_solver
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ============================================================================
# Install headers
# ============================================================================
install(DIRECTORY include/
    DESTINATION include
)

# ============================================================================
# Install resources (launch, params, rviz, etc.)
# ============================================================================
install(DIRECTORY launch rviz params
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

ament_package()