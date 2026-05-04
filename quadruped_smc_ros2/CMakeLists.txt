cmake_minimum_required(VERSION 3.8)
project(quadruped_smc_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()
set(CMAKE_CXX_STANDARD 17)

# ── Dependencies ──────────────────────────────────────────────
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(pluginlib          REQUIRED)
find_package(webots_ros2_driver REQUIRED)

# ── 1. Plugin shared library ──────────────────────────────────
add_library(${PROJECT_NAME} SHARED
    src/quadruped_driver.cpp
    src/SMC_controller.c
    src/SMC_controller_initialize.c
    src/SMC_controller_terminate.c
)

# C source biên dịch như C
set_source_files_properties(
    src/SMC_controller.c
    src/SMC_controller_initialize.c
    src/SMC_controller_terminate.c
    PROPERTIES LANGUAGE C
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    pluginlib
    webots_ros2_driver
)

# Link math library
target_link_libraries(${PROJECT_NAME} m)

pluginlib_export_plugin_description_file(webots_ros2_driver plugin.xml)

# ── 2. Keyboard teleop node ───────────────────────────────────
add_executable(gait_keyboard_node src/gait_keyboard_node.cpp)
ament_target_dependencies(gait_keyboard_node rclcpp std_msgs)

# ── Install ───────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(TARGETS gait_keyboard_node
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
    DESTINATION include
)
install(DIRECTORY launch config urdf resource
    DESTINATION share/${PROJECT_NAME}/
    OPTIONAL
)
install(FILES plugin.xml
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
