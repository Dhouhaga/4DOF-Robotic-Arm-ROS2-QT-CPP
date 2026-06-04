cmake_minimum_required(VERSION 3.18)
project(pca9685_pi_hw_interface LANGUAGES CXX)
if(POLICY CMP0167)
        cmake_policy(SET CMP0167 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)



set(dependencies hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle)

add_library(pca9685_pi_hw_interface SHARED
        src/pca9685_pi_hw_interface.cpp
        src/i2c_driver.cpp
        src/pca9685_driver.cpp
)

target_include_directories(
        pca9685_pi_hw_interface  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/pca9685_pi_hw_interface>
)

ament_target_dependencies(
        pca9685_pi_hw_interface ${dependencies}
)

# prevent pluginlib from using boost
target_compile_definitions(pca9685_pi_hw_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface pca9685_pi_hw_interface.xml)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS pca9685_pi_hw_interface
        EXPORT export_${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_libraries(pca9685_pi_hw_interface)
ament_export_dependencies(${dependencies})

ament_package()
