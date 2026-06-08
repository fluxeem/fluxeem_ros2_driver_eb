cmake_minimum_required(VERSION 3.8)
project(fluxeem_ros2_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Og")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og")

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -ffast-math")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -ffast-math")

set(BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "Enable architecture-aware optimization (-march=native)")

if(BUILD_WITH_MARCH_NATIVE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mtune=native -march=native")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native -march=native")
  message(STATUS "Architecture-aware optimization (-march=native): ENABLED")
else()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mtune=native")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native")
  message(STATUS "Architecture-aware optimization (-march=native): DISABLED")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(event_msgs REQUIRED)
find_package(camera_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(MetavisionSDK COMPONENTS core stream REQUIRED)

set(SRC_FILES
  src/fluxeem_ros2_publisher.cpp
  src/fluxeem_ros2_publisher_core.cpp
  src/fluxeem_ros2_publisher_setup.cpp
  src/fluxeem_ros2_publisher_services.cpp
  src/fluxeem_ros2_publisher_camera.cpp
  src/event_memory.cpp
  src/histogram_memory.cpp
  src/time_surface_memory.cpp
  src/event_to_image.cpp
  src/pixel_operations.cpp
)

add_executable(fluxeem_ros2_publisher_node ${SRC_FILES})
target_link_libraries(fluxeem_ros2_publisher_node
  MetavisionSDK::core
  MetavisionSDK::stream
  rt
)
target_include_directories(fluxeem_ros2_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(fluxeem_ros2_publisher_node PUBLIC c_std_99 cxx_std_20)
ament_target_dependencies(
  fluxeem_ros2_publisher_node
  rclcpp
  event_msgs
  camera_interfaces
  sensor_msgs
)

install(TARGETS fluxeem_ros2_publisher_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
