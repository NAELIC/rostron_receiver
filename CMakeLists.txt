cmake_minimum_required(VERSION 3.16.2)
project(rostron_receiver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rostron_interfaces REQUIRED)

find_package(Boost 1.56 REQUIRED COMPONENTS system)

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
endif(Boost_FOUND)

# Protobuf dependencies
add_subdirectory(include/rostron_receiver/proto)

# Create vision executable
SET(
  SRCS_VISION
  src/vision.cpp
  src/net/udp_receiver.cpp
)


SET(
  HEADERS_VISION
  include/rostron_receiver/net/udp_receiver.h
)

add_executable(vision ${SRCS_VISION} ${HEADERS_VISION})

target_link_libraries(vision ${Protobuf_LIBRARIES} NAELIC::Proto ${Boost_SYSTEM_LIBRARY})

target_include_directories(vision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Protobuf_INCLUDE_DIRS}
)

# TODO : Verify
ament_target_dependencies(vision rclcpp rostron_interfaces)
ament_export_dependencies(Boost Protobuf)
ament_export_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${Protobuf_LIBRARIES} NAELIC::Proto)

install(TARGETS vision
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
