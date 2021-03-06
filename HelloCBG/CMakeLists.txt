cmake_minimum_required(VERSION 3.5)
project(hello_callback_group)

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
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

set(DEPENDS
  rclcpp
  std_msgs
  )

set(MAIN hello_callback_group)
add_executable(${MAIN} src/main.cpp)
ament_target_dependencies(${MAIN} ${DEPENDS})

set(MAIN hello_callback_group_diff_priority)
add_executable(${MAIN} src/main_different_priority.cpp)
ament_target_dependencies(${MAIN} ${DEPENDS})

set(MAIN main_1node_2cb_2executor)
add_executable(${MAIN} src/main_1node_2cb_2executor.cpp)
ament_target_dependencies(${MAIN} ${DEPENDS})

install(TARGETS
  ${MAIN}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
