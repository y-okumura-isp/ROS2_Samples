cmake_minimum_required(VERSION 3.8)
project(sync_seq)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rcutils REQUIRED)

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "example_interfaces"
    "rclcpp"
    "rcutils")
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

function(add_demo_dependencies library)
  target_compile_definitions(${library}
    PRIVATE "DEMO_NODES_CPP_BUILDING_DLL")
  ament_target_dependencies(${library}
    "example_interfaces"
    "rclcpp"
    "rclcpp_components"
    "rcutils")
endfunction()

# custom_executable(add_two_ints_client)

add_library(services_library SHARED
  src/add_two_ints.cpp
  )
add_demo_dependencies(services_library)

rclcpp_components_register_node(services_library
  PLUGIN "my_demo_nodes_cpp::ClientNode"
  EXECUTABLE add_two_ints_client)
rclcpp_components_register_node(services_library
  PLUGIN "my_demo_nodes_cpp::ServerNode"
  EXECUTABLE add_two_ints_server)


install(TARGETS
  services_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
