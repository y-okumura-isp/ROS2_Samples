cmake_minimum_required(VERSION 3.8)
project(sample_message_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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

add_library(sample_message_filter SHARED
  src/sample_publisher.cpp
  src/sample_subscriber.cpp)
ament_target_dependencies(sample_message_filter
  "rclcpp"
  "message_filters"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs")

rclcpp_components_register_node(sample_message_filter
  PLUGIN "sample_message_filter::SamplePublisherWithStamp"
  EXECUTABLE publisher_with_stamp)
rclcpp_components_register_node(sample_message_filter
  PLUGIN "sample_message_filter::SamplePublisherWithoutStamp"
  EXECUTABLE publisher_without_stamp)
rclcpp_components_register_node(sample_message_filter
  PLUGIN "sample_message_filter::SampleSubscriberWithHeader"
  EXECUTABLE subscriber_with_header)

install(TARGETS
  sample_message_filter)
install(
  TARGETS sample_message_filter EXPORT sample_message_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
