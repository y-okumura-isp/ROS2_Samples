#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
   member function as a callback from the timer. */
//#define LEN_SET (200 * 1024)
#define LEN_SET (10 * 1024 * 1024)

class MinimalSubscriber : public rclcpp::Node
{
 public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    rclcpp::QoSInitialization qos_initialization = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default);

    //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", rclcpp::QoS(qos_initialization));
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic",
        10,
        [this](std_msgs::msg::String::UniquePtr msg) {
          // RCLCPP_INFO(this->get_logger(), "I got message" + std::to_string(msg->data.size()));
        });
  }

 private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
