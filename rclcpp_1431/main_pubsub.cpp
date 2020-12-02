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
#define LEN_SET (200 * 1024)
//#define LEN_SET (10 * 1024 * 1024)

class MinimalPublisher : public rclcpp::Node
{
 public:
  MinimalPublisher(int msg_size)
      : Node("minimal_publisher"), count_(0), msg_size_(msg_size)
  {
    rclcpp::QoSInitialization qos_initialization = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default);

    //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", rclcpp::QoS(qos_initialization));
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    std::cout << "msg_size: " << msg_size_ << std::endl;
    timer_ = this->create_wall_timer(100ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    std::string tmp(msg_size_, 'a');
    message.data = tmp;
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    static int64_t old = 0;
    auto tt = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::cout << "time in nanosencond " << tt - old << std::endl;
    old = tt;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  int msg_size_;
};

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
  int msg_size = argc >= 2 ? std::stoi(argv[1]) : LEN_SET;
  rclcpp::executors::SingleThreadedExecutor exec;
  auto pub = std::make_shared<MinimalPublisher>(msg_size);
  auto sub = std::make_shared<MinimalSubscriber>();
  exec.add_node(pub);
  exec.add_node(sub);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
