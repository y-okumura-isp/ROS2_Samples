#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

template<typename AllocatorT = std::allocator<void>>
class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(rclcpp::CallbackGroup::SharedPtr cb = nullptr)
  : Node("minimal_publisher"), count_(0)
  {
    rclcpp::PublisherOptionsWithAllocator<AllocatorT> options;
    options.callback_group = cb;

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10, options);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback, cb);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

template<typename AllocatorT = std::allocator<void>>
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(rclcpp::CallbackGroup::SharedPtr cb = nullptr)
  : Node("minimal_subscriber")
  {
    rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> options;
    options.callback_group = cb;
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [this](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      },
      options);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("no_use");
  auto cb = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto pub_node = std::make_shared<MinimalPublisher<>>(cb);
  auto sub_node = std::make_shared<MinimalSubscriber<>>(cb);

  auto exec = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
  exec->add_node(pub_node);
  exec->add_node(sub_node);

  exec->spin();
  rclcpp::shutdown();
  return 0;
}
