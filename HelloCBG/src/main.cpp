#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

template<typename AllocatorT = std::allocator<void>>
class MinimalPubSub : public rclcpp::Node
{
public:
  MinimalPubSub()
  : Node("minimal_pubsub"), count_(0)
  {
    auto cb = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // publisher & its timer
    rclcpp::PublisherOptionsWithAllocator<AllocatorT> pub_options;
    pub_options.callback_group = cb;

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10, pub_options);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback, cb);

    // subscriber
    rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> sub_options;
    sub_options.callback_group = cb;
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [this](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      },
      sub_options);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MinimalPubSub<>>();

  auto exec = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
  exec->add_node(node);

  exec->spin();
  rclcpp::shutdown();
  return 0;
}
