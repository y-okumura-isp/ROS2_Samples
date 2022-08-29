#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

template<typename AllocatorT = std::allocator<void>>
class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    cb_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::PublisherOptionsWithAllocator<AllocatorT> pub_options;
    pub_options.callback_group = cb_;

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10, pub_options);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback, cb_);
  }

  rclcpp::CallbackGroup::SharedPtr get_callback_group() {
    return cb_;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::CallbackGroup::SharedPtr cb_;
  size_t count_;
};

template<typename AllocatorT = std::allocator<void>>
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    cb_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> sub_options;
    sub_options.callback_group = cb_;

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [this](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      },
      sub_options);
  }

  rclcpp::CallbackGroup::SharedPtr get_callback_group() {
    return cb_;
  }

 private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr cb_;
};

template<typename AllocatorT = std::allocator<void>>
class MinimalPubSub_2cb : public rclcpp::Node
{
public:
  MinimalPubSub_2cb()
  : Node("minimal_pubsub_2cb"), count_(0)
  {
    // init callback groups
    cb_pub_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cb_sub_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // publisher & its timer
    rclcpp::PublisherOptionsWithAllocator<AllocatorT> pub_options;
    pub_options.callback_group = cb_pub_;

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10, pub_options);
    auto timer_callback =
        [this]() -> void {
          auto message = std_msgs::msg::String();
          message.data = " says Hello, world! " + std::to_string(this->count_++);
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
          this->publisher_->publish(message);
        };
    timer_ = this->create_wall_timer(500ms, timer_callback, cb_pub_);

    // subscriber
    rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> sub_options;
    sub_options.callback_group = cb_sub_;
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [this](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      },
      sub_options);
  }

  rclcpp::CallbackGroup::SharedPtr cb_pub_;
  rclcpp::CallbackGroup::SharedPtr cb_sub_;

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  size_t count_;
};

