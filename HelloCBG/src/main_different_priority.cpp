#include <chrono>
#include <memory>
#include <pthread.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// based on https://github.com/boschresearch/ros2_examples/blob/experiment/cbg-executor-0.6.1/rclcpp/cbg-executor_ping-pong/main.cpp
void configure_thread(
    std::thread & t,
    size_t sched_priority,
    int policy,
    int cpu_id)
{
  struct sched_param params;
  params.sched_priority = sched_priority;
  if (pthread_setschedparam(t.native_handle(), policy, &params)) {
    std::cerr << "Failed to set scheduler parameters of thread!" << std::endl;;
  }

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(cpu_id, &cpuset);
  int ret = pthread_setaffinity_np(t.native_handle(), sizeof(cpu_set_t), &cpuset);
  if(ret < 0) {
    std::cerr << "Failed to set affinity: " << ret << std::endl;
  }
}


template<typename AllocatorT = std::allocator<void>>
class MinimalPubSub : public rclcpp::Node
{
public:
  MinimalPubSub(const std::string &node_name="minimal_pubsub")
  : Node(node_name), count_(0)
  {
    auto cb = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // publisher & its timer
    rclcpp::PublisherOptionsWithAllocator<AllocatorT> pub_options;
    pub_options.callback_group = cb;

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10, pub_options);
    auto timer_callback =
        [this, node_name]() -> void {
          auto message = std_msgs::msg::String();
          message.data = node_name + " says Hello, world! " + std::to_string(this->count_++);
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
  constexpr int cpuId = 1;

  rclcpp::init(argc, argv);

  auto node_be = std::make_shared<MinimalPubSub<>>("node_be");
  auto node_rt = std::make_shared<MinimalPubSub<>>("node_rt");

  auto exec_be = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
  auto exec_rt = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();

  exec_be->add_node(node_be);
  exec_rt->add_node(node_rt);

  std::thread thread_rt([&exec_rt]() {
      std::cout << "Thread with id=" << std::this_thread::get_id() << " exec_rt" << std::endl;
      exec_rt->spin();
    });
  configure_thread(thread_rt, 90, SCHED_FIFO, cpuId);

  std::thread thread_be([&exec_be]() {
      std::cout << "Thread with id=" << std::this_thread::get_id() << " exec_be" << std::endl;
      exec_be->spin();
    });
  configure_thread(thread_be, 0, SCHED_OTHER, cpuId);

  std::this_thread::sleep_for(10s);

  rclcpp::shutdown();
  thread_be.join();
  thread_rt.join();

  return 0;
}
