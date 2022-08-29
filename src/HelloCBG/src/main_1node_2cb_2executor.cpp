#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "utils.hpp"
#include "sample_nodes.hpp"

int main(int argc, char * argv[])
{
  constexpr int cpuId = 1;

  rclcpp::init(argc, argv);

  auto node = std::make_shared<MinimalPubSub_2cb<>>();

  auto exec_be = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
  auto exec_rt = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();

  // pub + be
  exec_be->add_callback_group(node->cb_pub_, node->get_node_base_interface());
  // sub + rt
  exec_rt->add_callback_group(node->cb_sub_, node->get_node_base_interface());

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
