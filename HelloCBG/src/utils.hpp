#include <pthread.h>

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
    std::cerr << "Failed to set scheduler parameters of thread! id=" << t.get_id() << std::endl;;
  }

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(cpu_id, &cpuset);
  int ret = pthread_setaffinity_np(t.native_handle(), sizeof(cpu_set_t), &cpuset);
  if(ret < 0) {
    std::cerr << "Failed to set affinity: " << ret << " id=" << t.get_id() << std::endl;
  }
}

