#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"

using namespace std::chrono_literals;

typedef sensor_msgs::msg::PointCloud2 Msg;
typedef std::shared_ptr<sensor_msgs::msg::PointCloud2 const> MsgConstPtr;
typedef std::shared_ptr<sensor_msgs::msg::PointCloud2> MsgPtr;

namespace sample_message_filter
{
// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class SampleSyncWithHeader : public rclcpp::Node
{
  using SyncPolicy = message_filters::sync_policies::ExactTime<Msg, Msg>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

public:
  explicit SampleSyncWithHeader(const rclcpp::NodeOptions & options)
  : Node("sub_with_header", options)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    sub_pc1_.subscribe(this, "in1", qos.get_rmw_qos_profile());
    sub_pc2_.subscribe(this, "in2", qos.get_rmw_qos_profile());
    sync_ptr_ = std::make_shared<Sync>(SyncPolicy(5), sub_pc1_, sub_pc2_);

    /*
    // compile error because signal9 does not have
    // function object version addCallback()
    auto sub_callback =
      [this](const MsgConstPtr &msg1,
             const MsgConstPtr &msg2) -> void
      {
        (void) msg1;
        (void) msg2;
        RCLCPP_INFO(this->get_logger(), "recieved");
      };
    sync_ptr_->registerCallback(sub_callback);
    */
    sync_ptr_->registerCallback(std::bind(&SampleSyncWithHeader::sub_callback2, this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));
  }

private:
  message_filters::Subscriber<Msg> sub_pc1_;
  message_filters::Subscriber<Msg> sub_pc2_;

  std::shared_ptr<Sync> sync_ptr_;

  void sub_callback2(const MsgConstPtr &msg1,
                     const MsgConstPtr &msg2)
  {
    (void) msg1;
    (void) msg2;
    std::cout << "sub_callback2" << std::endl;
  }
};

}  // namespace tilde_sample

RCLCPP_COMPONENTS_REGISTER_NODE(sample_message_filter::SampleSyncWithHeader)
