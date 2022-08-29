// Copyright 2021 Research Institute of Systems Planning, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "message_filters/subscriber.h"

using namespace std::chrono_literals;

typedef sensor_msgs::msg::PointCloud2 Msg;
typedef std::shared_ptr<sensor_msgs::msg::PointCloud2 const> MsgConstPtr;
typedef std::shared_ptr<sensor_msgs::msg::PointCloud2> MsgPtr;

namespace sample_message_filter
{
// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class SampleSubscriberWithHeader : public rclcpp::Node
{
public:
  explicit SampleSubscriberWithHeader(const rclcpp::NodeOptions & options)
  : Node("sub_with_header", options)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto sub_callback =
      [this](MsgConstPtr msg) -> void
      {
        (void) msg;
        RCLCPP_INFO(this->get_logger(), "recieved");
      };

    sub_pc_.subscribe(this, "topic_with_stamp", qos.get_rmw_qos_profile());
    sub_pc_.registerCallback(sub_callback);
  }

private:
  message_filters::Subscriber<Msg> sub_pc_;
};

}  // namespace tilde_sample

RCLCPP_COMPONENTS_REGISTER_NODE(sample_message_filter::SampleSubscriberWithHeader)
