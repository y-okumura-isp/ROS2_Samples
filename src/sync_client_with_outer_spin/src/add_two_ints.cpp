// Copyright
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
#include <cinttypes>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

namespace my_demo_nodes_cpp
{
class ClientNode : public rclcpp::Node
{
  using ServiceResponseFuture =
      rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture;

public:
  explicit ClientNode(const rclcpp::NodeOptions & options)
      : Node("add_two_ints_client", options), cnt_(0)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    client_node_ = rclcpp::Node::make_shared(std::string(this->get_name())  + "__client");
    executor_->add_node(client_node_);
    client_ = client_node_->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    timer_ = create_wall_timer(1s, std::bind(&ClientNode::queue_async_request, this));
  }

  void queue_async_request()
  {
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = cnt_;
    request->b = cnt_ + 1;
    cnt_ += 1;

    auto future = client_->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "start_wait a = %" PRId64, request->a);

    auto c =  executor_->spin_until_future_complete(future, 100ms);
    std::string body = "";
    switch(c) {
      case rclcpp::FutureReturnCode::SUCCESS:
        body = " ret_value: " + std::to_string(future.get()->sum);
        break;
      case rclcpp::FutureReturnCode::TIMEOUT:
        body = " timeout";
        break;
      default:
        body = " interrupted";
    }
    RCLCPP_INFO(this->get_logger(), (std::string("end_wait") + body).c_str());
  }

private:
  int cnt_;
  rclcpp::Executor::SharedPtr executor_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr request_timeout_timer_;
};

class ServerNode : public rclcpp::Node
{
public:
  explicit ServerNode(const rclcpp::NodeOptions & options)
  : Node("add_two_ints_server", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto handle_add_two_ints =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) -> void
      {
        (void)request_header;

        auto sleep_ms = (request->a * 10) % 200;
        RCLCPP_INFO(
          this->get_logger(), "Incoming request\na: %" PRId64 " b: %" PRId64 " sleep ms: %" PRId64,
          request->a, request->b, sleep_ms);

        std::this_thread::sleep_for(std::chrono::duration<int64_t, std::milli>(sleep_ms));

        response->sum = request->a + request->b;
      };
    // Create a service that will use the callback function to handle requests.
    srv_ = create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", handle_add_two_ints);
  }

private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(my_demo_nodes_cpp::ClientNode)
RCLCPP_COMPONENTS_REGISTER_NODE(my_demo_nodes_cpp::ServerNode)
