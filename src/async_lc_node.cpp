// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

// include AsyncClientParameters to call the parameter server node
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/parameter_client.hpp"

using namespace std::chrono_literals;
class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit LifecycleTalker(const std::string &node_name,
                           bool intra_process_comms = false)
      : rclcpp_lifecycle::LifecycleNode(
            node_name, rclcpp::NodeOptions().use_intra_process_comms(
                           intra_process_comms)) {
    register_async_on_configure(std::bind(&LifecycleTalker::on_configure_async,
                                          this, std::placeholders::_1,
                                          std::placeholders::_2));
    register_async_on_activate(std::bind(&LifecycleTalker::on_activate_async,
                                         this, std::placeholders::_1,
                                         std::placeholders::_2));

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds{250},
                                std::bind(&LifecycleTalker::doing_work, this));

    client_ = this->create_client<rcl_interfaces::srv::GetParameters>(
        "minimal_param_node/get_parameters");
  }

  void on_configure_async(
      const rclcpp_lifecycle::State &,
      std::shared_ptr<rclcpp_lifecycle::ChangeStateHandler> change_state_hdl) {
    // Cancel monitoring
    transition_cancel_monitoring_timer_ = create_wall_timer(
        std::chrono::milliseconds{100}, [this, change_state_hdl]() {
          if (change_state_hdl->response_sent()) {
            transition_cancel_monitoring_timer_.reset();
            return;
          } else if (change_state_hdl
                         ->transition_is_cancelled()) { /*handle cancel*/
            size_t num_pruned_req = client_->prune_pending_requests();
            RCLCPP_INFO(this->get_logger(), "Handle cancel: pruned %ld request",
                        num_pruned_req);
            change_state_hdl->handled_transition_cancel(true);

            transition_cancel_monitoring_timer_.reset();
          }
        });

    // Callback for future response of getting a parameter
    auto response_received_callback =
        [logger = this->get_logger(), change_state_hdl](
            rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture
                future) {
          if (!change_state_hdl->response_sent()) {
            auto request_response_pair = future.get();
            RCLCPP_INFO(logger, "Received parameter response: %s",
                        request_response_pair->values[0].string_value.c_str());
            change_state_hdl->send_callback_resp(
                rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                    CallbackReturn::SUCCESS);
          }
        };

    // Sending the request and attaching the callback
    auto request =
        std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("param1");
    client_->async_send_request(request, std::move(response_received_callback));
  } // Off executor

  void on_activate_async(
      const rclcpp_lifecycle::State &state,
      std::shared_ptr<rclcpp_lifecycle::ChangeStateHandler> change_state_hdl) {
    LifecycleNode::on_activate(
        state); // activates managed entities (i.e., lifecycle_publishers)
    // create a thread to do some work passing the change_state_hdl to the
    // thread
    std::thread t(&LifecycleTalker::defer_on_activate_work, this,
                  change_state_hdl);
    t.detach();
  }

  void defer_on_activate_work(
      std::shared_ptr<rclcpp_lifecycle::ChangeStateHandler> change_state_hdl) {
    int sleep_time = 3;
    RCLCPP_INFO(this->get_logger(),
                "on_activate() {async} is called, sleeping is separate thread "
                "for %d seconds.",
                sleep_time);
    std::this_thread::sleep_for(std::chrono::seconds{sleep_time});
    RCLCPP_INFO(this->get_logger(),
                "on_activate() done sleeping, returning success");
    change_state_hdl->send_callback_resp(
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::SUCCESS);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &state) {
    int sleep_time = 3;
    LifecycleNode::on_deactivate(state);
    RCLCPP_INFO(this->get_logger(),
                "on_deactivate() {sync} is called, sleeping for %d seconds.",
                sleep_time);
    std::this_thread::sleep_for(std::chrono::seconds{sleep_time});
    RCLCPP_INFO(this->get_logger(),
                "on_deactivate() done sleeping, returning success");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) {
    timer_.reset();
    pub_.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &state) {
    timer_.reset();
    pub_.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called from state %s.",
                           state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  void publish() {
    static size_t count = 0;
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Lifecycle HelloWorld #" + std::to_string(++count);

    // Print the current state for demo purposes
    if (!pub_->is_activated()) {
      RCLCPP_INFO(get_logger(), "Lifecycle publisher is currently "
                                "inactive. Messages are not "
                                "published.");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Lifecycle publisher is "
                  "active. Publishing: [%s]",
                  msg->data.c_str());
    }

    pub_->publish(std::move(msg));
  }

  void doing_work() {
    RCLCPP_INFO(this->get_logger(), "LC not blocked, in state (%s)",
                this->get_current_state().label().c_str());
  }

private:
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>>
      pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
  std::shared_ptr<rclcpp::TimerBase> transition_cancel_monitoring_timer_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<LifecycleTalker> lc_node =
      std::make_shared<LifecycleTalker>("async_lc_node");

  exe.add_node(lc_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}