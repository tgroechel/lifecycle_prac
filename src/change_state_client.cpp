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

#include <assert.h>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/cancel_transition.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;

static constexpr char const *lifecycle_node = "async_lc_node";

static constexpr char const *node_get_state_topic = "async_lc_node/get_state";
static constexpr char const *node_change_state_topic = "async_lc_node/change_state";
static constexpr char const *node_cancel_transition_topic = "async_lc_node/cancel_transition";

template <typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT &future, WaitTimeT time_to_wait)
{
    auto end = std::chrono::steady_clock::now() + time_to_wait;
    std::chrono::milliseconds wait_period(20);
    std::future_status status = std::future_status::timeout;
    do
    {
        auto now = std::chrono::steady_clock::now();
        auto time_left = end - now;
        if (time_left <= std::chrono::seconds(0))
        {
            break;
        }
        status = future.wait_for((time_left < wait_period) ? time_left
                                                           : wait_period);
    } while (rclcpp::ok() && status != std::future_status::ready);
    return status;
}

class LifecycleServiceClient : public rclcpp::Node
{
public:
    explicit LifecycleServiceClient(const std::string &node_name)
        : Node(node_name)
    {
    }

    void init()
    {
        client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
            node_get_state_topic);
        client_change_state_ =
            this->create_client<lifecycle_msgs::srv::ChangeState>(
                node_change_state_topic);

        client_cancel_transition_ =
            this->create_client<lifecycle_msgs::srv::CancelTransition>(
                node_cancel_transition_topic); // TODO @tgroechel: rename with node after update
    }

    unsigned int get_state(std::chrono::seconds time_out = 3s)
    {
        auto request =
            std::make_shared<lifecycle_msgs::srv::GetState::Request>();

        if (!client_get_state_->wait_for_service(time_out))
        {
            RCLCPP_ERROR(get_logger(),
                         "Service %s is not available.",
                         client_get_state_->get_service_name());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        auto future_result =
            client_get_state_->async_send_request(request).future.share();

        auto future_status = wait_for_result(future_result, time_out);

        if (future_status != std::future_status::ready)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Server time out while getting current state for node %s",
                lifecycle_node);
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        if (future_result.get())
        {
            RCLCPP_INFO(get_logger(),
                        "Node %s has current state %s.",
                        lifecycle_node,
                        future_result.get()->current_state.label.c_str());
            return future_result.get()->current_state.id;
        }
        else
        {
            RCLCPP_ERROR(get_logger(),
                         "Failed to get current state for node %s",
                         lifecycle_node);
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
    }

    bool change_state(std::uint8_t transition,
                      std::chrono::seconds time_out = 20s)
    {
        auto request =
            std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;

        if (!client_change_state_->wait_for_service(time_out))
        {
            RCLCPP_ERROR(get_logger(),
                         "Service %s is not available.",
                         client_change_state_->get_service_name());
            return false;
        }

        auto future_result =
            client_change_state_->async_send_request(request).future.share();

        auto future_status = wait_for_result(future_result, time_out);

        if (future_status != std::future_status::ready)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Server time out while changing state for node %s",
                lifecycle_node);
            return false;
        }

        if (future_result.get()->success)
        {
            RCLCPP_INFO(get_logger(),
                        "Transition %d successfully triggered.",
                        static_cast<int>(transition));
            return true;
        }
        else
        {
            RCLCPP_WARN(get_logger(),
                        "Failed to trigger transition %u",
                        static_cast<unsigned int>(transition));
            return false;
        }
    }

    bool cancel_transition(std::chrono::seconds timeout_sec = 10s)
    {
        auto request = std::make_shared<lifecycle_msgs::srv::CancelTransition::Request>();
        request->timeout_sec = ((float)timeout_sec.count()) - 0.05;
        
        if (!client_cancel_transition_->wait_for_service(timeout_sec))
        {
            RCLCPP_ERROR(get_logger(),
                         "Service %s is not available.",
                         client_cancel_transition_->get_service_name());
            return false;
        }

        RCLCPP_INFO(get_logger(), "Sending cancel transition");
        auto future_result =
            client_cancel_transition_->async_send_request(request).future.share();

        auto future_status = wait_for_result(future_result, timeout_sec);

        if (future_status != std::future_status::ready)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Server time out while cancelling transition for node %s",
                lifecycle_node);
            return false;
        }

        if (future_result.get()->success)
        {
            RCLCPP_INFO(get_logger(),
                        "Transition successfully cancelled.");
            return true;
        }
        else
        {
            RCLCPP_WARN(get_logger(),
                        "Failed to cancel transition with reason: %s", 
                        future_result.get()->error_msg.c_str());
            return false;
        }

    }

    void change_state_w_future_request(std::uint8_t transition)
    {
        auto request =
            std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;

        // We give the async_send_request() method a callback that will get executed once the response
        // is received.
        // This way we can return immediately from this method and allow other work to be done by the
        // executor in `spin` while waiting for the response.
        using ServiceResponseFuture = rclcpp::Client<
            lifecycle_msgs::srv::ChangeState>::SharedFutureWithRequest;

        auto response_received_callback =
            [logger = this->get_logger()](ServiceResponseFuture future)
        {
            auto request_response_pair = future.get();
            RCLCPP_INFO(logger,
                        "Transition (%d) request was a %s",
                        request_response_pair.first->transition.id,
                        request_response_pair.second->success ? "success"
                                                              : "failure");
        };
        auto result = client_change_state_->async_send_request(
            request, std::move(response_received_callback));
        RCLCPP_INFO(this->get_logger(),
                    "Sending a request to the server (request_id =%ld), we're "
                    "going to let you know the result when ready!",
                    result.request_id);
    }

private:
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>>
        client_get_state_;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>
        client_change_state_;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::CancelTransition>>
        client_cancel_transition_;
};

void callee_script(std::shared_ptr<LifecycleServiceClient> lc_client)
{
    double sec_between_frames = 0.5;
    double fps = 1.0 / sec_between_frames;

    rclcpp::WallRate time_between_state_changes(fps);

    bool transitioned{false};
    for(int i = 0; i < 5 && !transitioned; ++i)
    {
        std::cout << "Attempting TRANSITION_CONFIGURE #" << i << std::endl;
        transitioned = lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, 2s);
        if(!transitioned){
            lc_client->cancel_transition();
        }
    }
    std::cout << "CONFIGURED" << std::endl;

    time_between_state_changes.sleep();

    std::cout << "send TRANSITION_ACTIVATE" << std::endl;

    lc_client->change_state(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    time_between_state_changes.sleep();
    std::cout << "send TRANSITION_DEACTIVATE" << std::endl;
    lc_client->change_state(
        lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
}

void wake_executor(std::shared_future<void> future,
                   rclcpp::executors::SingleThreadedExecutor &exec)
{
    future.wait();
    exec.cancel();
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    auto lc_client = std::make_shared<LifecycleServiceClient>("lc_client");
    lc_client->init();

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(lc_client);

    std::shared_future<void> script =
        std::async(std::launch::async, std::bind(callee_script, lc_client));
    auto wake_exec = std::async(
        std::launch::async, std::bind(wake_executor, script, std::ref(exe)));

    exe.spin_until_future_complete(script);

    rclcpp::shutdown();

    return 0;
}