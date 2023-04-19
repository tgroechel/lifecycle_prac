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

using namespace std::chrono_literals;
class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
	explicit LifecycleTalker(const std::string &node_name,
							 bool intra_process_comms = false)
		: rclcpp_lifecycle::LifecycleNode(
			  node_name,
			  rclcpp::NodeOptions().use_intra_process_comms(
				  intra_process_comms))
	{

		register_on_configure(std::bind(
								 &LifecycleTalker::on_configure, this,
								 std::placeholders::_1),
							 true); // is_async = true
		register_on_deactivate(std::bind(
								 &LifecycleTalker::on_deactivate, this,
								 std::placeholders::_1),
							 true); // is_async = true
		timer_ = this->create_wall_timer(
			std::chrono::milliseconds{250},
			
			std::bind(&LifecycleTalker::doing_work, this));
		// set up client_ to call the parameter server node for "param1"
		client_ = this->create_client<rclcpp::node_interfaces::GetParameters>(
			"parameter_node", rmw_qos_profile_services_default);

	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	on_configure(const rclcpp_lifecycle::State &)
	{
		pub_ = this->create_publisher<std_msgs::msg::String>(
			"lifecycle_chatter", 10);
			
		int sleep_time = 3;
		RCLCPP_INFO(this->get_logger(), "on_configure() {async} is called, getting `param1` from parameter_node", sleep_time);
		// get the parameter from the parameter server node
		auto request = std::make_shared<rclcpp::node_interfaces::GetParameters::Request>();
		request->names.push_back("param1");
		while (!client_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
			}
			RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
		}
		auto result_future = client_->async_send_request(request);
		if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
			rclcpp::executor::FutureReturnCode::SUCCESS)
		{
			RCLCPP_ERROR(this->get_logger(), "service call failed :(");
			return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
		}
		RCLCPP_INFO(this->get_logger(), "on_configure() done, returning success");
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
			CallbackReturn::SUCCESS;
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	on_activate(const rclcpp_lifecycle::State &state)
	{
		LifecycleNode::on_activate(state); // activates managed entities (i.e., lifecycle_publishers)

		int sleep_time = 3;
		RCLCPP_INFO(get_logger(), "on_activate() {sync} is called, sleeping for %d seconds.", sleep_time);
		std::this_thread::sleep_for(std::chrono::seconds{sleep_time});
		RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() done sleeping, returning success");

		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
			CallbackReturn::SUCCESS;
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	on_deactivate(const rclcpp_lifecycle::State &state)
	{
		LifecycleNode::on_deactivate(state);
		RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
		std::this_thread::sleep_for(3s);

		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
			CallbackReturn::SUCCESS;
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	on_cleanup(const rclcpp_lifecycle::State &)
	{
		timer_.reset();
		pub_.reset();

		RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
			CallbackReturn::SUCCESS;
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	on_shutdown(const rclcpp_lifecycle::State &state)
	{
		timer_.reset();
		pub_.reset();

		RCUTILS_LOG_INFO_NAMED(get_name(),
							   "on shutdown is called from state %s.",
							   state.label().c_str());

		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
			CallbackReturn::SUCCESS;
	}

	void publish()
	{
		static size_t count = 0;
		auto msg = std::make_unique<std_msgs::msg::String>();
		msg->data = "Lifecycle HelloWorld #" + std::to_string(++count);

		// Print the current state for demo purposes
		if (!pub_->is_activated())
		{
			RCLCPP_INFO(get_logger(),
						"Lifecycle publisher is currently "
						"inactive. Messages are not "
						"published.");
		}
		else
		{
			RCLCPP_INFO(get_logger(),
						"Lifecycle publisher is "
						"active. Publishing: [%s]",
						msg->data.c_str());
		}

		pub_->publish(std::move(msg));
	}

	void doing_work()
	{
		RCLCPP_INFO(this->get_logger(), "LC not blocked, time(%lf), in state (%s)",
					this->now().seconds(),
					this->get_current_state().label().c_str());
	}

private:
	std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>>
		pub_;
	std::shared_ptr<rclcpp::TimerBase> timer_;
	// create a client to get "param1" parameter from the parameter server node
	// the type should be GetParameters, which is defined in the rclcpp::node_interfaces::GetParameters
	// the client_ is used in the on_configure() callback function
	rclcpp::Client<rclcpp::node_interfaces::GetParameters>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor exe;

	std::shared_ptr<LifecycleTalker> lc_node =
		std::make_shared<LifecycleTalker>("lc_talker");

	exe.add_node(lc_node->get_node_base_interface());

	exe.spin();

	rclcpp::shutdown();

	return 0;
}