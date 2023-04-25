#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include <rclcpp/rclcpp.hpp>
#include "lifecycle_prac/deferrable_callback_waitable.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;
using AddTwoInts = example_interfaces::srv::AddTwoInts;


class DeferNode : public rclcpp::Node
{
public:
  DeferNode()
      : Node("defer_node")
  {
    auto callback = std::make_shared<DeferrableCallbackWrapper<int>>(
        std::bind(&DeferNode::print_data, this, std::placeholders::_1));

    dcw_ = std::make_shared<DeferrableCallbackWaitable<int>>();

    dcw_->add_callback(callback);

    this->get_node_waitables_interface()
        ->add_waitable(
            dcw_, (rclcpp::CallbackGroup::SharedPtr) nullptr);
    // initialize timer
    timer_ = this->create_wall_timer(
        1s, std::bind(&DeferNode::timer_callback, this));

    // create a client
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
  }

  void print_data(int data)
  {
    std::cout << "response: " << data << std::endl;
  }

  void timer_callback()
  {
    if (deferal_sent)
    {
      return;
    }
    deferal_sent = true;


    auto request = std::make_shared<AddTwoInts::Request>();
    request->a = 1;
    request->b = 2;

    using ServiceResponseFuture =
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFutureWithRequest;
    auto response_received_callback =
        [logger = this->get_logger(), dcw_ = this->dcw_, deferal_sent = this->deferal_sent](ServiceResponseFuture future)
    {
      auto request_response_pair = future.get();
      RCLCPP_INFO(
          logger,
          "Result of %ld + %ld is: %ld",
          request_response_pair.first->a,
          request_response_pair.first->b,
          request_response_pair.second->sum);
      dcw_->get_cb()->send_resp(request_response_pair.second->sum);
    };
    auto result = client_->async_send_request(
        request, std::move(response_received_callback));
    RCLCPP_INFO(
        this->get_logger(),
        "Sending a request to the server (request_id =%ld), we're going to let you know the result when ready!",
        result.request_id);
  }

private:
  bool deferal_sent{false};
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<DeferrableCallbackWaitable<int>> dcw_;
  // create an async client
  rclcpp::Client<AddTwoInts>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeferNode>());
  rclcpp::shutdown();
  return 0;
}