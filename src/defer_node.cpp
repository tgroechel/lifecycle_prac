#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "lifecycle_prac/deferrable_callback_waitable.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

class DeferNode : public rclcpp::Node
{
public:
  DeferNode()
      : Node("defer_node"), dcw_()
  {
    auto callback = std::make_shared<DeferrableCallbackWrapper<void>>(
        std::bind(&DeferNode::print_data, this, std::placeholders::_1));

    dcw_ = make_shared<DeferrableCallbackWaitable>(rclcpp::contexts::default_context::get_global_default_context());

    dcw_->add_callback(std::move(callback));

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
    using ServiceResponseFuture =
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFutureWithRequest;
    auto response_received_callback =
        [logger = this->get_logger(), dcw_ = this->dcw_, deferal_sent = this->deferal_sent](ServiceResponseFuture future)
    {
      auto request_response_pair = future.get();
      RCLCPP_INFO(
          logger,
          "Result of %" PRId64 " + %" PRId64 " is: %" PRId64,
          request_response_pair.first->a,
          request_response_pair.first->b,
          request_response_pair.second->sum);
      dcw_->send_resp(request_response_pair.second->sum);
    };
    auto result = client_->async_send_request(
        request, std::move(response_received_callback));
    RCLCPP_INFO(
        this->get_logger(),
        "Sending a request to the server (request_id =%" PRId64
        "), we're going to let you know the result when ready!",
        result.request_id);
  }

private:
  bool deferal_sent{false};
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<DeferrableCallbackWaitable> dcw_;
  // create an async client
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeferNode>());
  rclcpp::shutdown();
  return 0;
}