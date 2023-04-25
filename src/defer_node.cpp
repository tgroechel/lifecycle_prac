#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "lifecycle_prac/deferrable_callback_waitable.hpp"

using namespace std::chrono_literals;

class DeferNode : public rclcpp::Node
{
public:
  DeferNode()
      : Node("minimal_param_node"), dcw_()
  {
    auto callback = std::make_shared<DeferrableCallbackWrapper<void>>(
        std::bind(&DeferNode::print_data, this, std::placeholders::_1));

    dcw_ = make_shared<DeferrableCallbackWaitable>(rclcpp::contexts::default_context::get_global_default_context());

    dcw_->add_callback(std::move(callback));

    this->get_node_waitables_interface()
        ->add_waitable(
            dcw_, (rclcpp::CallbackGroup::SharedPtr) nullptr);
  }

  void print_data(int data)
  {
    std::cout << "response: " << data << std::endl;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<DeferrableCallbackWaitable> dcw_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeferNode>());
  rclcpp::shutdown();
  return 0;
}