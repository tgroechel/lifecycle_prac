#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "lifecycle_prac/deferrable_callback_waitable.hpp"

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
  : Node("minimal_param_node")
  {
    this->declare_parameter("param1", "world");
  }

private:
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}
