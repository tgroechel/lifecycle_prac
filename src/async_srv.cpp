#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>
#include <chrono>
#include <thread>

/**
 * Async callback signature:
 * 1. does not include a Response field
 * 2. optionally includes a service_handle; If you already have the pointer to the service handle (e.g., a class member), you can omit this
*/
void add(
  const std::shared_ptr<rclcpp::Service<example_interfaces::srv::AddTwoInts>> service_handle,
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request)
{
  /*Do work*/
  int res = request.a + request.b;

  /*Create response*/
  example_interfaces::srv::AddTwoInts::Response resp;
  resp.sum = res;

  /*Send result using service_handle, note this can be done from any thread*/
  service_handle->send_response(*header, resp);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  // Bind `add` to this service callback, can also easily be a lamda
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
