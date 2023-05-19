#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>
#include <chrono>

void add(
    const std::shared_ptr<rclcpp::Service<example_interfaces::srv::AddTwoInts>> service_handle,
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request)
{
    // response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld"
                                              " b: %ld",
                request->a, request->b);
    // send first response
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending first response then sleeping");
    example_interfaces::srv::AddTwoInts::Response resp;
    resp.sum = 1;
    service_handle->send_response(*header, resp);
    // sleep
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending second response then ending");

    // send second response
    example_interfaces::srv::AddTwoInts::Response resp2;
    resp.sum = 2;
    service_handle->send_response(*header, resp2);
    service_handle->send_response(*header, resp2);
    service_handle->send_response(*header, resp2);
    service_handle->send_response(*header, resp2);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "done");


    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
        node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}