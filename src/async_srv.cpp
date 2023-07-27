#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <thread>

using namespace std::placeholders;

class AsyncServiceExample : public rclcpp::Node {

public:
  explicit AsyncServiceExample(const std::string &node_name)
      : Node(node_name) {
    
    // Bind the add_w_param_req to the service
    m_service = this->create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints_w_param_req",
        std::bind(&AsyncServiceExample::add_w_param_req, this, _1, _2));
    
    // Create parameter client
    this->create_client<rcl_interfaces::srv::GetParameters>(
        "minimal_param_node/get_parameters");
  }

private:
  /**
   * Our callback for our adding service, it adds two ints plus another int from
   * a parameter request Note it does not have the optional service_handle in
   * the arguments Instead, we can just directly use the class member m_service
   */
  void add_w_param_req(
      const std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request>
          request) {
    /**
     * Create a future callback
     * This callback gets "thrown" onto the executor when m_param_client sends
     * its request This callback is then called when the response from the
     * parameter service is received From this, we can do our work and send our
     * original AddTwoInts Response
     */
    using ServiceResponseFuture = rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFutureWithRequest;
    auto param_response_received_callback =
        [header, request, this](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
          // Get the parameter response
          auto request_params = future.get();

          // Create response to original service request
          example_interfaces::srv::AddTwoInts::Response resp;
          resp.sum =
              request->a + request->b + request_params->values[1].int_value;

          // Send response using m_service member
          m_service->send_response(*header, resp);
        };

    // Sending the request and attaching the callback
    auto request =
        std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("random_int");
    m_param_client->async_send_request(request,
                                       std::move(param_response_received_callback));
  } // Service callback done, off Executor
    // Executor can now process incoming events (including the SharedFuture response)

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr m_service;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr m_param_client;
};
/**
 * Async callback signature:
 * 1. does not include a Response field
 * 2. optionally includes a service_handle; If you already have the pointer to
 * the service handle (e.g., a class member), you can omit this
 */
void add(
    const std::shared_ptr<rclcpp::Service<example_interfaces::srv::AddTwoInts>>
        service_handle,
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request>
        request) {
  /*Do work*/
  int res = request.a + request.b;

  /*Create response*/
  example_interfaces::srv::AddTwoInts::Response resp;
  resp.sum = res;

  /*Send result using service_handle, note this can be done from any thread*/
  service_handle->send_response(*header, resp);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("add_two_ints_server");

  // Bind `add` to this service callback, can also easily be a lamda
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
      node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints",
                                                                &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
