# lifecycle_prac
This is a sandbox repository used for testing different `rclcpp/lifecycle_node` features.

The current core feature request is for async lifecycle transition functions (e.g., `on_configure`) that are cancelable.
The demo is outlined in [Async Lifecycle Node Example](#async-lifecycle-node-example) below.

## Async Lifecycle Node Example:
This demo is scaffolded from the `rclcpp::Lifecycle` [demo here]().

### Files
- `src/async_lc_node.cpp`: the async lifecycle node
- `src/change_state_client.cpp`: the node for requesting change states and cancels of `async_lc_node`
- `src/parameter_node.cpp`: simple parameter server


### Video
`.mkv` here
> Left: AsyncLCNode; Top right: ChangeStateClient; Bottom right;
1. AsyncLCNode has a walltimer that prints every to demonstrate if the executor is blocked or not
2. ChangeStateClient attempts to transition sequence of CONFIGURE → ACTIVATE → DEACTIVATE, cancelling + retrying each if a 2s timeout is hit without a response
3. Async LC Node in on_configure sends a request to the parameter node (which is not running at the beginning).
It should* be checking if this exists but this is demonstrating a handing transition (server exists → server goes down → client sends request will hang)
4. Once the parameter node is started up, the `on_configure` can successfully complete
5. Activate request sent from ChangeStateClient
6. AsyncLCNode `on_activate` spawns another thread that sleeps for 3 seconds, responds when completed
7. Deactivate request sent form ChangeStateClient
8. AsyncLCNode `on_deactivate` is a synchronous transition that sleeps for 3 seconds, blocking the executor, and returning

