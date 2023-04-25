#pragma once

#include <functional>
#include <chrono>
#include "rclcpp/waitable.hpp"
#include "rclcpp/rclcpp.hpp"



template <typename T>
class DeferrableCallbackWrapper{
public:
    using CallbackFunction = std::function<void(T)>;

    explicit DeferrableCallbackWrapper(CallbackFunction callback) : callback_(callback)
    {
    }

    bool is_ready()
    {
        return response_sent_;
    }

    void call()
    {
        callback_(data_);
    }

    void send_resp()
    {
        
    }

    void send_resp(T data)
    {
        data_ = data;
        response_sent_ = true;
    }

private:
    CallbackFunction callback_;
    T data_;
    bool response_sent_{false};
};

// Specialization to handle 'void' for call & send_resp
template <>
class DeferrableCallbackWrapper<void>{
public:
    using CallbackFunction = std::function<void()>;

    explicit DeferrableCallbackWrapper(CallbackFunction callback) : callback_(callback)
    {
    }

    bool is_ready()
    {
        return response_sent_;
    }

    void call()
    {
        callback_();
    }

    void send_resp()
    {
        response_sent_ = true;
    }

private:
    CallbackFunction callback_;
    bool response_sent_{false};
};


template<typename T>
class DeferrableCallbackWaitable : public rclcpp::Waitable
{
public:
    explicit DeferrableCallbackWaitable(std::shared_ptr<rclcpp::Context> context_ptr)
    {

        rcl_guard_condition_options_t guard_condition_options =
            rcl_guard_condition_get_default_options();

        // Guard condition is used by the wait set to handle execute-or-not logic
        gc_ = rcl_get_zero_initialized_guard_condition();
        rcl_guard_condition_init(
            &gc_, context_ptr->get_rcl_context().get(), guard_condition_options);
    }

    /**
     * @brief tell the CallbackGroup how many guard conditions are ready in this waitable
     */
    size_t get_number_of_ready_guard_conditions() { return 1; }

    /**
     * @brief tell the CallbackGroup that this waitable is ready to execute anything
     */
    bool is_ready(rcl_wait_set_t *wait_set) override
    {
        (void)wait_set;
        std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
        return deferrable_callback_ptr_->is_ready();
    }

    /**
     * @brief add_to_wait_set is called by rclcpp during NodeWaitables::add_waitable() and CallbackGroup::add_waitable()
      waitable_ptr = std::make_shared<DeferrableCallbackWrapper>();
      node->get_node_waitables_interface()->add_waitable(waitable_ptr, (rclcpp::CallbackGroup::SharedPtr) nullptr);
     */
    bool add_to_wait_set(rcl_wait_set_t *wait_set) override
    {
        std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
        rcl_ret_t ret = rcl_wait_set_add_guard_condition(wait_set, &gc_, NULL);
        return RCL_RET_OK == ret;
    }

    // XXX check this against the threading model of the multi-threaded executor.
    void execute()
    {
        std::lock_guard<std::recursive_mutex> lock(cb_mutex_);
        deferrable_callback_ptr_->call();
    }

    void add_callback(const std::shared_ptr<DeferrableCallbackWrapper<T>> &callback)
    {
        std::lock_guard<std::recursive_mutex> lock(cb_mutex_);
        deferrable_callback_ptr_ = callback;
        rcl_trigger_guard_condition(&gc_);
    }

    void add_callback(std::shared_ptr<DeferrableCallbackWrapper<T>> &&callback)
    {
        std::lock_guard<std::recursive_mutex> lock(cb_mutex_);
        deferrable_callback_ptr_ = std::move(callback);
        rcl_trigger_guard_condition(&gc_);
    }

    void remove_callback()
    {
        deferrable_callback_ptr_.reset();
    }

private:
    std::recursive_mutex reentrant_mutex_; //!< mutex to allow this callback to be added to multiple callback groups simultaneously
    rcl_guard_condition_t gc_;             //!< guard condition to drive the waitable

    std::recursive_mutex cb_mutex_; //!< mutex to allow this callback to be added to multiple callback groups simultaneously
    std::shared_ptr<DeferrableCallbackWrapper<T>> deferrable_callback_ptr_;
};
