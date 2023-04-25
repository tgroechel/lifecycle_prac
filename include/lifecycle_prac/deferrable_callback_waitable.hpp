#pragma once

#include <functional>
#include <chrono>
#include "rclcpp/waitable.hpp"

template <typename T>
class DeferrableCallbackWrapper
{
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
        response_sent_ = false;
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
class DeferrableCallbackWrapper<void>
{
public:
    using CallbackFunction = std::function<void()>;

    explicit DeferrableCallbackWrapper(CallbackFunction callback) : callback_(callback)
    {
    }

    void call()
    {
        callback_();
        response_sent_ = false;
    }

    void send_resp()
    {
        response_sent_ = true;
    }

private:
    CallbackFunction callback_;
    bool response_sent_{false};
};

template <typename T>
class DeferrableCallbackWaitable : public rclcpp::Waitable
{
public:
    DeferrableCallbackWaitable()
    {
        std::shared_ptr<rclcpp::Context> context_ptr = rclcpp::contexts::get_global_default_context();
        rcl_guard_condition_options_t guard_condition_options =
            rcl_guard_condition_get_default_options();

        gc_ = rcl_get_zero_initialized_guard_condition();
        auto ret = rcl_guard_condition_init(
            &gc_, context_ptr->get_rcl_context().get(), guard_condition_options);
        (void)ret;
    }

    size_t get_number_of_ready_guard_conditions() { return 1; }

    bool is_ready(rcl_wait_set_t *wait_set) override
    {
        (void)wait_set;
        std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
        return deferrable_callback_ptr_->is_ready();
    }

    virtual void add_to_wait_set(rcl_wait_set_t *wait_set) override
    {
        std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);
        auto ret = rcl_wait_set_add_guard_condition(wait_set, &gc_, NULL);
        (void)ret;
    }

    virtual void
    execute(std::shared_ptr<void> &data) override
    {
        (void)data;
        deferrable_callback_ptr_->call();
    }

    std::shared_ptr<void>
    take_data()
    {
        return deferrable_callback_ptr_;
    }

    void add_callback(const std::shared_ptr<DeferrableCallbackWrapper<T>> &callback)
    {
        std::lock_guard<std::recursive_mutex> lock(cb_mutex_);
        deferrable_callback_ptr_ = callback;
        auto ret = rcl_trigger_guard_condition(&gc_);
        (void)ret;
    }

    void add_callback(std::shared_ptr<DeferrableCallbackWrapper<T>> &&callback)
    {
        std::lock_guard<std::recursive_mutex> lock(cb_mutex_);
        deferrable_callback_ptr_ = std::move(callback);
        auto ret = rcl_trigger_guard_condition(&gc_);
        (void)ret;
    }

    void remove_callback()
    {
        deferrable_callback_ptr_.reset();
    }

    std::shared_ptr<DeferrableCallbackWrapper<T>> get_cb()
    {
        return deferrable_callback_ptr_;
    }

private:
    std::recursive_mutex reentrant_mutex_; //!< mutex to allow this callback to be added to multiple callback groups simultaneously
    rcl_guard_condition_t gc_;             //!< guard condition to drive the waitable

    std::recursive_mutex cb_mutex_; //!< mutex to allow this callback to be added to multiple callback groups simultaneously
    std::shared_ptr<DeferrableCallbackWrapper<T>> deferrable_callback_ptr_;
};
