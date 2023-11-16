#ifndef __CORE_HPP__
#define __CORE_HPP__

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

class Nav2Keyboard : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit Nav2Keyboard(const rclcpp::NodeOptions &options);
    void publish_message();
    void subscriber_callback(const std_msgs::msg::String::SharedPtr msg);
protected:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_active(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr service_client_;
    char key_;
};

int getch(void);

#endif

