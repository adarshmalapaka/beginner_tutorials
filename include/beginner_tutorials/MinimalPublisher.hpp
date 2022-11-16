/**
 * @file MinimalPublisher.hpp
 * @author Adarsh Malapaka (adarshmalapaka98@gmail.com)
 * @brief Class implementation of the Minimal publisher
 * @version 0.2
 * @date 2022-11-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/logging.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/update_message.hpp"

using namespace std::chrono_literals;
using REQ = const std::shared_ptr<beginner_tutorials
                ::srv::UpdateMessage::Request>;
using RESP = std::shared_ptr<beginner_tutorials
                ::srv::UpdateMessage::Response>;

/**
 * @brief Class (subclass of Node) and uses std::bind() to register a member function as a callback from the timer.
 * 
 */
class MinimalPublisher : public rclcpp::Node {
 public:
    MinimalPublisher();

 private:
    void timer_callback();
    void change_message(REQ req, RESP resp);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Service<beginner_tutorials::srv::UpdateMessage>::SharedPtr service_;
    size_t count_;
    std::string publish_message_;  // String to hold the message to be published
};
