// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

#include "../include/beginner_tutorials/MinimalPublisher.hpp"

using REQ = const std::shared_ptr<beginner_tutorials
                ::srv::UpdateMessage::Request>;
using RESP = std::shared_ptr<beginner_tutorials
                ::srv::UpdateMessage::Response>;

// Global string to hold published message data
auto publish_message = std::string("Custom String Message for Printing");

MinimalPublisher::MinimalPublisher()
    : Node("minimal_publisher"), count_(0) {
       publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
       timer_ = this->create_wall_timer(500ms,
        std::bind(&MinimalPublisher::timer_callback, this));

       auto serviceCallbackPtr = std::bind(&MinimalPublisher::change_message,
                      this, std::placeholders::_1, std::placeholders::_2);
       service_ = create_service<beginner_tutorials::srv::UpdateMessage>(
        "update_message", serviceCallbackPtr);
}

void MinimalPublisher::timer_callback() {
    auto message = std_msgs::msg::String();
    auto id_str = std::string(", ID: ");
    message.data = publish_message + id_str + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

void MinimalPublisher::change_message(REQ req, RESP resp) {
    if (req->data == std::string("None")) {
        RCLCPP_WARN(this->get_logger(), "Received an empty request string!");
    } else {
        resp->new_message = req->data;
        publish_message = resp->new_message;
        RCLCPP_INFO(this->get_logger(), "Request Received: '%s'",
                                req->data.c_str());
        RCLCPP_INFO(this->get_logger(), "Response Sent: '%s'",
                                resp->new_message.c_str());
    }
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
