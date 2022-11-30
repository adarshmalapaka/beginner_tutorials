// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>

#include "../include/beginner_tutorials/MinimalPublisher.hpp"

class TaskTalker : public testing::Test {
 public:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

 protected:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TaskTalker, test_num_publishers) {
  node_ = rclcpp::Node::make_shared("test_publisher");
  auto test_pub = node_->create_publisher<std_msgs::msg::String>
                    ("chatter", 10.0);

  auto num_pub = node_->count_publishers("chatter");
  EXPECT_EQ(1, num_pub);
}
