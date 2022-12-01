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

#include <chrono>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class TaskTalker : public testing::Test {
 protected:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TaskTalker, test_num_publishers) {
  node_ = rclcpp::Node::make_shared("test_num_pubs");
  auto test_pub = node_->create_publisher<std_msgs::msg::String>
                    ("chatter", 10.0);

  auto num_pub = node_->count_publishers("chatter");
  EXPECT_EQ(1, static_cast<int>(num_pub));
}

TEST_F(TaskTalker, test_tf2) {
  auto node_ = rclcpp::Node::make_shared("test_tf");
  auto tf_br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  auto tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  geometry_msgs::msg::TransformStamped t;
  rclcpp::Time now = node_->get_clock()->now();
  t.header.stamp = now;
  t.header.frame_id = "world";
  t.child_frame_id = "test";
  t.transform.translation.x = 0.1;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;
  tf_br->sendTransform(t);

  std::string tgt_frame = "test";
  std::string src_frame = "world";
  auto t_bool = tf_buffer->canTransform(tgt_frame, src_frame, now, 50ms);

  EXPECT_TRUE(t_bool);
}
