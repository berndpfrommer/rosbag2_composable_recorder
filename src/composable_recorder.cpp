// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2021 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include "rosbag2_composable_recorder/composable_recorder.hpp"

#include <stdio.h>

#include <chrono>
#include <iomanip>
#include <rclcpp_components/register_node_macro.hpp>
#include <sstream>

namespace rosbag2_composable_recorder
{
static std::string get_time_stamp()
{
  std::stringstream datetime;
  auto now = std::chrono::system_clock::now();
  auto t_now = std::chrono::system_clock::to_time_t(now);
  datetime << std::put_time(std::localtime(&t_now), "%Y-%m-%d-%H-%M-%S");
  return (datetime.str());
}

ComposableRecorder::ComposableRecorder(const rclcpp::NodeOptions & options)
: rosbag2_transport::Recorder(
    std::make_shared<rosbag2_cpp::Writer>(), rosbag2_storage::StorageOptions(),
    rosbag2_transport::RecordOptions(), "recorder",
    rclcpp::NodeOptions(options).start_parameter_event_publisher(false))
{
  std::vector<std::string> topics =
    declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>());
  for (const auto & topic : topics) {
    RCLCPP_INFO_STREAM(get_logger(), "recording topic: " << topic);
  }
  // set storage options
  rosbag2_storage::StorageOptions & sopt = storage_options_;
  sopt.storage_id = declare_parameter<std::string>("storage_id", "sqlite3");
  sopt.uri = declare_parameter<std::string>("bag_prefix", "rosbag2_") + get_time_stamp();
  sopt.max_cache_size = declare_parameter<int>("max_cache_size", 100 * 1024 * 1024);

  // set recorder options
  rosbag2_transport::RecordOptions & ropt = record_options_;
  ropt.all = declare_parameter<bool>("record_all", false);
  ropt.is_discovery_disabled = declare_parameter<bool>("disable_discovery", false);
  ropt.rmw_serialization_format = declare_parameter<std::string>("serialization_format", "cdr");
  ropt.topic_polling_interval = std::chrono::milliseconds(100);
  ropt.topics.insert(ropt.topics.end(), topics.begin(), topics.end());

  stop_discovery_ = ropt.is_discovery_disabled;
  if (declare_parameter<bool>("start_recording_immediately", false)) {
    record();
  } else {
    service_ = create_service<std_srvs::srv::Trigger>(
      "start_recording",
      std::bind(
        &ComposableRecorder::startRecording, this, std::placeholders::_1, std::placeholders::_2));
  }
}

bool ComposableRecorder::startRecording(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  res->success = false;
  if (isRecording_) {
    RCLCPP_WARN(get_logger(), "already recording!");
    res->message = "already recording!";
  } else {
    RCLCPP_INFO(get_logger(), "starting recording...");
    try {
      record();
      isRecording_ = true;
      RCLCPP_INFO(get_logger(), "started recording successfully");
      res->success = true;
      res->message = "started recoding!";
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN(get_logger(), "cannot toggle recording!");
      res->message = "runtime error occurred: " + std::string(e.what());
    }
  }
  return (true);
}

ComposableRecorder::~ComposableRecorder() {}
}  // namespace rosbag2_composable_recorder

RCLCPP_COMPONENTS_REGISTER_NODE(rosbag2_composable_recorder::ComposableRecorder)
