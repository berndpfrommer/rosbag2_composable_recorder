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
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <iomanip>
#include <rclcpp_components/register_node_macro.hpp>
#include <sstream>
#include <stdexcept>

namespace rosbag2_composable_recorder
{
namespace
{
std::string resolve_topic_name(
  const std::string & topic_name, const rclcpp::Logger & logger, const std::string & node_name,
  const std::string & node_namespace, const char * context)
{
  try {
    return rclcpp::expand_topic_or_service_name(topic_name, node_name, node_namespace, false);
  } catch (const std::exception & ex) {
    RCLCPP_WARN_STREAM(
      logger, "failed to expand " << context << " topic name '" << topic_name << "': " << ex.what()
                                  << ". using raw topic name.");
    return topic_name;
  }
}

rclcpp::QoS make_qos_from_yaml_node(
  const YAML::Node & qos_node, const rclcpp::Logger & logger, const std::string & topic_name)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(rmw_qos_profile_default.depth));

  if (qos_node["history"]) {
    const auto history = qos_node["history"].as<std::string>();
    if (history == "keep_last") {
      // depth is handled below.
    } else if (history == "keep_all") {
      qos.keep_all();
    } else {
      RCLCPP_WARN_STREAM(
        logger, "unsupported history='" << history << "' for topic '" << topic_name
                                        << "', using keep_last.");
    }
  }

  if (qos_node["depth"]) {
    const auto depth = qos_node["depth"].as<int>();
    if (depth < 1) {
      RCLCPP_WARN_STREAM(
        logger, "invalid depth=" << depth << " for topic '" << topic_name << "', using depth=1.");
      if (qos.get_rmw_qos_profile().history != RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
        qos.keep_last(1);
      }
    } else if (qos.get_rmw_qos_profile().history != RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
      qos.keep_last(static_cast<size_t>(depth));
    }
  }

  if (qos_node["reliability"]) {
    const auto reliability = qos_node["reliability"].as<std::string>();
    if (reliability == "reliable") {
      qos.reliable();
    } else if (reliability == "best_effort") {
      qos.best_effort();
    } else {
      RCLCPP_WARN_STREAM(
        logger, "unsupported reliability='" << reliability << "' for topic '" << topic_name
                                            << "', using system default.");
    }
  }

  if (qos_node["durability"]) {
    const auto durability = qos_node["durability"].as<std::string>();
    if (durability == "volatile") {
      qos.durability_volatile();
    } else if (durability == "transient_local") {
      qos.transient_local();
    } else {
      RCLCPP_WARN_STREAM(
        logger, "unsupported durability='" << durability << "' for topic '" << topic_name
                                           << "', using system default.");
    }
  }

  return qos;
}

void load_qos_profile_overrides_from_file(
  const std::string & qos_profile_overrides_path,
  std::unordered_map<std::string, rclcpp::QoS> & topic_qos_profile_overrides,
  const rclcpp::Logger & logger, const std::string & node_name, const std::string & node_namespace)
{
  // Parse QoS override YAML with yaml-cpp directly for compatibility with setups
  // where rosbag2_storage QoS YAML helper headers are not available via includes.
  try {
    YAML::Node yaml_file = YAML::LoadFile(qos_profile_overrides_path);
    std::unordered_map<std::string, rclcpp::QoS> qos_overrides;
    for (const auto & topic_qos : yaml_file) {
      const auto topic_name = topic_qos.first.as<std::string>();
      auto resolved_topic_name =
        resolve_topic_name(topic_name, logger, node_name, node_namespace, "QoS override");

      qos_overrides.emplace(
        resolved_topic_name,
        make_qos_from_yaml_node(topic_qos.second, logger, resolved_topic_name));
    }
    topic_qos_profile_overrides = std::move(qos_overrides);
  } catch (const YAML::Exception & ex) {
    throw std::runtime_error(std::string("Exception on parsing QoS overrides file: ") + ex.what());
  }

  RCLCPP_INFO_STREAM(
    logger, "loaded " << topic_qos_profile_overrides.size()
                      << " QoS override entries from: " << qos_profile_overrides_path);
}
}  // namespace

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
    rclcpp::NodeOptions(options).start_parameter_event_publisher(false)),
  bag_name_(declare_parameter<std::string>("bag_name", "")),
  bag_prefix_(declare_parameter<std::string>("bag_prefix", "rosbag2_"))
{
  const std::vector<std::string> configured_topics =
    declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>());
  std::vector<std::string> topics;
  topics.reserve(configured_topics.size());
  for (const auto & topic : configured_topics) {
    auto resolved_topic =
      resolve_topic_name(topic, get_logger(), get_name(), get_namespace(), "record");
    topics.emplace_back(resolved_topic);
    RCLCPP_INFO_STREAM(get_logger(), "recording topic: " << resolved_topic);
  }
  // set storage options
#ifdef USE_GET_STORAGE_OPTIONS
  rosbag2_storage::StorageOptions & sopt = get_storage_options();
#else
  rosbag2_storage::StorageOptions & sopt = storage_options_;
#endif
  sopt.storage_id = declare_parameter<std::string>("storage_id", "sqlite3");
  sopt.max_cache_size = declare_parameter<int>("max_cache_size", 100 * 1024 * 1024);
  sopt.max_bagfile_size = declare_parameter<int>("max_bagfile_size", 0);
  sopt.max_bagfile_duration = declare_parameter<int>("max_bagfile_duration", 0);

  // set recorder options
#ifdef USE_GET_RECORD_OPTIONS
  rosbag2_transport::RecordOptions & ropt = get_record_options();
#else
  rosbag2_transport::RecordOptions & ropt = record_options_;
#endif
#ifdef USE_ALL_TOPICS
  ropt.all_topics = declare_parameter<bool>("record_all", false);
#else
  ropt.all = declare_parameter<bool>("record_all", false);
#endif
  ropt.is_discovery_disabled = declare_parameter<bool>("disable_discovery", false);
  ropt.rmw_serialization_format = declare_parameter<std::string>("serialization_format", "cdr");
  ropt.topic_polling_interval = std::chrono::milliseconds(100);
  ropt.topics.insert(ropt.topics.end(), topics.begin(), topics.end());

  const std::string qos_profile_overrides_path =
    declare_parameter<std::string>("qos_profile_overrides_path", "");
  if (!qos_profile_overrides_path.empty()) {
    load_qos_profile_overrides_from_file(
      qos_profile_overrides_path, ropt.topic_qos_profile_overrides, get_logger(), get_name(),
      get_namespace());
  } else {
    RCLCPP_INFO(
      get_logger(),
      "qos_profile_overrides_path is empty, no explicit QoS profile overrides will be applied.");
  }

  if (ropt.is_discovery_disabled) {
#ifdef USE_STOP_DISCOVERY
    stop_discovery();
#else
    stop_discovery_ = ropt.is_discovery_disabled;
#endif
  }

  if (declare_parameter<bool>("start_recording_immediately", false)) {
    if (!bag_name_.empty()) {
      sopt.uri = bag_name_;
    } else {
      sopt.uri = bag_prefix_ + get_time_stamp();
    }
    record();
    isRecording_ = true;
  } else {
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "start_recording",
      std::bind(
        &ComposableRecorder::startRecording, this, std::placeholders::_1, std::placeholders::_2));
  }
  stop_service_ = create_service<std_srvs::srv::Trigger>(
    "stop_recording",
    std::bind(
      &ComposableRecorder::stopRecording, this, std::placeholders::_1, std::placeholders::_2));
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
#ifdef USE_GET_STORAGE_OPTIONS
    rosbag2_storage::StorageOptions & sopt = get_storage_options();
#else
    rosbag2_storage::StorageOptions & sopt = storage_options_;
#endif
    if (!bag_name_.empty()) {
      sopt.uri = bag_name_;
    } else {
      sopt.uri = bag_prefix_ + get_time_stamp();
    }
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

bool ComposableRecorder::stopRecording(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  res->success = false;
  if (!isRecording_) {
    RCLCPP_WARN(get_logger(), "not recording yet!");
    res->message = "not recording yet!";
  } else {
    RCLCPP_INFO(get_logger(), "stopping recording...");
    try {
      stop();
      isRecording_ = false;
      RCLCPP_INFO(get_logger(), "stopped recording successfully");
      res->success = true;
      res->message = "stopped recoding!";
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
