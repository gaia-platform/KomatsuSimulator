// Copyright 2021 Gaia Platform
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
// limitations under the License.#include <iostream>

#pragma once

//#include "rclcpp/rclcpp.hpp"
//#include "rclcpp_action/rclcpp_action.hpp"
//#include "rclcpp_service/rclcpp_service.hpp"
//#include "nav2_msgs/action/navigate_to_pose.hpp"
//#include "irule_ops.hpp"

#include "rclcpp/rclcpp.hpp"
#include <rosbag2_snapshot_msgs/srv/trigger_snapshot.hpp>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class SnapshotClient 
{

private:

  using TriggerSnapshot = rosbag2_snapshot_msgs::srv::TriggerSnapshot;
  std::shared_ptr<rclcpp::Client<SnapshotClient::TriggerSnapshot>> m_snapshot_client;
  std::string m_service_name;
  std::chrono::seconds m_service_wait_time = 1s;

  rclcpp::Node* m_caller_p;
  bool m_connected = false;
  bool m_verbose = true;

public:

  SnapshotClient();

  void connect(rclcpp::Node* caller, std::string service_name);

  bool send_request( int start_sec, uint32_t start_nsec, 
    int end_sec, uint32_t end_nsec, std::string file_name, 
    std::vector<std::string>topics);
};

