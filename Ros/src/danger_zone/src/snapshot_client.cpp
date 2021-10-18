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

#include "snapshot_client.hpp"

template <typename... Args>
inline void unused(Args&&...)
{
}

SnapshotClient::SnapshotClient()
{
}

/**
 * Connect to the ROS service
 *
 * @param[in] rclcpp::Node* caller
 * @param[in] std::string service_name
 * @return void
 * @throws
 * @exceptsafe yes
 */
void SnapshotClient::connect(rclcpp::Node* caller, std::string service_name)
{
    if (m_connected)
    {
        return;
    }

    m_caller_ptr = caller;
    m_service_name = service_name;

    m_snapshot_client = m_caller_ptr->create_client<TriggerSnapshot>(service_name);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service client created: %s", m_service_name.c_str());

    m_connected = true;
}

/**
 * Tell the ROS service to take a snapshot
 *
 * @param[in] int start_sec
 * @param[in] uint32_t start_nsec
 * @param[in] int end_sec
 * @param[in] uint32_t end_nsec
 * @param[in] std::string file_name
 * @param[in] std::vector<std::string>topics
 * @return bool : success = true, failure = false;
 * @throws
 * @exceptsafe yes
 */
bool SnapshotClient::send_request(int start_sec, uint32_t start_nsec, int end_sec, uint32_t end_nsec, std::string file_name, std::vector<std::string> topics)
{
    if (!m_snapshot_client->wait_for_service(m_service_wait_time))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, exiting");
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service connected: %s", m_service_name.c_str());

    auto req = std::make_shared<TriggerSnapshot::Request>();

    builtin_interfaces::msg::Time start_time;
    builtin_interfaces::msg::Time end_time;

    start_time.set__sec(start_sec);
    start_time.set__nanosec(start_nsec);

    end_time.set__sec(end_sec);
    end_time.set__nanosec(end_nsec);

    req->set__start_time(start_time);
    req->set__stop_time(end_time);
    req->set__filename(file_name);
    req->set__topics(topics);

    auto result = m_snapshot_client->async_send_request(req);

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Service request sent: %s", m_service_name.c_str());

    /* TODO * enable?

    if (rclcpp::spin_until_future_complete(m_snapshot_client, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: %s : %s",
        result.get()->success?"success":"failue", result.get()->message.c_str());
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }*/

    return true;
}
