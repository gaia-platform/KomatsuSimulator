/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

//*****************************************************************************
//*
//* Dependencies: Requires Gaia preview (without libc++) or later, Gaia March
//* 2021 will not work
//*
//*****************************************************************************

#include "danger_zone.hpp"

#include <functional>
#include <memory>
#include <mutex>

#include <danger_zone_msgs/msg/obstacle.hpp>
#include <danger_zone_msgs/msg/obstacle_array.hpp>
#include "../inc/snapshot_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "gaia/logger.hpp"
#include "gaia/rules/rules.hpp"
#include "gaia/system.hpp"

#include "gaia_danger_zone.h"
#include "zones.hpp"

using std::placeholders::_1;

template <typename... T_args>
inline void unused(T_args&&...)
{
}

class subscriber_node_t : public rclcpp::Node, danger_zone_t
{
public:
    subscriber_node_t()
        : Node("danger_zone_ros")
    {
        // TODO: make this modern.
        danger_zone_ptr = static_cast<danger_zone_t*>(this);

        m_detection3d_subscription = this->create_subscription<vision_msgs::msg::Detection3DArray>(
            m_detected_topic_name, 10, std::bind(&subscriber_node_t::detection3d_callback, this, _1));

        m_obstacles_pub = this->create_publisher<danger_zone_msgs::msg::ObstacleArray>(
            m_obstacles_topic_name, 1);
    }

    // danger_zone interface.

    void cb_send_obstacle_array_message_old(
        std::string type_name, uint roi, uint direction,
        double pos_x, double pos_y, double pos_z,
        double size_x, double size_y, double size_z,
        double orient_x, double orient_y, double orient_z, double orient_w,
        std::string frame_id, int32_t sec, uint32_t nsec) const
    {
        std::vector<danger_zone_msgs::msg::Obstacle> obstacles;
        obstacles.push_back(*(build_obstacle_message(
            type_name, roi, direction, pos_x, pos_y, pos_z,
            size_x, size_y, size_z, orient_x, orient_y, orient_z, orient_w)));

        // TODO: get this out of the thread.
        m_obstacles_pub->publish(build_obstacle_array_message(obstacles, frame_id, sec, nsec));
    }

    void cb_send_obstacle_array_message(
        std::string type_name, uint roi, uint direction,
        double pos_x, double pos_y, double pos_z,
        double size_x, double size_y, double size_z,
        double orient_x, double orient_y, double orient_z, double orient_w,
        std::string frame_id, int32_t sec, uint32_t nsec) override
    {
        // std::vector<danger_zone_msgs::msg::Obstacle> obstacles;
        m_obstacles.push_back(*(build_obstacle_message(
            type_name, roi, direction, pos_x, pos_y, pos_z,
            size_x, size_y, size_z, orient_x, orient_y, orient_z, orient_w)));

        unused(frame_id, sec, nsec);

        // TODO: get this out of the thread.
        // m_obstacles_pub_->publish(build_obstacle_array_message(obstacles, frame_id,  sec, nsec));
    }

    void cb_trigger_log(
        int start_sec, uint32_t start_nsec, int end_sec, uint32_t end_nsec,
        std::string file_name, std::vector<std::string> topics) override
    {
        // TODO: we need to debounce this, figure out how to properly handle overlaps.

        auto sc = new SnapshotClient();
        sc->connect(this, m_snapshot_service_name);
        sc->send_request(start_sec, start_nsec, end_sec, end_nsec, file_name, topics);
    }

    void cb_trigger_log(
        int seconds_past, int seconds_forward,
        std::string file_name, std::vector<std::string> topics) override
    {
        auto base_time = get_clock()->now();
        auto base_sec = base_time.seconds();
        auto base_nsec = base_time.nanoseconds();

        cb_trigger_log(base_sec - seconds_past, base_nsec, base_sec + seconds_forward, base_nsec, file_name, topics);
    }

private:
    bool m_simple_echo = false;

    const std::string m_detected_topic_name = "/komatsu/detections";
    const std::string m_obstacles_topic_name = "/komatsu/obstacles";
    // Name found in snapshotter.cpp.
    const std::string m_snapshot_service_name = "trigger_snapshot";

    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr m_detection3d_subscription;
    rclcpp::Publisher<danger_zone_msgs::msg::ObstacleArray>::SharedPtr m_obstacles_pub;

    // Mutex to lock down detection3d_callback.
    std::mutex m_mtx;

    // The list of Gaia processed obstacles.
    std::vector<danger_zone_msgs::msg::Obstacle> m_obstacles;

private:
    void detection3d_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
        // Prevent two threads from operating on this method simultaneously.
        std::lock_guard<std::mutex> lck(m_mtx);

        // Upon entry, clear all obstacles from list.
        m_obstacles.clear();

        std::vector<danger_zone_msgs::msg::Obstacle> obstacles;

        gaia::db::begin_transaction();

        auto db_detection_id = gaia::danger_zone::detection_t::insert_row(false);
        auto db_detection = gaia::danger_zone::detection_t::get(db_detection_id);

        for (const vision_msgs::msg::Detection3D& detection : msg->detections)
        {
            // TODO: (Mark West) Commented, to get past build.
            // RCLCPP_INFO(this->get_logger(), "I saw: '%s'", detection.id.c_str());
            // RCLCPP_INFO(this->get_logger(), "I saw: '%s'", "something");

            vision_msgs::msg::ObjectHypothesisWithPose max_hyp;

            max_hyp.hypothesis.class_id = "";
            max_hyp.hypothesis.score = 0.0;

            for (const auto& result : detection.results)
            {
                if (result.hypothesis.score > max_hyp.hypothesis.score)
                {
                    max_hyp = result;
                }
            }

            if (max_hyp.hypothesis.class_id == "")
            {
                // TODO: Log this?
                continue;
            }

            // Note: detection.id.c_str() is non-unique ATM, it does not seem an ID either.
            auto db_detected_object_id = gaia::danger_zone::d_object_t::insert_row(
                detection.id.c_str(), max_hyp.hypothesis.class_id.c_str(), max_hyp.hypothesis.score,
                detection.header.frame_id.c_str(), 0, 0,
                detection.header.stamp.sec, detection.header.stamp.nanosec,
                detection.bbox.center.position.x, detection.bbox.center.position.y,
                detection.bbox.center.position.z,
                detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z,
                max_hyp.pose.pose.orientation.x, max_hyp.pose.pose.orientation.y,
                max_hyp.pose.pose.orientation.z, max_hyp.pose.pose.orientation.w);

            auto db_detected_object = gaia::danger_zone::d_object_t::get(db_detected_object_id);

            db_detection.d_objects().connect(db_detected_object);

            if (m_simple_echo)
            {
                obstacles.push_back(*(build_obstacle_message(
                    max_hyp.hypothesis.class_id,
                    static_cast<int>(zones_t::get_singleton()->get_range_zone_id(
                        detection.bbox.center.position.x, detection.bbox.center.position.z)),
                    static_cast<int>(zones_t::get_singleton()->get_direction_zone_id(
                        detection.bbox.center.position.z, detection.bbox.center.position.x)),
                    detection.bbox.center.position.x, detection.bbox.center.position.y,
                    detection.bbox.center.position.z,
                    detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z,
                    max_hyp.pose.pose.orientation.x, max_hyp.pose.pose.orientation.y,
                    max_hyp.pose.pose.orientation.z, max_hyp.pose.pose.orientation.w)));
            }
        }

        gaia::db::commit_transaction();

        if (m_simple_echo)
        {
            m_obstacles_pub->publish(build_obstacle_array_message(
                obstacles, msg->header.frame_id, msg->header.stamp.sec, msg->header.stamp.nanosec));
        }
        else if (!m_obstacles.empty())
        {
            m_obstacles_pub->publish(build_obstacle_array_message(
                m_obstacles, msg->header.frame_id, msg->header.stamp.sec, msg->header.stamp.nanosec));
        }
    }

    danger_zone_msgs::msg::ObstacleArray::UniquePtr build_obstacle_array_message(
        std::vector<danger_zone_msgs::msg::Obstacle> obstacles,
        std::string frame_id, int32_t sec, uint32_t nsec) const
    {
        danger_zone_msgs::msg::ObstacleArray::UniquePtr obstacle_array(new danger_zone_msgs::msg::ObstacleArray);

        obstacle_array->header = std_msgs::msg::Header();
        obstacle_array->header.frame_id = frame_id;
        obstacle_array->header.stamp.sec = sec;
        obstacle_array->header.stamp.nanosec = nsec;

        for (const auto& obstacle : obstacles)
        {
            obstacle_array->obstacles.push_back(obstacle);
        }

        return obstacle_array;
    }

    danger_zone_msgs::msg::Obstacle::UniquePtr build_obstacle_message(
        std::string type_name, uint roi, uint direction,
        double pos_x, double pos_y, double pos_z,
        double size_x, double size_y, double size_z,
        double orient_x, double orient_y, double orient_z, double orient_w) const
    {
        danger_zone_msgs::msg::Obstacle::UniquePtr obstacle(new danger_zone_msgs::msg::Obstacle);

        geometry_msgs::msg::Point::UniquePtr point(new geometry_msgs::msg::Point);
        point->x = pos_x;
        point->y = pos_y;
        point->z = pos_z;

        geometry_msgs::msg::Vector3::UniquePtr size(new geometry_msgs::msg::Vector3);
        size->x = size_x;
        size->y = size_y;
        size->z = size_z;

        geometry_msgs::msg::Quaternion::UniquePtr orient(new geometry_msgs::msg::Quaternion);
        orient->x = orient_x;
        orient->y = orient_y;
        orient->z = orient_z;
        orient->w = orient_w;

        geometry_msgs::msg::Pose::UniquePtr pose(new geometry_msgs::msg::Pose);
        pose->position = *point;
        pose->orientation = *orient;

        vision_msgs::msg::BoundingBox3D::UniquePtr bbox(new vision_msgs::msg::BoundingBox3D);
        bbox->center = *pose;
        bbox->size = *size;

        obstacle->type = type_name;
        obstacle->roi = roi;
        obstacle->direction = direction;
        obstacle->bounds = *bbox;

        return obstacle;
    }

    danger_zone_msgs::msg::ObstacleArray::UniquePtr build_obstacle_array_message(
        std::string type_name, uint roi, uint direction,
        double pos_x, double pos_y, double pos_z,
        double size_x, double size_y, double size_z,
        double orient_x, double orient_y, double orient_z, double orient_w,
        std::string frame_id, int32_t sec, uint32_t nsec) const
    {
        std::vector<danger_zone_msgs::msg::Obstacle> obstacles;
        obstacles.push_back(*(build_obstacle_message(
            type_name, roi, direction, pos_x, pos_y, pos_z,
            size_x, size_y, size_z, orient_x, orient_y, orient_z, orient_w)));

        return build_obstacle_array_message(obstacles, frame_id, sec, nsec);
    }

    void send_test_obstacle_array_message() const
    {
        auto obstacle_array = build_obstacle_array_message(
            "theType", 1, 1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, "theFrame", 1, 1);

        m_obstacles_pub->publish(std::move(obstacle_array));
    }
};

int main(int argc, char* argv[])
{
    gaia::system::initialize();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<subscriber_node_t>());
    rclcpp::shutdown();
    return 0;
}
