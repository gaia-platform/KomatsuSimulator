//*****************************************************************************
//*
//* Dependencies: Requires Gaia preview (without libc++) or later, Gaia March
//* 2021 will not work
//*
//*****************************************************************************

#include <functional>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <danger_zone_msgs/msg/obstacle.hpp>
#include <danger_zone_msgs/msg/obstacle_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "gaia/rules/rules.hpp"
#include "gaia/system.hpp"

#include "gaia_danger_zone.h"
#include "idanger_zone.hpp"
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
        // TODO : yes, I know, make this modern
        danger_zone_p = static_cast<danger_zone_t*>(this);

        m_detection3d_subscription = this->create_subscription<vision_msgs::msg::Detection3DArray>(
            m_detected_topic_name, 10, std::bind(&subscriber_node_t::detection3d_callback, this, _1));

        m_obstacles_pub_ = this->create_publisher<danger_zone_msgs::msg::ObstacleArray>(
            m_obstacles_topic_name, 1);
    }

    //*** iDangerZone interface ***

    void cb_send_obstacle_array_message_old(
        std::string type_name, uint roi, uint direction,
        double posx, double posy, double posz,
        double sizex, double sizey, double sizez,
        double orientx, double orienty, double orientz, double orientw,
        std::string frame_id, int32_t sec, uint32_t nsec) const
    {
        std::vector<danger_zone_msgs::msg::Obstacle> obstacles;
        obstacles.push_back(*(build_obstacle_message(
            type_name, roi, direction, posx, posy, posz,
            sizex, sizey, sizez, orientx, orienty, orientz, orientw)));

        // TODO : get this out of the thread
        m_obstacles_pub_->publish(build_obstacleArray_message(obstacles, frame_id, sec, nsec));
    }

    void cb_send_obstacle_array_message(
        std::string type_name, uint roi, uint direction,
        double posx, double posy, double posz,
        double sizex, double sizey, double sizez,
        double orientx, double orienty, double orientz, double orientw,
        std::string frame_id, int32_t sec, uint32_t nsec) override
    {
        // std::vector<danger_zone_msgs::msg::Obstacle> obstacles;
        m_obstacles.push_back(*(build_obstacle_message(
            type_name, roi, direction, posx, posy, posz,
            sizex, sizey, sizez, orientx, orienty, orientz, orientw)));

        unused(frame_id, sec, nsec);

        // TODO : get this out of the thread
        // m_obstacles_pub_->publish(build_obstacleArray_message(obstacles, frame_id,  sec, nsec));
    }

private:
    bool m_simple_echo = false;
    const std::string m_detected_topic_name = "/komatsu/detections";
    const std::string m_obstacles_topic_name = "/komatsu/obstacles";
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr m_detection3d_subscription;
    rclcpp::Publisher<danger_zone_msgs::msg::ObstacleArray>::SharedPtr m_obstacles_pub_;

    // Mutex to lock down detection3d_callback.
    std::mutex m_mtx;

    // The list of Gaia processed obstacles, cleared a dnrefilled with each detection3d_callback.
    std::vector<danger_zone_msgs::msg::Obstacle> m_obstacles;

    void insert_seen_object(int object_id, std::string class_id, float score, const char* frame_id, int32_t range_id, int32_t direction_id, int32_t seconds, int32_t nseconds, float pos_x, float pos_y, float pos_z, float size_x, float size_y, float size_z, float orient_x, float orient_y, float orient_z, float orient_w) const
    {
        gaia::db::begin_transaction();

        // add detected object row to DB
        auto id = gaia::danger_zone::dobject_t::insert_row(
            object_id, class_id.c_str(), score, frame_id, range_id, direction_id, seconds, nseconds,
            pos_x, pos_y, pos_z, size_x, size_y, size_z,
            orient_x, orient_y, orient_z, orient_w);

        unused(id);

        gaia::db::commit_transaction();
    }

    void detection3d_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
        // TODO : just a test, remove this
        // send_test_obstacleArray_message();

        // std::unique_lock<std::mutex> lck (mtx,std::defer_lock);

        // prevent two threads from operating on this method simultaneously
        std::lock_guard<std::mutex> lck(m_mtx);

        // upon entry, clear all obstacles from list
        m_obstacles.clear();

        std::vector<danger_zone_msgs::msg::Obstacle> obstacles;

        for (const vision_msgs::msg::Detection3D& detection : msg->detections)
        {
            // TODO MW : To get past build
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
                return; // TODO : Log this?
            }

            insert_seen_object(0, max_hyp.hypothesis.class_id, max_hyp.hypothesis.score, detection.header.frame_id.c_str(), 0, 0, detection.header.stamp.sec, detection.header.stamp.nanosec, detection.bbox.center.position.x, detection.bbox.center.position.y, detection.bbox.center.position.z, detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z, max_hyp.pose.pose.orientation.x, max_hyp.pose.pose.orientation.y, max_hyp.pose.pose.orientation.z, max_hyp.pose.pose.orientation.w);

            if (m_simple_echo)
                obstacles.push_back(*(build_obstacle_message(
                    max_hyp.hypothesis.class_id,
                    (int)zones_t::get_singleton()->get_range_zone_id(detection.bbox.center.position.x, detection.bbox.center.position.z),
                    (int)zones_t::get_singleton()->get_direction_zone_id(detection.bbox.center.position.z, detection.bbox.center.position.x),
                    detection.bbox.center.position.x, detection.bbox.center.position.y, detection.bbox.center.position.z,
                    detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z,
                    max_hyp.pose.pose.orientation.x, max_hyp.pose.pose.orientation.y, max_hyp.pose.pose.orientation.z, max_hyp.pose.pose.orientation.w)));
        }

        if (m_simple_echo)
            m_obstacles_pub_->publish(build_obstacleArray_message(
                obstacles, msg->header.frame_id, msg->header.stamp.sec, msg->header.stamp.nanosec));
        else if (!m_obstacles.empty())
            m_obstacles_pub_->publish(build_obstacleArray_message(
                m_obstacles, msg->header.frame_id, msg->header.stamp.sec, msg->header.stamp.nanosec));
    }

    danger_zone_msgs::msg::ObstacleArray::UniquePtr build_obstacleArray_message(
        std::vector<danger_zone_msgs::msg::Obstacle> obstacles,
        std::string frame_id, int32_t sec, uint32_t nsec) const
    {
        danger_zone_msgs::msg::ObstacleArray::UniquePtr obstacle_array(new danger_zone_msgs::msg::ObstacleArray);

        obstacle_array->header = std_msgs::msg::Header();
        obstacle_array->header.frame_id = frame_id;
        obstacle_array->header.stamp.sec = sec;
        obstacle_array->header.stamp.nanosec = nsec;

        for (const auto& obstacle : obstacles)
            obstacle_array->obstacles.push_back(obstacle);

        return obstacle_array;
    }

    danger_zone_msgs::msg::Obstacle::UniquePtr build_obstacle_message(
        std::string type_name, uint roi, uint direction,
        double posx, double posy, double posz,
        double sizex, double sizey, double sizez,
        double orientx, double orienty, double orientz, double orientw) const
    {
        danger_zone_msgs::msg::Obstacle::UniquePtr obst(new danger_zone_msgs::msg::Obstacle);

        geometry_msgs::msg::Point::UniquePtr point(new geometry_msgs::msg::Point);
        point->x = posx;
        point->y = posy;
        point->z = posz;

        geometry_msgs::msg::Vector3::UniquePtr size(new geometry_msgs::msg::Vector3);
        size->x = sizex;
        size->y = sizey;
        size->z = sizez;

        geometry_msgs::msg::Quaternion::UniquePtr orient(new geometry_msgs::msg::Quaternion);
        orient->x = orientx;
        orient->y = orienty;
        orient->z = orientz;
        orient->w = orientw;

        geometry_msgs::msg::Pose::UniquePtr pose(new geometry_msgs::msg::Pose);
        pose->position = *point;
        pose->orientation = *orient;

        vision_msgs::msg::BoundingBox3D::UniquePtr bbox(new vision_msgs::msg::BoundingBox3D);
        bbox->center = *pose;
        bbox->size = *size;

        obst->type = type_name;
        obst->roi = roi;
        obst->direction = direction;
        obst->bounds = *bbox;

        return obst;
    }

    danger_zone_msgs::msg::ObstacleArray::UniquePtr build_obstacleArray_message(
        std::string type_name, uint roi, uint direction,
        double posx, double posy, double posz,
        double sizex, double sizey, double sizez,
        double orientx, double orienty, double orientz, double orientw,
        std::string frame_id, int32_t sec, uint32_t nsec) const
    {
        std::vector<danger_zone_msgs::msg::Obstacle> obstacles;
        obstacles.push_back(*(build_obstacle_message(
            type_name, roi, direction, posx, posy, posz,
            sizex, sizey, sizez, orientx, orienty, orientz, orientw)));

        return build_obstacleArray_message(obstacles, frame_id, sec, nsec);
    }

    void send_test_obstacleArray_message() const
    {
        auto obstacle_array = build_obstacleArray_message(
            "theType", 1, 1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, "theFrame", 1, 1);

        m_obstacles_pub_->publish(std::move(obstacle_array));
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
