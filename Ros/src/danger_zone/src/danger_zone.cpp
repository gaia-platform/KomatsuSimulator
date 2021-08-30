#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <danger_zone_msgs/msg/obstacle.hpp>
#include <danger_zone_msgs/msg/obstacle_array.hpp>

//#include "/media/mark/Data1/U20/develop/Unity/KomatsuSimulator/Ros/install/danger_zone_msgs/include/danger_zone_msgs/msg/obstacle.hpp"

#include "../generated/gaia_danger_zone.h"

#include "rules.hpp"
#include "system.hpp"

using std::placeholders::_1;

template <typename... Args> inline void unused(Args&&...) {}

class DirectAccess
{
private:

  const std::string m_config_file_name = "./gaia_conf.toml"; 
  
  //*****************************************************************************
  //*
  //*
  //*
  //*****************************************************************************

  void log_this(std::string prefix, const std::exception& e)
  {
      //auto ew = e.what();
      //UNUSED(ew);
      std::cout << "Exception: " << prefix << " : " << e.what();
  }

  //*****************************************************************************
  //*
  //*
  //*
  //*****************************************************************************

  void log_this(std::string prefix)
  {
      std::cout << prefix;
  }

  //*****************************************************************************
  //*
  //*
  //*
  //*****************************************************************************

  int init_in()
  {
    try
      {
          //Initialize Gaia
          gaia::system::initialize(m_config_file_name.c_str());

          init_storage();
      }
      catch (const std::exception& e)
      {
          log_this("DirectAccess::Init()", e);
          throw;
      }
      catch (...)
      {
          log_this("Exception in DirectAccess::Init() ...");
          throw;
      }

      return 0;
  }  

  //*****************************************************************************
  //* Insert a detected object into the db
  //* Arguments: int id
  //* Returns: gaia_id_t the id of the new row 
  //*****************************************************************************

  gaia::common::gaia_id_t insert_detected_object(int id)
  {
      gaia::danger_zone::dobject_writer ow;
      ow.id = id;
      return ow.insert_row();
  }

  //*****************************************************************************
  //*
  //*
  //*
  //*****************************************************************************

  void init_storage()
  {
      //remove_everything();

      gaia::direct_access::auto_transaction_t tx(gaia::direct_access::auto_transaction_t::no_auto_begin);

      if (gaia::danger_zone::detected_t::get_first())
      {
          //remove_everything();
          tx.commit();
          return;
      }

      tx.commit();
  }

public:

  void init(){ init_in();}

};

class SubscriberNode : public rclcpp::Node
{
public:

  //*****************************************************************************
  //*
  //*
  //*
  //*****************************************************************************

  SubscriberNode(): Node("danger_zone_ros")
  {
    m_detection3d_subscription = this->create_subscription<vision_msgs::msg::Detection3DArray>(
      m_detected_topic_name, 10, std::bind(&SubscriberNode::detection3d_callback, this, _1));

    m_obstacles_pub_ = this->create_publisher<danger_zone_msgs::msg::ObstacleArray>(
      m_obstacles_topic_name, 1);
  }

private:

  const std::string m_detected_topic_name = "/komatsu/detections"; 
  const std::string m_obstacles_topic_name = "/komatsu/obstacles"; 
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr m_detection3d_subscription;
  rclcpp::Publisher<danger_zone_msgs::msg::ObstacleArray>::SharedPtr m_obstacles_pub_;

  //*****************************************************************************
  //*
  //*
  //*
  //*****************************************************************************

  void insert_seen_object(int object_id, std::string class_id, 
    float score, float pos_x, float pos_y, float pos_z, 
    float size_x, float size_y, float size_z) const
  {
    gaia::db::begin_transaction();

    // add detected object row to DB
    auto id = gaia::danger_zone::dobject_t::insert_row(object_id, 
      class_id.c_str(), score, pos_x, pos_y, size_x, size_y);

    unused(id, pos_z, size_z);

    gaia::db::commit_transaction();
  }

  //*****************************************************************************
  //*
  //*
  //*
  //*****************************************************************************

  void detection3d_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg) const
  {
    for(vision_msgs::msg::Detection3D detection : msg->detections)
    {
      //TODO MW : To get past build
      //RCLCPP_INFO(this->get_logger(), "I saw: '%s'", detection.id.c_str());      
      RCLCPP_INFO(this->get_logger(), "I saw: '%s'", "something");

      std::string max_class = "";
      float max_score = 0.0;

      for( auto result : detection.results )
      {
        if( result.hypothesis.score > max_score )
        {
          max_score = result.hypothesis.score;
          max_class = result.hypothesis.class_id;
        }
      }
            
      if( max_class == "" )
      {
        return; // TODO : Log this?
      }

      //auto md = detection.header;
      
      insert_seen_object(0, max_class, max_score, 
        detection.bbox.center.position.x, detection.bbox.center.position.y, detection.bbox.center.position.z, 
        detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z);
    }
  }

  //*****************************************************************************
  //*
  //*
  //*
  //*****************************************************************************

  danger_zone_msgs::msg::ObstacleArray::UniquePtr build_obstacleArray_message(
    std::vector<danger_zone_msgs::msg::Obstacle> obstacles, 
    std::string frame_id, int32_t sec, uint32_t nsec)
  {
    danger_zone_msgs::msg::ObstacleArray::UniquePtr obstacle_array(new danger_zone_msgs::msg::ObstacleArray);

    obstacle_array->header = std_msgs::msg::Header();
    obstacle_array->header.frame_id = frame_id;
    obstacle_array->header.stamp.sec = sec;
    obstacle_array->header.stamp.nanosec = nsec;

    for(auto obstacle : obstacles)
      obstacle_array->obstacles.push_back(obstacle);

    return obstacle_array;
  }  
  
  //*****************************************************************************
  //*
  //*
  //*
  //*****************************************************************************

  danger_zone_msgs::msg::Obstacle::UniquePtr build_obstacle_message()
  {
    danger_zone_msgs::msg::Obstacle::UniquePtr obst(new danger_zone_msgs::msg::Obstacle);

    geometry_msgs::msg::Point::UniquePtr point(new geometry_msgs::msg::Point);
    point->x = 1.0;
    point->y = 1.0;
    point->z = 1.0;

    geometry_msgs::msg::Vector3::UniquePtr size(new geometry_msgs::msg::Vector3);
    size->x = 1.0;
    size->y = 1.0;
    size->z = 1.0;

    geometry_msgs::msg::Quaternion::UniquePtr orient(new geometry_msgs::msg::Quaternion);
    orient->x = 0.0;
    orient->y = 0.0;
    orient->z = 0.0;
    orient->w = 1.0;

    geometry_msgs::msg::Pose::UniquePtr pose(new geometry_msgs::msg::Pose);
    pose->position = *point;
    pose->orientation = *orient;

    vision_msgs::msg::BoundingBox3D::UniquePtr bbox(new vision_msgs::msg::BoundingBox3D);
    bbox->center = *pose;
    bbox->size = *size;

    obst->type = "theType";
    obst->roi = danger_zone_msgs::msg::Obstacle::RED;
    obst->direction = danger_zone_msgs::msg::Obstacle::FRONT_LEFT;
    obst->bounds = *bbox;

    return obst;
  }

  //*****************************************************************************
  //*
  //*
  //*
  //*****************************************************************************

  danger_zone_msgs::msg::Obstacle::UniquePtr build_test_obstacle_message()
  {
    danger_zone_msgs::msg::Obstacle::UniquePtr obst(new danger_zone_msgs::msg::Obstacle);

    geometry_msgs::msg::Point::UniquePtr point(new geometry_msgs::msg::Point);
    point->x = 1.0;
    point->y = 1.0;
    point->z = 1.0;

    geometry_msgs::msg::Vector3::UniquePtr size(new geometry_msgs::msg::Vector3);
    size->x = 1.0;
    size->y = 1.0;
    size->z = 1.0;

    geometry_msgs::msg::Quaternion::UniquePtr orient(new geometry_msgs::msg::Quaternion);
    orient->x = 0.0;
    orient->y = 0.0;
    orient->z = 0.0;
    orient->w = 1.0;

    geometry_msgs::msg::Pose::UniquePtr pose(new geometry_msgs::msg::Pose);
    pose->position = *point;
    pose->orientation = *orient;

    vision_msgs::msg::BoundingBox3D::UniquePtr bbox(new vision_msgs::msg::BoundingBox3D);
    bbox->center = *pose;
    bbox->size = *size;

    obst->type = "theType";
    obst->roi = danger_zone_msgs::msg::Obstacle::RED;
    obst->direction = danger_zone_msgs::msg::Obstacle::FRONT_LEFT;
    obst->bounds = *bbox;

    return obst;
  }

  //*****************************************************************************
  //*
  //*
  //*
  //*****************************************************************************

  void send_test_obstacleArray_message()
  {
    std::vector<danger_zone_msgs::msg::Obstacle> obstacles;
    obstacles.push_back(*(build_test_obstacle_message()));

    auto obstacleArray = build_obstacleArray_message(obstacles,"theFrame", 1, 1);

    m_obstacles_pub_->publish(std::move(obstacleArray));
  }
};

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int main(int argc, char * argv[])
{
  DirectAccess da;
  da.init();


  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
