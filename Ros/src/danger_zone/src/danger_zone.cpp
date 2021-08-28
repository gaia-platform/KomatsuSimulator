#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.h>

#include "../generated/gaia_danger_zone.h"

#include "rules.hpp"
#include "system.hpp"

using std::placeholders::_1;

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
  }

private:

  const std::string m_detected_topic_name = "/detections"; 
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr m_detection3d_subscription;

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
};

int main(int argc, char * argv[])
{
  DirectAccess da;
  da.init();


  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
