#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

template <typename... Args> inline void unused(Args&&...) {}

class SubscriberNode : public rclcpp::Node
{
public:

  //*****************************************************************************
  //*
  //*
  //*
  //*****************************************************************************

  SubscriberNode(): Node("retro_log")
  {
    m_command_subscription = this->create_subscription<retro_log::msg::Command>(
      m_command_topic_name, 10, std::bind(&SubscriberNode::command_callback, this, _1));
  }

private:

  const std::string m_command_topic_name = "/retrolog/command"; 
  rclcpp::Subscription<retro_log::msg::Command>::SharedPtr m_command_subscription;

  //*****************************************************************************
  //*
  //*
  //*
  //*****************************************************************************

  void command_callback(const retro_log::msg::Command msg) const
  {
  }
};

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
