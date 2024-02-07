#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap/octomap.h>


using std::placeholders::_1;


class Repeater : public rclcpp::Node
{
  public:
    Repeater() : Node("repeater")
    {
      publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>("/repeater_binary_robot", 10);
      subscriber_= this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_binary_robot", 10, std::bind(&Repeater::repeaterCallBack, this, _1));
    }
    bool repeat=false;
    octomap_msgs::msg::Octomap myMap; 
        
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr publisher_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscriber_;

  private:
    void repeaterCallBack(const octomap_msgs::msg::Octomap::SharedPtr message)
    {
      RCLCPP_INFO(rclcpp::get_logger("repeater_node"),"Robot Octomap Message Received \n");
      repeat=true;
      myMap=*message;
    }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("repeater_node"),"Repeater Node started \n");

  auto repeater=  std::make_shared<Repeater>();
  rclcpp::Rate rate(5);
    while(rclcpp::ok()){
    if(repeater->repeat) {

      repeater->publisher_->publish(repeater->myMap);
    }
    rclcpp::spin_some(repeater);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
