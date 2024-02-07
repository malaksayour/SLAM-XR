#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
using std::placeholders::_1;


class Ps4Controller : public rclcpp::Node
{
  public:
    Ps4Controller() : Node("ps4_controller")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/joy_teleop/cmd_vel", 10);
      subscriber_= this->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&Ps4Controller::joyCallBack, this, _1));
    }

  private:
    void joyCallBack(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
      // create a new twist message
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = msg->axes[1];
      twist_msg.angular.z = msg->axes[0];
      publisher_->publish(twist_msg);
    }
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ps4Controller>());
  rclcpp::shutdown();
  return 0;
}
