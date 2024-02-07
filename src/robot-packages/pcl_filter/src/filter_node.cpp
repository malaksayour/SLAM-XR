#define BOOST_BIND_NO_PLACEHOLDERS
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;


class PclFilter : public rclcpp::Node
{
  public:
    PclFilter() : Node("pcl_filter")
    {
      pclPublisher= this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_filtered", 10);
      pclSubscriber= this->create_subscription<sensor_msgs::msg::PointCloud2>("/map", 10, std::bind(&PclFilter::pclFilterCallBack, this, _1));
      leafSize=declare_parameter("filterLeafSize" ,0.1);
    }
        
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclPublisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pclSubscriber;
    double leafSize;

  private:
    void pclFilterCallBack(const sensor_msgs::msg::PointCloud2::ConstPtr cloud_msg) const
    {
      RCLCPP_INFO(rclcpp::get_logger("filter_node"),"pcl received \n");
      // Container for original & filtered data
      pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
      pcl::PCLPointCloud2 cloud_filtered;

      // Convert to PCL data type
      pcl_conversions::toPCL(*cloud_msg, *cloud);

      pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
      voxel_filter.setLeafSize(leafSize,leafSize,leafSize);



      // Perform the actual filtering
      voxel_filter.setInputCloud (cloudPtr);
      voxel_filter.filter (cloud_filtered);

      // Convert to ROS data type
      sensor_msgs::msg::PointCloud2 output;
      pcl_conversions::fromPCL(cloud_filtered, output);

      // Publish the data
      pclPublisher->publish(output);
      RCLCPP_INFO(rclcpp::get_logger("filter_node"),"pcl filtered! \n");

    }

};

int main(int argc, char ** argv)
{
  RCLCPP_INFO(rclcpp::get_logger("filter_node"),"Filter Node Started \n");

  rclcpp::init(argc, argv);
  auto pcl_filter=  std::make_shared<PclFilter>();
  rclcpp::spin(pcl_filter);
  rclcpp::shutdown();
  return 0;
}
