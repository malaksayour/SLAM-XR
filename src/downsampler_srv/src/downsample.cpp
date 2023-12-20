#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/downsample.hpp"
#include "agents.h"

#include <memory>
typedef pcl::PointXYZ PointT;

void downsample(const std::shared_ptr<custom_interfaces::srv::Downsample::Request> request,
          std::shared_ptr<custom_interfaces::srv::Downsample::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request for ID: %ld", request->id);

    //convert octomap tp pcl
    octomap::OcTree *tree; 
    tree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(request->octo_in);
    pcl::PointCloud<PointT> occupiedCells;
    pcl::PCLPointCloud2 cloudpcl2; 

    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(),
        end = tree->end_leafs(); it != end; ++it)
    {
        if (tree->isNodeOccupied(*it)) {
        occupiedCells.push_back(
            pcl::PointXYZ(it.getX(),
                it.getY(),
                it.getZ()
                )
            );
        }
    }

    //convert to pcl2
    pcl::toPCLPointCloud2(occupiedCells,cloudpcl2);
    pcl::PCLPointCloud2ConstPtr cloudPtr=pcl::make_shared<pcl::PCLPointCloud2>(cloudpcl2);
    pcl::PCLPointCloud2 cloud_filtered;

    //filter the pcl2
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
    voxel_filter.setLeafSize(filterDictionary[request->id] ,filterDictionary[request->id] ,filterDictionary[request->id] );
    voxel_filter.setInputCloud (cloudPtr);
    voxel_filter.filter (cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);
    output.header.stamp = rclcpp::Time( request->id);
    output.header.frame_id = "map";
    // Publish the data
    response->pcl_output=output;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Done: %ld", request->id);


}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("downsample_server");

  rclcpp::Service<custom_interfaces::srv::Downsample>::SharedPtr service =
    node->create_service<custom_interfaces::srv::Downsample>("downsample", &downsample);



  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to downsample.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}