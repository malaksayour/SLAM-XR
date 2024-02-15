#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/savemap.hpp"
#include "agents.h"

typedef pcl::PointXYZRGB PointL;
typedef pcl::PointCloud<PointL> PointCloudLabeled;

void saveMap(const std::shared_ptr<custom_interfaces::srv::Savemap::Request> request,
          std::shared_ptr<custom_interfaces::srv::Savemap::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Saving the map");
    PointCloudLabeled pcl_cloud;
    pcl::fromROSMsg(request->final_pcl, pcl_cloud);

    // Save the PCL point cloud to a PCD file
    pcl::PCDWriter writer;
    std::string pcl_file_name= "/home/user/SLAM-XR/maps/pcl_maps/"+request->file_name +".pcd";

    writer.write(pcl_file_name, pcl_cloud);

    std::string octo_file_name= "/home/user/SLAM-XR/maps/octo_maps/"+request->file_name+".bt";
    octomap::OcTree *tree=(octomap::OcTree*)octomap_msgs::binaryMsgToMap(request->final_octomap);

    std::ofstream file(octo_file_name, std::ios_base::out | std::ios_base::binary);
    tree->writeBinaryConst(file);
    file.close();

    response->resp=true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map saved");
    delete tree;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("save_map_server");

  rclcpp::Service<custom_interfaces::srv::Savemap>::SharedPtr service =
    node->create_service<custom_interfaces::srv::Savemap>("save_map", &saveMap);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to save_map.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}