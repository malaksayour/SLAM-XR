#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/mapnames.hpp"
#include "custom_interfaces/srv/loadmap.hpp"

#include <experimental/filesystem>
#include "agents.h"


namespace fs = std::experimental::filesystem;
typedef pcl::PointXYZRGB PointL;
typedef pcl::PointCloud<PointL> PointCloudLabeled;


void getMaps(const std::shared_ptr<custom_interfaces::srv::Mapnames::Request> request,
          std::shared_ptr<custom_interfaces::srv::Mapnames::Response> response)
{

    std::string path = "/home/user/SLAM-XR/maps/octo_maps";  // Replace this with the path to your directory
    std::vector<std::string> mapfiles;
    try
    {
        for (const auto& entry : fs::directory_iterator(path)) {
            if (fs::is_regular_file(entry.path())) { 
                mapfiles.push_back(entry.path().filename().string());
            }
        }
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
    response->resp=mapfiles;

}
void loadMaps(const std::shared_ptr<custom_interfaces::srv::Loadmap::Request> request,
          std::shared_ptr<custom_interfaces::srv::Loadmap::Response> response)
{
    std::string path="/home/user/SLAM-XR/maps/octo_maps/"+request->map_name;
    octomap::OcTree octree(mapResolution);
    octree.readBinary(path);

    octomap_msgs::msg::Octomap msg;
    octomap_msgs::binaryMapToMsg(octree, msg);
    msg.header.stamp = rclcpp::Time(); 
    msg.header.frame_id = "map";
    msg.id = "OcTree";

    //PointCloudLabeled::Ptr cloud(new PointCloudLabeled);

    //pcl::io::loadPCDFile<PointL>(pcd_path, *cloud);
    //sensor_msgs::msg::PointCloud2 cloud_msg;
    //pcl::toROSMsg(*cloud, cloud_msg);
    //cloud_msg.header.frame_id = "map";
    response->octo_output=msg;

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("load_map_server");

  rclcpp::Service<custom_interfaces::srv::Mapnames>::SharedPtr service =
    node->create_service<custom_interfaces::srv::Mapnames>("map_names", &getMaps);

  rclcpp::Service<custom_interfaces::srv::Loadmap>::SharedPtr service2 =
    node->create_service<custom_interfaces::srv::Loadmap>("load_map", &loadMaps);

  RCLCPP_INFO(rclcpp::get_logger("load_service"), "Ready to load maps.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}