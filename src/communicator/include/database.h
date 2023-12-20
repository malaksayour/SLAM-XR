#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeStamped.h>
#include <octomap_msgs/msg/octomap.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/msg/twist.hpp>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

extern std::mutex mtx_;
extern std::map<int, octomap_msgs::msg::Octomap> octomapDictionary;
extern std::map<int, Eigen::Matrix4f> tfDictionary;
// extern std::map<int, Eigen::Matrix4f> tfRefinedDictionary;
extern sensor_msgs::msg::PointCloud2 add_pcl;
extern sensor_msgs::msg::PointCloud2 delete_pcl;

