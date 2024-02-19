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

int agent_number=4;
std::mutex mtx_;
//std::map<int, octomap_msgs::msg::Octomap> octomapDictionary;
std::map<int, double> filterDictionary={{1,0.5},{2,0.2},{3,0.5},{4,0.5},{5,0.5},{6,0.5},{7,0.5}};
std::string offline_map_path="";
int offline_map_id=4;
//std::map<int, Eigen::Matrix4f> tfDictionary;
//std::map<int, Eigen::Matrix4f> tfRefinedDictionary;
//sensor_msgs::msg::PointCloud2 add_pcl;
//sensor_msgs::msg::PointCloud2 delete_pcl;
float mapResolution=0.05;
std::vector<float> agentWeights={1.5, 2, 3,1};


//wa2tiye
int octo_type=0;


//align params
int initTransMaxError=5;
int downsampleRate= 10;
int epsilonFactor=60;
int maxIterations=4;