#include <cstdio>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZRGB PointL;
typedef pcl::PointCloud<PointL> PointCloudLabeled;


//general functions. to be moved to another file.

bool findIndex(int val, std::vector<int> vec){
  /**
   * @brief checks if the input vector contains the value val.
   * @param int val
   * @return bool value
  */
  for (int element : vec) {
    if (element == val) return true;
  }return false;
}

Eigen::Matrix4f poseToTransformationMatrix(const typename geometry_msgs::msg::Twist msg)
{/**
  * @brief transforms a twist message to a 4*4 matrix.
  @param geometry_msgs::msg::Twist 
  @return Eigen::Matrix4f
*/
  double cos_roll = cos(msg.angular.x);
  double sin_roll = sin(msg.angular.x);
  double cos_pitch = cos(msg.angular.y);
  double sin_pitch = sin(msg.angular.y);
  double cos_yaw = cos(msg.angular.z);
  double sin_yaw = sin(msg.angular.z);

  Eigen::Matrix4f transformation_matrix;
  transformation_matrix << cos_yaw * cos_pitch, -sin_yaw * cos_roll + cos_yaw * sin_pitch * sin_roll, sin_yaw * sin_roll + cos_yaw * sin_pitch * cos_roll, (double)msg.linear.x,
                          sin_yaw * cos_pitch, cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll, -cos_yaw * sin_roll + sin_yaw * sin_pitch * cos_roll, (double)msg.linear.y,
                          -sin_pitch, cos_pitch * sin_roll, cos_pitch * cos_roll, (double)msg.linear.z,
                          0, 0, 0, 1;

  return transformation_matrix;
}

octomap::OcTree* applyTf(octomap::OcTree* tree, Eigen::Matrix4f tf){
  /**
   * @brief applies a transfomration to an octomap.
   * @param octree to transform pointer, tf to be applied.
  */
  octomap::OcTree *transformedOctree=new octomap::OcTree(mapResolution);
  //transform the octomap
  for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(),end = tree->end_leafs(); it != end; ++it)
    {octomap::point3d originalPos = it.getCoordinate();
      //Apply the transformation to the position
      Eigen::Vector4f originalPosition(originalPos.x(), originalPos.y(), originalPos.z(), 1.0);
      Eigen::Vector4f transformedPosition = tf * originalPosition;

      // Retrieve the log-odds value from the original node
      double logOddsValue = it->getLogOdds();

      // Insert the transformed point into the new octree with the old log-odds value
      transformedOctree->setNodeValue(transformedPosition.x(), transformedPosition.y(), transformedPosition.z(), logOddsValue);
  }
  return transformedOctree;
}

sensor_msgs::msg::PointCloud2 generate_semantic_pcl(octomap::OcTree* tree, PointCloudLabeled* labels)
{
  /** @brief Genrates a labeled poincloud from an octree structure.
   * @param octomap::OcTree octree containing the point values.
   * @param octomap::OcTree octree containing the label values.
   * @return sensor_msgs::msg::PointCloud2 with 4 fileds being x,y,z and label. 
  */

  sensor_msgs::msg::PointCloud2 pcl_msg;
  // Define PointFields
  sensor_msgs::msg::PointField field1, field2, field3, field4, field5, field6;


  // Fill in the details for each PointField
  field1.name = "x";
  field1.offset = 0;
  field1.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field1.count = 1;

  field2.name = "y";
  field2.offset = 4;  // Offset of 4 bytes (since "x" is FLOAT32)
  field2.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field2.count = 1;

  field3.name = "z";
  field3.offset = 8;  // Offset of 8 bytes (since "x" and "y" are FLOAT32)
  field3.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field3.count = 1;

  field4.name = "r";
  field4.offset = 12;
  field4.datatype = sensor_msgs::msg::PointField::UINT8;
  field4.count = 1;

  field5.name = "g";
  field5.offset = 13;
  field5.datatype = sensor_msgs::msg::PointField::UINT8;
  field5.count = 1;

  field6.name = "b";
  field6.offset = 14;
  field6.datatype = sensor_msgs::msg::PointField::UINT8;
  field6.count = 1;

  // Add the fields to the PointCloud2 message
  pcl_msg.fields.push_back(field1);
  pcl_msg.fields.push_back(field2);
  pcl_msg.fields.push_back(field3);
  pcl_msg.fields.push_back(field4);
  pcl_msg.fields.push_back(field5);
  pcl_msg.fields.push_back(field6);

  // Set other attributes of the PointCloud2 message
  pcl_msg.header.stamp = rclcpp::Clock().now();
  pcl_msg.header.frame_id = "map";  // Set the frame ID
  pcl_msg.is_bigendian = false;
  pcl_msg.point_step = 15;  // The size of each point in bytes (4 fields of FLOAT32)
  pcl_msg.row_step = 15;  // Total size of the point cloud in bytes
  pcl_msg.is_dense = true;
  pcl_msg.height = 1;  // 1 indicates that we're using an unorganized point cloud


// Allocate memory for the data array
pcl_msg.data.resize(pcl_msg.row_step);

tree->expand();
PointCloudLabeled::Ptr labeledPcl(new PointCloudLabeled);

for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it){

  PointL point;
  point.x=it.getX(); point.y=it.getY();point.z=it.getZ();point.r=it->getLogOdds(); 
  point.g=0; point.b=0;
  if (it->getLogOdds()>1) labeledPcl->push_back(point);
}

  //uint8_t proba=100;
  //check if the node is present in the labeled pcl
  for (const auto& pclPoint : labels->points) {

      // Convert pcl::PointXYZ to octomap::point3d
      octomap::point3d query(pclPoint.x, pclPoint.y, pclPoint.z);

      // Check if the point is within the octree
      octomap::OcTreeNode* result = tree->search(query);

      if (result != nullptr) {
        //octomap::OcTreeKey key = result->getKey();
        //octomap::point3d pcl_point=tree->keyToCoord(key);

        PointL point;
        point.x=pclPoint.x; point.y=pclPoint.y;point.z=pclPoint.z;point.r=result->getLogOdds(); 
        point.g=pclPoint.g; point.b=pclPoint.b;
        if (result->getLogOdds()>1) labeledPcl->push_back(point);
        }
      // delete result;
  }

pcl::toROSMsg(*labeledPcl, pcl_msg);
// pcl.header.stamp = ros::Time::now();
pcl_msg.header.frame_id = "map";
//delete occupiedCells
return pcl_msg;
}

 
sensor_msgs::msg::PointCloud2 labeledToPcl2(PointCloudLabeled labels)
{
  /** @brief Genrates a labeled poincloud from an octree structure.
   * @param octomap::OcTree octree containing the point values.
   * @param octomap::OcTree octree containing the label values.
   * @return sensor_msgs::msg::PointCloud2 with 4 fileds being x,y,z and label. 
  */

  sensor_msgs::msg::PointCloud2 pcl_msg;
  // Define PointFields
  sensor_msgs::msg::PointField field1, field2, field3, field4, field5, field6;

  // Fill in the details for each PointField
  field1.name = "x";
  field1.offset = 0;
  field1.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field1.count = 1;

  field2.name = "y";
  field2.offset = 4;  // Offset of 4 bytes (since "x" is FLOAT32)
  field2.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field2.count = 1;

  field3.name = "z";
  field3.offset = 8;  // Offset of 8 bytes (since "x" and "y" are FLOAT32)
  field3.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field3.count = 1;

  field4.name = "r";
  field4.offset = 12;
  field4.datatype = sensor_msgs::msg::PointField::UINT8;
  field4.count = 1;

  field5.name = "g";
  field5.offset = 13;
  field5.datatype = sensor_msgs::msg::PointField::UINT8;
  field5.count = 1;

  field6.name = "b";
  field6.offset = 14;
  field6.datatype = sensor_msgs::msg::PointField::UINT8;
  field6.count = 1;

  // Add the fields to the PointCloud2 message
  pcl_msg.fields.push_back(field1);
  pcl_msg.fields.push_back(field2);
  pcl_msg.fields.push_back(field3);
  pcl_msg.fields.push_back(field4);
  pcl_msg.fields.push_back(field5);
  pcl_msg.fields.push_back(field6);

  // Set other attributes of the PointCloud2 message
  pcl_msg.header.stamp = rclcpp::Clock().now();
  pcl_msg.header.frame_id = "map";  // Set the frame ID
  pcl_msg.is_bigendian = false;
  pcl_msg.point_step = 15;  // The size of each point in bytes (4 fields of FLOAT32)
  pcl_msg.row_step = 15;  // Total size of the point cloud in bytes
  pcl_msg.is_dense = true;
  pcl_msg.height = 1;  // 1 indicates that we're using an unorganized point cloud


  pcl::toROSMsg(labels, pcl_msg);
  pcl_msg.header.frame_id = "map";
  return pcl_msg;
}

 