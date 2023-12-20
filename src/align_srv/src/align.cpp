#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/align.hpp"
#include "custom_interfaces/msg/transformation.hpp"

#include "agents.h"

#include <memory>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


rclcpp::Publisher<custom_interfaces::msg::Transformation>::SharedPtr tfPub;


// Convert a OcTree or OcTreeStamped to a PointCloud2
template <typename T>
void tree2PointCloud(T *tree, pcl::PointCloud<pcl::PointXYZ>& pclCloud) {
  for (typename T::leaf_iterator it = tree->begin_leafs(),
       end = tree->end_leafs(); it != end; ++it)
  {
    if (tree->isNodeOccupied(*it)) {
      pclCloud.push_back(
          pcl::PointXYZ(it.getX(),
            it.getY(),
            it.getZ()
            )
          );
    }
  }
}

bool pointInBBox(pcl::PointXYZ& point,
                 pcl::PointXYZ& bboxMin,
                 pcl::PointXYZ& bboxMax) {

  return (point.x < bboxMax.x && point.x > bboxMin.x) &&
         (point.y < bboxMax.y && point.y > bboxMin.y) &&
         (point.z < bboxMax.z && point.z > bboxMin.z);
}

Eigen::Matrix4f getICPTransformation2(pcl::PointCloud<pcl::PointXYZ>& cloud1,pcl::PointCloud<pcl::PointXYZ>& cloud2, Eigen::Matrix4f& tf) {


    // apply the initial transformation to cloudFrom
    pcl::transformPointCloud(cloud1, cloud1, tf);


    // get the bounding region of cloud1 to extract the points from cloud2 contained in the region
    pcl::PointXYZ minCloud1; pcl::PointXYZ maxCloud1;
    pcl::getMinMax3D(cloud1, minCloud1, maxCloud1);

    //Aplly the transformation error threshold
    minCloud1 = pcl::PointXYZ(
        minCloud1.x - initTransMaxError,
        minCloud1.y - initTransMaxError,
        minCloud1.z - initTransMaxError
        );
    maxCloud1 = pcl::PointXYZ(
        maxCloud1.x + initTransMaxError,
        maxCloud1.y + initTransMaxError,
        maxCloud1.z + initTransMaxError
        );

    // filter out the points in cloud2 that are not in cloud1’s range
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud2.begin();it != cloud2.end(); it++) {
      if (pointInBBox(*it, minCloud1, maxCloud1)) {
        cloud2filtered->push_back(*it);
      }
    }

    // filter out the points in cloud1 that are not in cloud2’s range
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ minCloud2filtered; pcl::PointXYZ maxCloud2filtered;
    pcl::getMinMax3D(*cloud2filtered, minCloud2filtered, maxCloud2filtered);

    minCloud2filtered = pcl::PointXYZ(
        minCloud2filtered.x - initTransMaxError,
        minCloud2filtered.y - initTransMaxError,
        minCloud2filtered.z - initTransMaxError
        );

    maxCloud2filtered = pcl::PointXYZ(
        maxCloud2filtered.x + initTransMaxError,
        maxCloud2filtered.y + initTransMaxError,
        maxCloud2filtered.z + initTransMaxError
        );

    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud1.begin();it != cloud1.end(); it++) {
      if (pointInBBox(*it, minCloud2filtered, maxCloud2filtered)) {
        cloud1filtered->push_back(*it);
      }
    }

    // Downsample for consistency and speed
    PointCloud::Ptr src(new PointCloud);
    PointCloud::Ptr tgt(new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize(downsampleRate * mapResolution, downsampleRate * mapResolution, downsampleRate * mapResolution);
    grid.setInputCloud(cloud1filtered);

    grid.filter(*src);
    grid.setInputCloud(cloud2filtered);

    grid.filter(*tgt);

    // Align
    pcl::IterativeClosestPointNonLinear<PointT, PointT> icp;
    //termination condition
    icp.setTransformationEpsilon(mapResolution / epsilonFactor);
    icp.setMaxCorrespondenceDistance(initTransMaxError);

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
    PointCloud::Ptr alignedCloud;

    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    alignedCloud=src;
    // align the cloud
    icp.setMaximumIterations(maxIterations);
    for (int i=0; i < 500; ++i) {
      src = alignedCloud;

      // Estimate
      icp.setInputSource(src);
      icp.align(*alignedCloud);

      // accumulate transformation between each Iteration
      Ti = icp.getFinalTransformation() * Ti;

      //refine ur parameters
      // icp.setMaxCorrespondenceDistance(initTransMaxError/2);

    }

    //save the resultant tf
    Ti =Ti.inverse().eval();
    return Ti*tf;

}

Eigen::Matrix4f getICPTransformation(
    pcl::PointCloud<pcl::PointXYZ>& cloud1,
    pcl::PointCloud<pcl::PointXYZ>& cloud2,
    Eigen::Matrix4f& tfEst,
    double mapRes) {

  // apply the tfEst to cloud2
  pcl::transformPointCloud(cloud2, cloud2, tfEst);

  // get the bounding region of cloud1 to
  // extract the points from cloud2 contained in the region
  pcl::PointXYZ minCloud1; pcl::PointXYZ maxCloud1;
  pcl::getMinMax3D(cloud1, minCloud1, maxCloud1);

  // filter out the points in cloud2 that are not in cloud1’s range
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2filtered(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud2.begin();
      it != cloud2.end(); it++) {

    if (pointInBBox(*it, minCloud1, maxCloud1)) {
      cloud2filtered->push_back(*it);
    }
  }

  // filter out the points in cloud1 that are not in cloud2’s range
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1filtered(
      new pcl::PointCloud<pcl::PointXYZ>);

  // same for other cloud
  pcl::PointXYZ minCloud2filtered; pcl::PointXYZ maxCloud2filtered;
  pcl::getMinMax3D(*cloud2filtered, minCloud2filtered, maxCloud2filtered);

  minCloud2filtered = pcl::PointXYZ(
      minCloud2filtered.x - 1,
      minCloud2filtered.y - 1,
      minCloud2filtered.z - 1
      );

  maxCloud2filtered = pcl::PointXYZ(
      maxCloud2filtered.x + 1,
      maxCloud2filtered.y + 1,
      maxCloud2filtered.z + 1
      );

  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud1.begin();
      it != cloud1.end(); it++) {

    if (pointInBBox(*it, minCloud2filtered, maxCloud2filtered)) {
      cloud1filtered->push_back(*it);
    }
  }

  // Downsample for consistency and speed
  PointCloud::Ptr src(new PointCloud);
  PointCloud::Ptr tgt(new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize(10 * mapRes, 10 * mapRes, 10 * mapRes);
  grid.setInputCloud(cloud1filtered);
  grid.filter(*tgt);

  grid.setInputCloud(cloud2filtered);
  grid.filter(*src);

  // Align
  pcl::IterativeClosestPointNonLinear<PointT, PointT> reg;
  reg.setTransformationEpsilon(mapRes / 60);
  reg.setMaxCorrespondenceDistance(20 * mapRes);//try 10

  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev;
  PointCloud::Ptr reg_result;

  if (src->size() < tgt->size()) {
    reg.setInputSource(src);
    reg.setInputTarget(tgt);

    // Run the same optimization in a loop and visualize the results
    reg_result = src;
    reg.setMaximumIterations(4);
    for (int i=0; i < 500; ++i) {
      // save cloud for visualization purpose
      src = reg_result;

      // Estimate
      reg.setInputSource(src);
      reg.align(*reg_result);

      // accumulate transformation between each Iteration
      Ti = reg.getFinalTransformation() * Ti;

      prev = reg.getLastIncrementalTransformation();
    }
  } else {
    reg.setInputSource(tgt);
    reg.setInputTarget(src);

    // Run the same optimization in a loop and visualize the results
    reg_result = tgt;
    reg.setMaximumIterations(4);
    for (int i=0; i < 500; ++i) {
      // save cloud for visualization purpose
      tgt = reg_result;

      // Estimate
      reg.setInputSource(tgt);
      reg.align(*reg_result);

      // accumulate transformation between each Iteration
      Ti = reg.getFinalTransformation() * Ti;

      prev = reg.getLastIncrementalTransformation();
    }
    Ti = Ti.inverse().eval();

  }

  return Ti * tfEst;
}


geometry_msgs::msg::Twist transformationMatrixToPose(const Eigen::Matrix4f& transformation_matrix)
{
    geometry_msgs::msg::Twist msg;
    // Extract translation (x, y, z) from the transformation matrix
    msg.linear.x = transformation_matrix(0, 3);
    msg.linear.y = transformation_matrix(1, 3);
    msg.linear.z = transformation_matrix(2, 3);

    // Extract rotation (roll, pitch, yaw) from the transformation matrix
    msg.angular.z = atan2(transformation_matrix(1, 0), transformation_matrix(0, 0));
    msg.angular.y = atan2(-transformation_matrix(2, 0), sqrt(transformation_matrix(2, 1) * transformation_matrix(2, 1) + transformation_matrix(2, 2) * transformation_matrix(2, 2)));
    msg.angular.x = atan2(transformation_matrix(2, 1), transformation_matrix(2, 2));

    return msg;
}
Eigen::Matrix4f poseToTransformationMatrix(const typename geometry_msgs::msg::Twist msg)
{
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
void align(const std::shared_ptr<custom_interfaces::srv::Align::Request> request,
          std::shared_ptr<custom_interfaces::srv::Align::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request for ID: %ld to %ld", request->idfrom ,request->idto);

    //convert octomap tp pcl
    octomap::OcTree *treefrom; 
    octomap::OcTree *treeto; 

    treefrom = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(request->octofrom);
    treeto = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(request->octoto);

    // make point clouds from each map
    pcl::PointCloud<pcl::PointXYZ> cloudFrom; tree2PointCloud(treefrom, cloudFrom);
    pcl::PointCloud<pcl::PointXYZ> cloudTo; tree2PointCloud(treeto, cloudTo);

    //get the icp transformation
    Eigen::Matrix4f init_tf= poseToTransformationMatrix(request->tf);
    Eigen::Matrix4f new_transform = getICPTransformation(cloudFrom, cloudTo, init_tf,0.05);

    geometry_msgs::msg::Twist refinedtf=transformationMatrixToPose(new_transform);
    custom_interfaces::msg::Transformation output_msg;
    output_msg.idfrom=request->idfrom;
    output_msg.idto=request->idto;
    output_msg.tf=refinedtf;
    tfPub->publish(output_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Done: %ld to %ld", request->idfrom ,request->idto);

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("align_server");

  rclcpp::Service<custom_interfaces::srv::Align>::SharedPtr service =
    node->create_service<custom_interfaces::srv::Align>("align", &align);

  tfPub= node->create_publisher<custom_interfaces::msg::Transformation>("/refined_tf", 10);



  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to align.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}