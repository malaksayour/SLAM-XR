#include "agents.h"
#include "custom_interfaces/msg/transformation.hpp"
#include "custom_interfaces/msg/instance.hpp"

#include "custom_functions.cpp"
#include <std_msgs/msg/int16.hpp>

using namespace std::chrono_literals;


typedef pcl::PointXYZRGB PointL;
typedef pcl::PointCloud<PointL> PointCloudLabeled;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;




class SemanticMap : public rclcpp::Node
{

  public:
    SemanticMap() : Node("semanticMap")
    {
      merged_octomap_topic=declare_parameter("mergedMapTopic" ,"merged_map_topic");
      human_label_sub_topic=declare_parameter("comHumanLabel" ,"com/human_label");
      refined_tf_topic=declare_parameter("refinedTfTopic" ,"refined_tf");
      semantic_pcl_topic=declare_parameter("semanticPclTopic" ,"semantic_node/semantic_pcl");
      delete_label_topic=declare_parameter("deleteLabelTopic" ,"com/delete_label");
      delete_instance_topic=declare_parameter("deleteInstanceTopic" ,"com/delete_instance");

      delete_topic=declare_parameter("deleteLabelTopicPub" ,"human/delete");

      semanticMapPclPub= create_publisher<sensor_msgs::msg::PointCloud2>(semantic_pcl_topic, 10);
      deletePub= create_publisher<sensor_msgs::msg::PointCloud2>(delete_topic, 10);

      initializeSubscribers();
    }


  private:
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr mergedMapSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr humanLabelSub;
    rclcpp::Subscription<custom_interfaces::msg::Transformation>::SharedPtr tfSub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr deleteLabelSub;
    rclcpp::Subscription<custom_interfaces::msg::Instance>::SharedPtr deleteInstanceSub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr semanticMapPclPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deletePub;


    std::string merged_octomap_topic;
    std::string human_label_sub_topic;
    std::string refined_tf_topic;
    std::string semantic_pcl_topic;
    std::string delete_label_topic;
    std::string delete_topic;
    std::string delete_instance_topic;

    PointCloudLabeled *labeledPcl=new PointCloudLabeled(); 
    octomap::OcTree *mergedTree=new octomap::OcTree(mapResolution); 

    std::map<int, Eigen::Matrix4f> tfRefinedDictionary;

    void initializeSubscribers() {
      RCLCPP_INFO(rclcpp::get_logger("semantic_node"),"Initialising Subscribers \n");

      rclcpp::SubscriptionOptions options;
      options.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      //create a subscriber for the merged map
      auto callback_merged =
      [this](const typename octomap_msgs::msg::Octomap::SharedPtr msg) -> void
      {
        std::lock_guard<std::mutex> lock(mtx_);
        //republish only if changes in merged tree were observed.   

        semanticMapPclPub->publish(generate_semantic_pcl((octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg),labeledPcl));
        std::cout <<"done" << std::endl;

      };
      mergedMapSub=create_subscription<octomap_msgs::msg::Octomap>(
      merged_octomap_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), callback_merged);

      //create a callback for the human labels
      auto callback_human_label =
      [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
      {
        //get the label ID
        //int ID=(int)msg->header.stamp.nanosec;
        //transform to pcl
        PointCloudLabeled::Ptr inputCloud(new PointCloudLabeled);
        pcl::fromROSMsg(*msg,*inputCloud);
        pcl::transformPointCloud(*inputCloud, *inputCloud, tfRefinedDictionary[2]);

        std::lock_guard<std::mutex> lock(mtx_);      
        for (std::size_t i = 0; i < inputCloud->size(); ++i) {
          //std::cout <<i << std::endl;
          labeledPcl->push_back(inputCloud->points[i]);
        }
        std::cout <<"labeled" << std::endl;


      };
      humanLabelSub=create_subscription<sensor_msgs::msg::PointCloud2>(
      human_label_sub_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_human_label);

      //create a callback for the human labels
      auto callback_delete_label =
      [this](const typename std_msgs::msg::Int16::SharedPtr msg) -> void
      {
        PointCloud todelete;
        //get all points belonging to a certain label
        for (const auto& pclPoint : labeledPcl->points) {
          if (pclPoint.g==msg->data) {
            PointT point;
            point.x=pclPoint.x;
            point.y=pclPoint.y;
            point.z=pclPoint.z;
            todelete.push_back(point);
          }
        }
        // publish the resultant points to the delete callback.
        sensor_msgs::msg::PointCloud2 delete_msg;
        pcl::toROSMsg(todelete,delete_msg);
        deletePub->publish(delete_msg);

      };
      deleteLabelSub=create_subscription<std_msgs::msg::Int16>(
      delete_label_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_delete_label);

      //create a callback for the human labels
      auto callback_delete_instance =
      [this](const typename custom_interfaces::msg::Instance::SharedPtr msg) -> void
      {
        PointCloud todelete;
        //get all points belonging to a certain label
        for (const auto& pclPoint : labeledPcl->points) {
          if (pclPoint.g==msg->label and pclPoint.b==msg->instance) {
            PointT point;
            point.x=pclPoint.x;
            point.y=pclPoint.y;
            point.z=pclPoint.z;
            todelete.push_back(point);
          }
        }
        // publish the resultant points to the delete callback.
        sensor_msgs::msg::PointCloud2 delete_msg;
        pcl::toROSMsg(todelete,delete_msg);
        deletePub->publish(delete_msg);

      };
      deleteInstanceSub=create_subscription<custom_interfaces::msg::Instance>(
      delete_instance_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_delete_instance);


      //tf subscriber. receives the refined transformaton form the align srv and stores in tfRefinedDictionary
      auto callback_tf =
      [this](const typename custom_interfaces::msg::Transformation::SharedPtr msg) -> void
      { //first check if it was aligned before
        std::lock_guard<std::mutex> lock(mtx_);
        //for now we will assume we always need to align to 1
        tfRefinedDictionary[msg->idfrom]=poseToTransformationMatrix(msg->tf); //convert from twist to matrix
      };
      tfSub=create_subscription<custom_interfaces::msg::Transformation>(
      refined_tf_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_tf);
    }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto semanticMap= std::make_shared<SemanticMap>();
  executor.add_node(semanticMap);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}

