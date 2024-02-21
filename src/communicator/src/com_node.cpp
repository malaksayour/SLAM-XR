#include "agents.h"
#include "custom_interfaces/srv/downsample.hpp"
#include "custom_interfaces/srv/align.hpp"
#include "custom_interfaces/srv/savemap.hpp"
#include "custom_interfaces/srv/loadmap.hpp"
#include "custom_interfaces/srv/mapnames.hpp"

#include "custom_interfaces/msg/transformation.hpp"
#include "custom_interfaces/msg/instance.hpp"

#include <chrono>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include "custom_functions.cpp"

#include <nav_msgs/msg/occupancy_grid.hpp>

//#include "database.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

typedef pcl::PointXYZ PointT;


std::map<int, octomap_msgs::msg::Octomap> octomapDictionary;
std::map<int, int> stateCount;
std::map<int, int> mergeCount;
sensor_msgs::msg::PointCloud2 add_pcl;
sensor_msgs::msg::PointCloud2 delete_pcl;

//to be fixed DAROURE!


// struct emanticOctreeNode : public octomap::OcTreeNode {
//     std::string label;
// };





class Merger : public rclcpp::Node
{
  public:
    Merger() : Node("merger")
    {
      merged_topic=declare_parameter("mergedMapTopic" ,"merged_map_topic");
      refined_tf_topic=declare_parameter("refinedTfTopic" ,"refined_tf");
      edits_topic=declare_parameter("editTopic" ,"com/edits");

      timer_ = this->create_wall_timer(7s, std::bind(&Merger::timerCallback, this));
      mergedMapPub= create_publisher<octomap_msgs::msg::Octomap>(merged_topic, 10);
      initializeSubscribers();
      alignedMaps.push_back(1);
    }

  private:
    std::string merged_topic;
    std::string refined_tf_topic;
    std::string edits_topic;


    rclcpp::Subscription<custom_interfaces::msg::Transformation>::SharedPtr tfSub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr stateSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr editsSub;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr mergedMapPub;
    std::vector<int> alignedMaps;
    octomap::OcTree *mergedTree=new octomap::OcTree(mapResolution); 
    octomap::OcTree *mergedWeights=new octomap::OcTree(mapResolution); 
    std::map<int, Eigen::Matrix4f> tfRefinedDictionary;



    void timerCallback(){
      /**
       * @brief merges all submaps and ppublishes the result as an octomap.
      */
      RCLCPP_INFO(rclcpp::get_logger("com_node"),"merging \n");
      //get updated aligned maps
      if(alignedMaps.size()>1){
        std::vector<int> toMergeSubmaps;
        
        //get the updated submaps
        for (int i : alignedMaps) {
          if( stateCount[i]>mergeCount[i]) {
            toMergeSubmaps.push_back(i); 
            RCLCPP_INFO(rclcpp::get_logger("com_node"),"pushed submap %d \n",i );
          }
        }
        
        //merge the submaps
        for (int submapIndex : toMergeSubmaps) {
          RCLCPP_INFO(rclcpp::get_logger("com_node"),"merging submap %d \n", submapIndex);

          //align the submap before merging
          octomap::OcTree *tree=(octomap::OcTree*)octomap_msgs::binaryMsgToMap(octomapDictionary[submapIndex]);
          //since for now we are considering octo1 to be the base map, we will not transform the first map
          if (submapIndex!=1) tree=applyTf(tree, tfRefinedDictionary[submapIndex]);

          //merge it with the merged map
          for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(),end = tree->end_leafs(); it != end; ++it)
          {
            octomap::OcTreeKey nodeKey = it.getKey();
            octomap::OcTreeNode *nodeIn1 = mergedTree->search(nodeKey);
            if (nodeIn1 == NULL){
              mergedTree->setNodeValue(nodeKey, it->getLogOdds());
              //populate the weight map
              mergedWeights->setNodeValue(nodeKey, agentWeights[submapIndex]);               
            }
            else{
              if (nodeIn1->getValue()==100 or nodeIn1->getValue()==-100 ) continue;
              octomap::OcTreeNode *weightNode = mergedWeights->search(nodeKey);
              float accWeight=weightNode->getLogOdds();
              float val=(accWeight*nodeIn1->getLogOdds()+agentWeights[submapIndex]*it->getLogOdds())/
              (accWeight+agentWeights[submapIndex]);
              //update the merged map
              it->setLogOdds(val);
              //update the weight map
              weightNode->setLogOdds(val);
            }
          }
          //update the submap merge counter
          mergeCount[submapIndex]=stateCount[submapIndex];
          delete tree;
          tree=NULL;
        }
        //publish the merged map
        octomap_msgs::msg::Octomap msg;
        mergedTree->prune();
        octomap_msgs::binaryMapToMsg(*mergedTree, msg);
        msg.header.stamp = rclcpp::Time(); 
        msg.header.frame_id = "map";
        msg.id = "OcTree"; // Required to convert OcTreeStamped into regular OcTree
        RCLCPP_INFO(rclcpp::get_logger("merger_node"),"Publishing the edited map  \n");
        octomapDictionary[0]=msg;
        mergedMapPub->publish(msg);        
      }
    }

    void initializeSubscribers() {
      //tf subscriber. receives the refined transformaton form the align srv and stores in tfRefinedDictionary
      auto callback_tf =
      [this](const typename custom_interfaces::msg::Transformation::SharedPtr msg) -> void
      { //first check if it was aligned before
        std::lock_guard<std::mutex> lock(mtx_);
        //for now we will assume we always need to align to 1
        std::cout << "refined tf received" <<std::endl;
        tfRefinedDictionary[msg->idfrom]=poseToTransformationMatrix(msg->tf); //convert from twist to matrix
        if (!findIndex(msg->idfrom,alignedMaps)) alignedMaps.push_back(msg->idfrom);
      };
      tfSub=create_subscription<custom_interfaces::msg::Transformation>(
      refined_tf_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_tf);
      
      //Edits cb
      auto callback_edits =
      [this](const typename std_msgs::msg::Bool::SharedPtr msg) -> void
      { //first check if it was aligned before
      if (msg->data){
        

        pcl::PointCloud<PointT>::Ptr addCloud(new pcl::PointCloud<PointT>);
        pcl::moveFromROSMsg(add_pcl,*addCloud);
        pcl::PointCloud<PointT>::Ptr deleteCloud(new pcl::PointCloud<PointT>);
        pcl::moveFromROSMsg(delete_pcl,*deleteCloud);
        std::lock_guard<std::mutex> lock(mtx_);
        for (std::size_t i = 0; i < addCloud->size(); ++i) {
          PointT point=addCloud->points[i];
          mergedTree->setNodeValue(point.x, point.y, point.z, 100);
        }
        for (std::size_t i = 0; i < deleteCloud->size(); ++i) {
          PointT point=deleteCloud->points[i];
          mergedTree->setNodeValue(point.x, point.y, point.z, -100);
        }
        
      }
      };
      editsSub=create_subscription<custom_interfaces::msg::Transformation>(
      edits_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_edits);
    }
};

class Communicator : public rclcpp::Node
{
  public:
    Communicator() : Node("communicator")
    {
      oct_base_topic=declare_parameter("octoBaseTopic" ,"octo");
      delete_base_topic=declare_parameter("deleteBaseTopic" ,"human/delete");
      add_base_topic=declare_parameter("addBaseTopic" ,"human/add");
      downsample_request_topic=declare_parameter("downsampleRequestTopic" ,"human/downsampled_request");
      transformation_topic=declare_parameter("transformationTopic" ,"human/Transformation");
      downsampled_topic=declare_parameter("downsamplerTopic" ,"com/downsampled");
      human_label_sub_topic=declare_parameter("humanLabelSub" ,"human/human_label");
      human_label_pub_topic=declare_parameter("humanLabelPub" ,"com/human_label");
      semantic_topic_sub=declare_parameter("semanticTopicSub" ,"semantic_node/semantic_pcl");
      semantic_topic_pub=declare_parameter("semanticTopicPub" ,"com/semantic_pcl");
      save_map_topic=declare_parameter("saveMapTopic" ,"human/save_map");
      human_delete_label_topic=declare_parameter("humanDeleteLabel" ,"human/delete_label");
      human_delete_instance_topic=declare_parameter("humanDeleteInstance" ,"human/delete_instance");

      semantic_delete_label_topic=declare_parameter("semanticDeleteLabel" ,"com/delete_label");
      semantic_delete_instance_topic=declare_parameter("deleteInstance" ,"com/delete_instance");
      load_map_topic=declare_parameter("loadMapRequest" ,"human/load_map");
      map_names_topic=declare_parameter("mapNamesTopic" ,"com/map_names");
      map_names_request_topic=declare_parameter("mapNamesRequestTopic" ,"human/map_names");
      localize_human_topic=declare_parameter("localizeHumanTopic" ,"human/localize");
      goal_human_topic=declare_parameter("goalHumanTopic" ,"human/goal");
      localize_com_topic=declare_parameter("localizeComTopic" ,"com/localize");
      goal_com_topic=declare_parameter("goalComTopic" ,"/goal_pose");


      downsamplerClient = this->create_client<custom_interfaces::srv::Downsample>("downsample");
      downsamplerPub= create_publisher<sensor_msgs::msg::PointCloud2>(downsampled_topic, 10);
      labelPub= create_publisher<sensor_msgs::msg::PointCloud2>(human_label_pub_topic, 10);
      semanticPub= create_publisher<sensor_msgs::msg::PointCloud2>(semantic_topic_pub, 10);
      deleteLabelPub= create_publisher<std_msgs::msg::Int16>(semantic_delete_label_topic, 10);
      deleteInstancePub= create_publisher<custom_interfaces::msg::Instance>(semantic_delete_instance_topic, 10);
      mapNamesPub= create_publisher<std_msgs::msg::String>(map_names_topic, 10);
      localizationPub= create_publisher<geometry_msgs::msg::Twist>(localize_com_topic, 10);
      goalPub= create_publisher<geometry_msgs::msg::Twist>(goal_com_topic, 10);

      loaded_map_topic = oct_base_topic + "_" + std::to_string(agent_number);
      loadMapPub= create_publisher<octomap_msgs::msg::Octomap>(loaded_map_topic, 10);

      alignClient = this->create_client<custom_interfaces::srv::Align>("align");
      saveMapClient = this->create_client<custom_interfaces::srv::Savemap>("save_map");
      loadMapClient = this->create_client<custom_interfaces::srv::Loadmap>("load_map");
      mapNamesClient = this->create_client<custom_interfaces::srv::Mapnames>("map_names");

      initializeSubscribers();
    }


  private:
    std::string oct_base_topic;
    std::string delete_base_topic;
    std::string add_base_topic;
    std::string downsample_request_topic;
    std::string transformation_topic;
    std::string downsampled_topic;
    std::string human_label_sub_topic;
    std::string human_label_pub_topic;
    std::string semantic_topic_sub;
    std::string semantic_topic_pub;
    std::string save_map_topic;
    std::string human_delete_label_topic;
    std::string semantic_delete_label_topic;
    std::string human_delete_instance_topic;
    std::string load_map_topic;
    std::string map_names_topic;
    std::string map_names_request_topic;
    std::string loaded_map_topic;
    std::string localize_human_topic;
    std::string localize_com_topic;
    std::string goal_human_topic;
    std::string goal_com_topic;
    std::string semantic_delete_instance_topic;


    sensor_msgs::msg::PointCloud2 labeledMergedPcl;


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr add_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr delete_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr label_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr semantic_sub;

    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr request_map_sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr delete_label_sub;
    rclcpp::Subscription<custom_interfaces::msg::Instance>::SharedPtr delete_instance_sub;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr save_map_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr load_map_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr map_names_sub;



    rclcpp::Subscription<custom_interfaces::msg::Transformation>::SharedPtr align_map_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr localize_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr goal_sub;

    std::map<int, rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr> subDictionary;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsamplerPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr semanticPub;
    

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr labelPub;

    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr deleteLabelPub;
    rclcpp::Publisher<custom_interfaces::msg::Instance>::SharedPtr deleteInstancePub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapNamesPub;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr loadMapPub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr localizationPub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr goalPub;


    rclcpp::Client<custom_interfaces::srv::Align>::SharedPtr alignClient;
    rclcpp::Client<custom_interfaces::srv::Downsample>::SharedPtr downsamplerClient;
    rclcpp::Client<custom_interfaces::srv::Savemap>::SharedPtr saveMapClient;
    rclcpp::Client<custom_interfaces::srv::Loadmap>::SharedPtr loadMapClient;
    rclcpp::Client<custom_interfaces::srv::Mapnames>::SharedPtr mapNamesClient;

    void initializeSubscribers() {

      RCLCPP_INFO(rclcpp::get_logger("com_node"),"Initialising Subscribers \n");

      //create a subscriber for each agent map
      for (int i = 1; i <= agent_number; i++)
      {
        std::string topic_name = oct_base_topic + "_" + std::to_string(i);
        RCLCPP_INFO(rclcpp::get_logger("com_node"),"creating topic " +topic_name +" \n");
        subDictionary[i]=create_subscription<octomap_msgs::msg::Octomap>(
        topic_name, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            [this, i](const octomap_msgs::msg::Octomap::SharedPtr msg) {
                //RCLCPP_INFO(rclcpp::get_logger("com_node"),"octomap received from %d \n", i);
                std::lock_guard<std::mutex> lock(mtx_);
                octomapDictionary[i] = *msg;
                stateCount[i]=stateCount[i]+1;
            });
      }
      //create a subscriber for the human edits /addition
      auto callback_add =
      [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
      {
        std::lock_guard<std::mutex> lock(mtx_);
        add_pcl = *msg;
      };
      add_sub=create_subscription<sensor_msgs::msg::PointCloud2>(
      add_base_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_add);

      //create a subscriber for the human edits /deletion
      auto callback_delete =
      [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
      {
        std::lock_guard<std::mutex> lock(mtx_);
        delete_pcl = *msg;
      };
      delete_sub=create_subscription<sensor_msgs::msg::PointCloud2>(
      delete_base_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_delete);

      //create a subscriber for the human labeled points
      auto callback_label =
      [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
      {
        labelPub->publish(*msg);
      };
      label_sub=create_subscription<sensor_msgs::msg::PointCloud2>(
      human_label_sub_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_label);

      // create  a calback for merged labeled generated maps.
      auto callback_semantics =
      [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
      {
        labeledMergedPcl=*msg;
        semanticPub->publish(*msg);
      };
      semantic_sub=create_subscription<sensor_msgs::msg::PointCloud2>(
      semantic_topic_sub, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_semantics);

      //create a subscriber for the requesting minimaps
      auto callback_request_map =
      [this](const std_msgs::msg::Int16::SharedPtr msg) -> void
      {
        auto request = std::make_shared<custom_interfaces::srv::Downsample::Request>();
        request->id=msg->data;   
        request->octo_in=octomapDictionary[msg->data];
        // wait for the service to become available
        while (!downsamplerClient->wait_for_service(1s)) {
          if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          }
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }   

        auto future = downsamplerClient->async_send_request(request);
        while(!future.valid()) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " waiting again...");
        }         
        downsamplerPub->publish(future.get()->pcl_output);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map: %ld downsampled", msg->data);   


      };
      rclcpp::SubscriptionOptions options;
      options.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      request_map_sub=create_subscription<std_msgs::msg::Int16>(
      downsample_request_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_request_map,options);

      //create a subscriber for deleting all intances under a crtain label
      auto callback_delete_label =
      [this](const std_msgs::msg::Int16::SharedPtr msg) -> void
      {
        deleteLabelPub->publish(*msg);
      };
      delete_label_sub=create_subscription<std_msgs::msg::Int16>(
      human_delete_label_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_delete_label);

      //create a subscriber for deleting a certain instance
      auto callback_delete_instance =
      [this](const custom_interfaces::msg::Instance::SharedPtr msg) -> void
      {
        deleteInstancePub->publish(*msg);
      };
      delete_instance_sub=create_subscription<custom_interfaces::msg::Instance>(
      human_delete_instance_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_delete_instance);

      //create a subscriber for the saving the generated map into a file.
      auto callback_save_map =
      [this](const std_msgs::msg::String::SharedPtr msg) -> void
      {
        auto request = std::make_shared<custom_interfaces::srv::Savemap::Request>();
        request->final_pcl=labeledMergedPcl;   
        request->final_octomap=octomapDictionary[0];
        request->file_name=msg->data;
        // wait for the service to become available
        while (!saveMapClient->wait_for_service(1s)) {
          if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          }
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }   

        auto future = saveMapClient->async_send_request(request);
        while(!future.valid()) {
        }         
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map saved");   
      };
      save_map_sub=create_subscription<std_msgs::msg::String>(
      save_map_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_save_map, options);

      //create a subscriber for the saving the generated map into a file.
      auto callback_load_map =
      [this](const std_msgs::msg::String::SharedPtr msg) -> void
      {
        auto request = std::make_shared<custom_interfaces::srv::Loadmap::Request>();
        request->map_name=msg->data;
        // wait for the service to become available
        while (!loadMapClient->wait_for_service(1s)) {
          if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          }
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }   

        auto future = loadMapClient->async_send_request(request);
        while(!future.valid()) {
        }         
        //publish the octomap saved to the last agent topic
        loadMapPub->publish(future.get()->octo_output);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map names sent");   
        
      };
      load_map_sub=create_subscription<std_msgs::msg::String>(
      load_map_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_load_map, options);


     //create a subscriber for the saving the generated map into a file.
      auto callback_map_name =
      [this](const std_msgs::msg::Bool::SharedPtr msg) -> void
      {
        if(msg->data){
          auto request = std::make_shared<custom_interfaces::srv::Mapnames::Request>();
          // wait for the service to become available
          while (!mapNamesClient->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
          }   

          auto future = mapNamesClient->async_send_request(request);
          while(!future.valid()) {
          }         
          auto output=future.get()->resp;
          for (int i = 0; i < output.size(); i++)
          {
            std_msgs::msg::String name;
            name.data=output[i]; 
            mapNamesPub->publish(name);
          }
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map names sent");   
        }
      };
      map_names_sub=create_subscription<std_msgs::msg::Bool>(
      map_names_request_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_map_name, options);

      //create a subscriber for alignment
      auto callback_align_map =
      [this](const custom_interfaces::msg::Transformation::SharedPtr msg) -> void
      {
        auto request = std::make_shared<custom_interfaces::srv::Align::Request>();
        request->idfrom=msg->idfrom;
        request->idto=msg->idto;
        request->octoto=octomapDictionary[msg->idto];    
        request->octofrom=octomapDictionary[msg->idfrom];  
        request->tf=msg->tf;    
        // wait for the service to become available
        while (!alignClient->wait_for_service(1s)) {
          if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          }
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }   
        auto future = alignClient->async_send_request(request);
      };
      align_map_sub=create_subscription<custom_interfaces::msg::Transformation>(
      transformation_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_align_map,options);

      //create a subscriber for localization
      auto callback_localize =
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
      {
        localizationPub->publish(*msg);
      };
      localize_sub=create_subscription<geometry_msgs::msg::Twist>(
      localize_human_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_localize);

      //create a subscriber for goal navigation
      auto callback_goal =
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
      {
        goalPub->publish(*msg);
      };
      goal_sub=create_subscription<geometry_msgs::msg::Twist>(
      goal_human_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_goal);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto communicator=  std::make_shared<Communicator>();
  auto merger= std::make_shared<Merger>();


  executor.add_node(communicator);
  executor.add_node(merger);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}

