#include "agents.h"
#include "custom_interfaces/msg/transformation.hpp"
#include "custom_interfaces/msg/instance.hpp"

#include "custom_functions.cpp"
#include <std_msgs/msg/int16.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>



using namespace std::chrono_literals;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::MapMetaData;




typedef pcl::PointXYZRGB PointL;
typedef pcl::PointCloud<PointL> PointCloudLabeled;



class NavigationNode : public rclcpp::Node
{

  public:
    NavigationNode() : Node("NavigationNode")
    {
      merged_octomap_topic=declare_parameter("mergedMapTopic" ,"merged_map_topic");
      refined_tf_topic=declare_parameter("refinedTfTopic" ,"refined_tf");
      min_x_size_ = declare_parameter("min_x_size", 0.0);
      min_y_size_ = declare_parameter("min_y_size", 0.0);
      map_pub_ = create_publisher<OccupancyGrid>("projected_map", 5);

      {
        rcl_interfaces::msg::ParameterDescriptor occupancy_min_z_desc;
        occupancy_min_z_desc.description =
          "Minimum height of occupied cells to consider in the final map";
        rcl_interfaces::msg::FloatingPointRange occupancy_min_z_range;
        occupancy_min_z_range.from_value = -100.0;
        occupancy_min_z_range.to_value = 100.0;
        occupancy_min_z_desc.floating_point_range.push_back(occupancy_min_z_range);
        occupancy_min_z_ = declare_parameter("occupancy_min_z", -100.0, occupancy_min_z_desc);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor occupancy_max_z_desc;
        occupancy_max_z_desc.description =
          "Maximum height of occupied cells to consider in the final map";
        rcl_interfaces::msg::FloatingPointRange occupancy_max_z_range;
        occupancy_max_z_range.from_value = -100.0;
        occupancy_max_z_range.to_value = 100.0;
        occupancy_max_z_desc.floating_point_range.push_back(occupancy_max_z_range);
        occupancy_max_z_ = declare_parameter("occupancy_max_z", 100.0, occupancy_max_z_desc);
      }


      
      initializeSubscribers();
    }


  private:
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr mergedMapSub;
    rclcpp::Subscription<custom_interfaces::msg::Transformation>::SharedPtr tfSub;

    rclcpp::Publisher<OccupancyGrid>::SharedPtr map_pub_;


    std::string merged_octomap_topic;
    std::string refined_tf_topic;

    //2D projection
    OccupancyGrid gridmap_;
    octomap::OcTreeKey update_bbox_min_;
    octomap::OcTreeKey update_bbox_max_;
    octomap::OcTreeKey padded_min_key_;
    unsigned multires_2d_scale_;
    bool project_complete_map_;
    bool incremental_2D_projection_;
    double min_x_size_ ;
    double min_y_size_ ;
    size_t max_tree_depth_;
    octomap::OcTree *mergedTree=new octomap::OcTree(mapResolution); 
    double occupancy_min_z_;
    double occupancy_max_z_;

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
        mergedTree=(octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
        //generate grid 
        handlePreNodeTraversal();

        //handle free and occupied nodes.
        for (octomap::OcTree::leaf_iterator it = mergedTree->begin_leafs(),end = mergedTree->end_leafs(); it != end; ++it)
        {
          //octomap::OcTreeKey nodeKey = it.getKey();

          bool in_update_bbox = isInUpdateBBX(it);

          if (mergedTree->isNodeOccupied(*it) & it.getZ()<1 & it.getZ()>-0.25) {
            double z = it.getZ();
            double half_size = it.getSize() / 2.0;
            if (z + half_size > occupancy_min_z_ && z - half_size < occupancy_max_z_) {
              double x = it.getX();
              double y = it.getY();

              handleOccupiedNode(it);
              if (in_update_bbox) {
                handleOccupiedNodeInBBX(it);
              }
            }
          }
          else{
            double z = it.getZ();
            double half_size = it.getSize() / 2.0;
            if (z + half_size > occupancy_min_z_ && z - half_size < occupancy_max_z_) {
              handleFreeNode(it);
              if (in_update_bbox) {
                handleFreeNodeInBBX(it);
              }
            }
          }
        }
        // call post-traversal hook:
        handlePostNodeTraversal();

      };
      mergedMapSub=create_subscription<octomap_msgs::msg::Octomap>(
      merged_octomap_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), callback_merged);
     
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

  inline bool mapChanged(const MapMetaData & old_map_info, const MapMetaData & new_map_info)
  {
    return old_map_info.height != new_map_info.height ||
           old_map_info.width != new_map_info.width ||
           old_map_info.origin.position.x != new_map_info.origin.position.x ||
           old_map_info.origin.position.y != new_map_info.origin.position.y;
  }

  void handleOccupiedNode(const octomap::OcTree::leaf_iterator & it)
  {
    if (project_complete_map_) {
      update2DMap(it, true);
    }
  }

  void handleFreeNode(const octomap::OcTree::leaf_iterator & it)
  {
    if (project_complete_map_) {
      update2DMap(it, false);
    }
  }

  void handleOccupiedNodeInBBX(const octomap::OcTree::leaf_iterator & it)
  {
    if (project_complete_map_) {
      update2DMap(it, true);
    }
  }

  void handleFreeNodeInBBX(const octomap::OcTree::leaf_iterator & it)
  {
    if (project_complete_map_) {
      update2DMap(it, false);
    }
  }

  void update2DMap(const octomap::OcTree::leaf_iterator & it, bool occupied)
  {
    // update 2D map (occupied always overrides):
    if (it.getDepth() == max_tree_depth_) {
      unsigned idx = mapIdx(it.getKey());
      if (occupied) {
        gridmap_.data[mapIdx(it.getKey())] = 100;
      } else if (gridmap_.data[idx] == -1) {
        gridmap_.data[idx] = 0;
      }

    } else {
      int int_size = 1 << (max_tree_depth_ - it.getDepth());
      octomap::OcTreeKey min_key = it.getIndexKey();
      for (int dx = 0; dx < int_size; dx++) {
        int i = (min_key[0] + dx - padded_min_key_[0]) / multires_2d_scale_;
        for (int dy = 0; dy < int_size; dy++) {
          unsigned idx = mapIdx(i, (min_key[1] + dy - padded_min_key_[1]) / multires_2d_scale_);
          if (occupied) {
            gridmap_.data[idx] = 100;
          } else if (gridmap_.data[idx] == -1) {
            gridmap_.data[idx] = 0;
          }
        }
      }
    }
  }


  void adjustMapData(OccupancyGrid & map, const MapMetaData & old_map_info) const
{
  if (map.info.resolution != old_map_info.resolution) {
    RCLCPP_ERROR(get_logger(), "Resolution of map changed, cannot be adjusted");
    return;
  }

  int i_off =
    static_cast<int>((old_map_info.origin.position.x - map.info.origin.position.x) /
    map.info.resolution + 0.5);
  int j_off =
    static_cast<int>((old_map_info.origin.position.y - map.info.origin.position.y) /
    map.info.resolution + 0.5);

  if (i_off < 0 || j_off < 0 ||
    old_map_info.width + i_off > map.info.width ||
    old_map_info.height + j_off > map.info.height)
  {
    RCLCPP_ERROR(
      get_logger(), "New 2D map does not contain old map area, this case is not implemented");
    return;
  }

  OccupancyGrid::_data_type old_map_data = map.data;

  map.data.clear();
  // init to unknown:
  map.data.resize(map.info.width * map.info.height, -1);

  OccupancyGrid::_data_type::iterator from_start, from_end, to_start;

  for (size_t j = 0; j < old_map_info.height; ++j) {
    // copy chunks, row by row:
    from_start = old_map_data.begin() + j * old_map_info.width;
    from_end = from_start + old_map_info.width;
    to_start = map.data.begin() + ((j + j_off) * gridmap_.info.width + i_off);
    copy(from_start, from_end, to_start);

//    for (int i =0; i < int(old_map_info.width); ++i){
//      map.data[gridmap_.info.width*(j+j_off) +i+i_off] = oldMapData[old_map_info.width*j +i];
//    }
  }
}

  void handlePostNodeTraversal()
{
    map_pub_->publish(gridmap_);
}

  /// Test if key is within update area of map (2D, ignores height)
  inline bool isInUpdateBBX(const octomap::OcTree::leaf_iterator & it) const
  {
    // 2^(tree_depth-depth) voxels wide:
    unsigned voxelWidth = (1 << (max_tree_depth_ - it.getDepth()));
    octomap::OcTreeKey key = it.getIndexKey();  // lower corner of voxel
    return key[0] + voxelWidth >= update_bbox_min_[0] &&
           key[1] + voxelWidth >= update_bbox_min_[1] &&
           key[0] <= update_bbox_max_[0] &&
           key[1] <= update_bbox_max_[1];
  }

  void handlePreNodeTraversal(){
      
      size_t tree_depth_=mergedTree->getTreeDepth();
      max_tree_depth_ = tree_depth_;


      MapMetaData old_map_info = gridmap_.info;
            double res_=mergedTree->getResolution();







      // init projected 2D map:
      gridmap_.header.frame_id = "map";
      //gridmap_.header.stamp = rostime;
      gridmap_.info.resolution=res_;
      //MapMetaData old_map_info = gridmap_.info;

      // TODO(someone): move most of this stuff into c'tor
      // and init map only once (adjust if size changes)
      double min_x{};
      double min_y{};
      double min_z{};
      double max_x{};
      double max_y{};
      double max_z{};
      mergedTree->getMetricMin(min_x, min_y, min_z);
      mergedTree->getMetricMax(max_x, max_y, max_z);

      octomap::point3d min_pt(min_x, min_y, min_z);
      octomap::point3d max_pt(max_x, max_y, max_z);
      octomap::OcTreeKey min_key = mergedTree->coordToKey(min_pt, max_tree_depth_);
      octomap::OcTreeKey max_key = mergedTree->coordToKey(max_pt, max_tree_depth_);

      RCLCPP_DEBUG(
        get_logger(),
        "min_key: %d %d %d / max_key: %d %d %d", min_key[0], min_key[1], min_key[2], max_key[0],
        max_key[1], max_key[2]);

      // add padding if requested (= new min/max_pts in x&y):
      double half_padded_x = 0.5 * min_x_size_;
      double half_padded_y = 0.5 * min_y_size_;
      min_x = std::min(min_x, -half_padded_x);
      max_x = std::max(max_x, half_padded_x);
      min_y = std::min(min_y, -half_padded_y);
      max_y = std::max(max_y, half_padded_y);
      min_pt = octomap::point3d(min_x, min_y, min_z);
      max_pt = octomap::point3d(max_x, max_y, max_z);

      octomap::OcTreeKey padded_max_key;
      if (!mergedTree->coordToKeyChecked(min_pt, max_tree_depth_, padded_min_key_)) {
        RCLCPP_ERROR(
          get_logger(),
          "Could not create padded min OcTree key at %f %f %f", min_pt.x(), min_pt.y(),
          min_pt.z());
        return;
      }
      if (!mergedTree->coordToKeyChecked(max_pt, max_tree_depth_, padded_max_key)) {
        RCLCPP_ERROR(
          get_logger(),
          "Could not create padded max OcTree key at %f %f %f", max_pt.x(), max_pt.y(),
          max_pt.z());
        return;
      }

      RCLCPP_DEBUG(
        get_logger(),
        "Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", padded_min_key_[0],
        padded_min_key_[1], padded_min_key_[2], padded_max_key[0], padded_max_key[1],
        padded_max_key[2]);
      assert(padded_max_key[0] >= max_key[0] && padded_max_key[1] >= max_key[1]);

      multires_2d_scale_ = 1 << (tree_depth_ - max_tree_depth_);
      gridmap_.info.width = (padded_max_key[0] - padded_min_key_[0]) / multires_2d_scale_ + 1;
      gridmap_.info.height = (padded_max_key[1] - padded_min_key_[1]) / multires_2d_scale_ + 1;

      [[maybe_unused]] int map_origin_x = min_key[0] - padded_min_key_[0];
      [[maybe_unused]] int map_origin_y = min_key[1] - padded_min_key_[1];
      assert(map_origin_x >= 0 && map_origin_y >= 0);

      // might not exactly be min / max of octree:
      octomap::point3d origin = mergedTree->keyToCoord(padded_min_key_, tree_depth_);
      double grid_res = mergedTree->getNodeSize(max_tree_depth_);
      project_complete_map_ =
        (!incremental_2D_projection_ || (std::abs(grid_res - gridmap_.info.resolution) > 1e-6));
      gridmap_.info.resolution = grid_res;
      gridmap_.info.origin.position.x = origin.x() - grid_res * 0.5;
      gridmap_.info.origin.position.y = origin.y() - grid_res * 0.5;
      if (max_tree_depth_ != tree_depth_) {
        gridmap_.info.origin.position.x -= res_ / 2.0;
        gridmap_.info.origin.position.y -= res_ / 2.0;
      }

      // workaround for  multires. projection not working properly for inner nodes:
      // force re-building complete map
      if (max_tree_depth_ < tree_depth_) {
        project_complete_map_ = true;
      }

      if (project_complete_map_) {
        RCLCPP_DEBUG(get_logger(), "Rebuilding complete 2D map");
        gridmap_.data.clear();
        // init to unknown:
        gridmap_.data.resize(gridmap_.info.width * gridmap_.info.height, -1);

      } else {
        if (mapChanged(old_map_info, gridmap_.info)) {
          RCLCPP_DEBUG(
            get_logger(), "2D grid map size changed to %dx%d", gridmap_.info.width,
            gridmap_.info.height);
          adjustMapData(gridmap_, old_map_info);
        }
        OccupancyGrid::_data_type::iterator startIt;
        size_t mapUpdateBBXmin_x =
          std::max(
          0,
          (static_cast<int>(update_bbox_min_[0]) - static_cast<int>(padded_min_key_[0])) /
          static_cast<int>(multires_2d_scale_));
        size_t mapUpdateBBXmin_y =
          std::max(
          0,
          (static_cast<int>(update_bbox_min_[1]) - static_cast<int>(padded_min_key_[1])) /
          static_cast<int>(multires_2d_scale_));
        size_t mapUpdateBBXmax_x =
          std::min(
          static_cast<int>(gridmap_.info.width - 1),
          (static_cast<int>(update_bbox_max_[0]) - static_cast<int>(padded_min_key_[0])) /
          static_cast<int>(multires_2d_scale_));
        size_t mapUpdateBBXmax_y =
          std::min(
          static_cast<int>(gridmap_.info.height - 1),
          (static_cast<int>(update_bbox_max_[1]) - static_cast<int>(padded_min_key_[1])) /
          static_cast<int>(multires_2d_scale_));

        assert(mapUpdateBBXmax_x > mapUpdateBBXmin_x);
        assert(mapUpdateBBXmax_y > mapUpdateBBXmin_y);

        size_t numCols = mapUpdateBBXmax_x - mapUpdateBBXmin_x + 1;

        // test for max idx:
        uint max_idx = gridmap_.info.width * mapUpdateBBXmax_y + mapUpdateBBXmax_x;
        if (max_idx >= gridmap_.data.size()) {
          RCLCPP_ERROR(
            get_logger(),
            "BBX index not valid: %d (max index %zu for size %d x %d) "
            "update-BBX is: [%zu %zu]-[%zu %zu]", max_idx,
            gridmap_.data.size(), gridmap_.info.width, gridmap_.info.height,
            mapUpdateBBXmin_x, mapUpdateBBXmin_y, mapUpdateBBXmax_x, mapUpdateBBXmax_y);
        }

        // reset proj. 2D map in bounding box:
        for (unsigned int j = mapUpdateBBXmin_y; j <= mapUpdateBBXmax_y; ++j) {
          std::fill_n(
            gridmap_.data.begin() + gridmap_.info.width * j + mapUpdateBBXmin_x,
            numCols, -1);
        }
      }
  }

  inline size_t mapIdx(const int i, const int j) const
  {
    return gridmap_.info.width * j + i;
  }

  inline size_t mapIdx(const octomap::OcTreeKey & key) const
  {
    return mapIdx(
      (key[0] - padded_min_key_[0]) / multires_2d_scale_,
      (key[1] - padded_min_key_[1]) / multires_2d_scale_);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto navigationNode= std::make_shared<NavigationNode>();
  executor.add_node(navigationNode);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}

