#pragma once
#ifndef EXPLORATIONMAPPER_H
#define EXPLORATIONMAPPER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <iostream>
#include <fstream>
#include <visualization_msgs/MarkerArray.h>
/* #include <geometry_msgs/PoseStamped.h> */
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <spheremap_server/GetSphereMapPathSrv.h>
#include <spheremap_server/SegmapMsg.h>

#include <spheremap_server/mapping_structures.h>
#include <spheremap_server/spheremap.h>

#include <sensor_msgs/PointCloud2.h>

namespace spheremap_server
{
class ExplorationMapper {
public:
  int  neighborhood_6_dirs[6][3]     = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  bool _profiler_enabled_            = false;
  bool outdoor_testing_mode_enabled_ = false;

  bool enable_sending_ltvmap_fragments_ = false;
  bool enable_sending_ltvmap_markers_    = true;

  float spheremap_min_safe_dist_;
  float spheremap_fully_safe_dist_;

  float absolute_max_map_update_dist         = 70;
  bool  hires_mode_                          = false;
  float absolute_max_map_update_dist_highres = 40;
  float max_graph_dist_for_spheremap_planning_ = 10;
  float spheremap_planning_safety_weight_;
  float spheremap_planning_safety_bias_;

  bool        use_global_search_bbx_;
  BoundingBox global_search_bbx_;
  BoundingBox staging_area_deny_box_in_subt_frame_;
  BoundingBox staging_area_deny_box_transformed_;

  bool                    navigation_priority_flag_    = false;
  int                     segmap_default_depth_offset_ = 2;
  bool                    full_demonstration_mode_     = false;
  std::shared_ptr<SegMap> last_sent_segmap_rebulit     = NULL;
  SegmapMsg               smmsg;
  bool                    is_initialized_        = false;
  bool                    planning_debug_mode_   = false;
  bool                    has_occupancy_octomap_ = false;
  bool                    has_surface_octomap_   = false;
  bool                    has_odometry_          = false;
  bool                    use_ugv_settings_      = false;
  bool                    use_cam360_            = false;
  float                   cam360_vertical_angle_ = M_PI / 4;
  float                   cam360_max_dist_       = 30;

  float dbscan_epsilon_                           = 6;
  float frontier_node_filter_dist_                = 4;
  float frontier_node_frontier_size_threshold_    = 200;
  float frontier_cluster_frontier_size_threshold_ = 500;

  std::vector<octomap::point3d> ground_areas_        = {};
  float                         ground_areas_radius_ = 3;
  std::vector<octomap::point3d> executed_trajectory_ = {};
  std::vector<int>              segment_history_     = {};

  float surface_recalc_bbx_size_;
  float surface_recalc_bbx_size_largescale_;

  octomap_msgs::Octomap         octomap_msg_;
  octomap_msgs::OctomapConstPtr octomap_msg_ptr_;
  std::mutex                    octomap_mutex_;

  bool octomap_mutex_ptr_set_;

  bool has_new_occupancy_octomap_msg_ = false;
  void setOctomapMsgPtr(const octomap_msgs::OctomapConstPtr msg_ptr);
  void setOctomapMsgMutexPtr(std::mutex* mutex_ptr);

  std::mutex                       occupancy_octree_ptr_mutex_;
  std::shared_ptr<octomap::OcTree> getOccupancyOcTreeSharedPtr() {
    occupancy_octree_ptr_mutex_.lock();
    std::shared_ptr<octomap::OcTree> res = occupancy_octree_;
    occupancy_octree_ptr_mutex_.unlock();
    return res;
  }

  std::mutex              pcl_map_ptr_mutex_;
  std::shared_ptr<PCLMap> getPCLMapSharedPtr() {
    pcl_map_ptr_mutex_.lock();
    std::shared_ptr<PCLMap> res = pcl_map_;
    pcl_map_ptr_mutex_.unlock();
    return res;
  }

  std::string artifact_frame_ = "";

  bool surface_mapping_enabled_;
  bool topology_mapping_enabled_;

  void initialize(ros::NodeHandle* n);

  octomap::point3d current_position_;
  float            current_heading_;

  /* staging area */
  /* float staging_area_wall_x_; */
  /* float staging_area_wall_direction_; */
  /* bool  use_staging_area_wall_; */

  bool                subt_tf_received_ = false;
  StagingAreaSettings staging_area_settings_;

  /* transformations */
  std::string map_frame_;
  std::string robots_shared_frame_;
  std::string baselink_frame_;
  std::string uav_name_;

  tf2_ros::Buffer                             tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  /* tf2_ros::Buffer tf_buffer; */
  /* tf2_ros::TransformListener tf_listener(tf_buffer); */

  geometry_msgs::Pose current_pose_;
  nav_msgs::Odometry  current_odometry_;

  /* publishers */
  ros::Publisher pub_frontier_clusters_;
  ros::Publisher pub_facetmap_detailed_;
  ros::Publisher pub_sent_segmap_;

  ros::Publisher pub_spheremap_spheres_;
  ros::Publisher pub_spheremap_points_;
  ros::Publisher pub_spheremap_topology_;
  ros::Publisher pub_spheremap_navigation_;
  ros::Publisher pub_spheremap_debug_;
  ros::Publisher pub_visited_positions_map_;
  ros::Publisher pub_markers_planning_;

  /* ros::Publisher pub_topology_markers_; */
  ros::Publisher pub_experiment_data_octomap_;
  ros::Publisher pub_experiment_data_surface_nodes_;

  /* subscribers */
  ros::Subscriber sub_odometry_;
  void            callbackOdometryMessageReceived(const nav_msgs::Odometry);

  ros::Subscriber sub_octomap_binary_const_ptr_;
  void            callbackOctomapBinaryConstPtrReceived(const octomap_msgs::OctomapConstPtr octomap_msg);

  ros::Subscriber       sub_octomap_binary_;
  void                  callbackOctomapBinaryReceived(const octomap_msgs::Octomap octomap_msg);
  octomap_msgs::Octomap latest_octomap_msg;
  bool                  octomap_msg_is_in_constptr_format_ = false;

  /* services */
  ros::ServiceServer spheremap_planning_srv_;
  bool               callbackGetSphereMapPathSrv(GetSphereMapPathSrv::Request& req, GetSphereMapPathSrv::Response& resp);

  /* timers */
  ros::Timer timer_update_visited_positions_map_;
  double     rate_timer_update_visited_positions_map_;
  void       callbackTimerUpdateVisitedPositionsMap(const ros::TimerEvent& te);

  ros::Timer timer_publish_markers_;
  double     rate_timer_publish_markers_;
  void       callbackTimerPublishMarkers(const ros::TimerEvent& te);

  ros::Timer timer_update_exploredness_map_;
  double     rate_timer_update_exploredness_map_;
  void       callbackTimerRaycastExploredness(const ros::TimerEvent& te);

  ros::Timer timer_create_surface_nodes_;
  double     rate_timer_create_surface_nodes_;
  void       callbackTimerUpdateSurfaceMap(const ros::TimerEvent& te);

  ros::Timer timer_create_surface_nodes_largescale;
  double     rate_timer_create_surface_nodes_largescale;
  void       callbackTimerUpdateSurfaceMapLargescale(const ros::TimerEvent& te);

  ros::Timer timer_update_segmap_;
  double     rate_timer_update_segmap_;
  void       callbackTimerUpdateTopologyMap(const ros::TimerEvent& te);

  bool       exploration_data_publishing_enabled_ = false;
  ros::Timer timer_publish_exploration_data_;
  double     rate_timer_publish_exploration_data_;
  void       callbackTimerPublishExplorationData(const ros::TimerEvent& te);

  ros::Timer             timer_empty_topology_msgs_buffer_;
  std::vector<SegmapMsg> topology_messages_buffer_ = {};
  double                 rate_timer_empty_topology_msgs_buffer_;
  void                   callbackTimerEmptyTopologyMsgsBuffer(const ros::TimerEvent& te);

  std::mutex                          mutex_segmap_rebuild_buffer;
  std::vector<std::vector<SegmapMsg>> segmap_rebuild_buffer                         = {};
  std::vector<bool>                   segmap_rebuild_buffer_flags                   = {};
  std::vector<uint>                   received_segmaps_num_rebuilt                  = {};
  std::vector<uint>                   received_segmaps_num_unique_msg_ids_received_ = {};
  ros::Time                           last_rebuilding_stats_print_time_;

  ros::Timer timer_rebuild_received_segmaps_;
  double     rate_timer_rebuild_received_segmaps_;
  void       callbackTimerRebuildReceivedSegmaps(const ros::TimerEvent& te);

  /* kdtree */
  ros::Timer              timer_calculate_kdtree_;
  double                  rate_timer_calculate_kdtree_;
  void                    callbackTimerCalculateKdtree(const ros::TimerEvent& te);
  bool                    has_occupancy_octomap_changed_since_last_kdtree_calc_;
  std::shared_ptr<PCLMap> pcl_map_ = NULL;
  bool                    pcl_map_initalized;

  /* occupancy map */
  std::shared_ptr<octomap::OcTree> occupancy_octree_;
  ros::Time                        latest_occupancy_octree_receive_time_;


  /* topology map */
  std::mutex              mutex_segmap_;
  std::mutex              mutex_received_segmaps_;
  std::shared_ptr<SegMap> segmap_;
  float                   segmap_voxel_min_safe_distance_;
  std::shared_ptr<SegMap> test_segmap_;
  bool                    received_test_segmap_;

  std::mutex                 mutex_spheremap_;
  std::shared_ptr<SphereMap> spheremap_;
  std::mutex                 mutex_cached_spheremap_;
  std::shared_ptr<SphereMap> cached_spheremap_ = NULL;

  TopologyMappingSettings topology_mapping_settings_;

  std::vector<FrontierNode> frontier_nodes_ = {};
  void                      updateTopologyMapAtCurrentPosition();
  void generateFrontierNodes(std::shared_ptr<octomap::OcTree> occupancy_octree, bool search_whole_map = true, BoundingBox search_bbx = BoundingBox());

  /* frontier group calculation */
  std::vector<FrontierGroup>            frontier_groups_             = {};
  std::vector<std::vector<uint>>        frontier_cluster_indices_    = {};
  std::vector<float>                    frontier_cluster_sizes_      = {};
  std::vector<FrontierExplorationPoint> frontier_exploration_points_ = {};
  void calculateFrontierGroups(bool disciard_small_frontier_groups = true, bool discard_staging_area_groups = false, bool debug_print = false);
  void calculateFrontierGroups2(bool disciard_small_frontier_groups = true, bool discard_staging_area_groups = false, bool debug_print = false);
  void calculateFrontierGroupsForSegmapSending();
  std::vector<FrontierGroup> filterFrontierGroups(std::vector<FrontierGroup> groups1, float grouping_proj_threshold);

  /* sending topology information */
  ros::Publisher                       pub_segmap_data_;
  ros::Time                            last_segmap_send_time_;
  float                                segmap_sending_interval_;
  std::vector<std::shared_ptr<SegMap>> received_segmaps_;
  std::vector<std::vector<SegmapMsg>>  received_segmaps_fragment_buffers_;
  std::vector<uint>                    received_segmaps_latest_message_indices_;

  std::vector<ros::Publisher> topology_vis_publisher_list_ = {};
  uint                        segmap_sending_index_        = 0;
  int                         segmap_sending_max_data_bytes_;

  std::vector<std::string>             robot_names_list_ = {};
  std::map<std::string, unsigned long> robot_names_map_to_ids_;

  std::mutex                           segmap_msg_received_mutex_;
  std::string                          robot_name_;
  std::vector<ros::Subscriber>         topology_subscriber_list_ = {};
  void                                 callbackSegmapMsgReceived(const boost::shared_ptr<const SegmapMsg> msg, int robot_index);
  void                                 publishAllTopologyData();
  std::vector<std::shared_ptr<SegMap>> getReceivedSegmapsPointers();

  float getMaxSurfaceCoverageValOfReceivedSegmapsAtPos(octomap::point3d);

  /* VISITED SPACE MAPPING */
  std::shared_ptr<SphereMap> visited_positions_map_;

  std::mutex       visited_positions_map_mutex_;
  octomap::point3d last_added_visited_position_;
  bool             added_first_visited_position_ = false;
  ros::Time        last_visited_positions_update_time_;
  float            visited_positions_min_addition_dist_       = 0.5;
  float            visited_position_max_time_stayed_          = 5;    // will be overriden by planner
  float            visited_positions_zevling_dist_            = 0.2;  // will be overriden by planner
  float            visited_positions_dist_for_blocking_       = 0.2;  // will be overriden by planner
  float            visited_positions_dist_for_value_decrease_ = 0.4;  // will be overriden by planner

  float getVisitationValueOfPosition(octomap::point3d, std::shared_ptr<octomap::OcTree> occupancy_octree);

  /* float visited_positions_map_update_dist_ */


  /* visualization */
  void publishSegmentMarkers(std::shared_ptr<octomap::SegmentOcTree> segment_octree_);
  void publishSegmentMapMarkers(std::shared_ptr<octomap::SegmentOcTree> segment_octree_);
  void publishPCA(std::vector<octomap::point3d>, const ros::Time& rostime);
  void publishEllipsoidMarker(std::vector<octomap::point3d>, const ros::Time& rostime);
  void publishTopologyMapMarkers(const ros::Time& rostime);
  void publishFrontierMarkers(const ros::Time& rostime, std::vector<FrontierNode>* frontier_nodes);
  void publishAll(const ros::Time& rostime);

  /* class utilities */
  std::mutex                      frontier_border_node_keys_mutex_;
  std::vector<octomap::OcTreeKey> frontier_border_node_keys_;

  int getFrontierNodeKeys(std::shared_ptr<octomap::OcTree>& occupancy_octree, octomap::OcTreeKey start_key, uint8_t num_voxels,
                          std::vector<octomap::OcTreeKey>* res_keys);
};


}  // namespace spheremap_server

#endif
