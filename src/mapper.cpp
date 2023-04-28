#include <geometry_msgs/Point.h>
#include <octomap_msgs/Octomap.h>
#include <spheremap_server/mapper.h>
#include <spheremap_server/utility_functions.h>
/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

namespace spheremap_server
{

/* initialize() //{ */
void ExplorationMapper::initialize(ros::NodeHandle* nh) {
  /* mrs_lib::ParamLoader param_loader(*nh, "ExplorationMapper"); */


  tf_listener = std::shared_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf_buffer));

  /* parameters */
  bool use_own_planning_manager;
  nh->getParam("uav_name", uav_name_);
  nh->getParam("robot_name", robot_name_);
  nh->getParam("baselink_frame", baselink_frame_);
  nh->getParam("map_frame", map_frame_);
  /* param_loader.loadParam("artifact_frame", robots_shared_frame_); */
  nh->getParam("rate_topology_msg_buffer_emptying", rate_timer_empty_topology_msgs_buffer_);
  nh->getParam("spheremap_min_safe_dist", spheremap_min_safe_dist_);
  nh->getParam("spheremap_fully_safe_dist", spheremap_fully_safe_dist_);
  nh->getParam("enable_ltvmap_fragment_sending", enable_sending_ltvmap_fragments_);
  nh->getParam("enable_ltvmap_markers_sending", enable_sending_ltvmap_markers_);
  nh->getParam("max_graph_dist_for_spheremap_planning", max_graph_dist_for_spheremap_planning_);
  nh->getParam("spheremap_update_box_size", absolute_max_map_update_dist);
  nh->getParam("spheremap_update_box_size_hires", absolute_max_map_update_dist_highres);
  nh->getParam("spheremap_planning_safety_weight", spheremap_planning_safety_weight_);

  /* SENDING OF FACETMAP DATA */
  rate_timer_publish_markers_              = 1;
  rate_timer_update_segmap_                = 2;
  rate_timer_update_visited_positions_map_ = 2;
  rate_timer_calculate_kdtree_             = 1;
  rate_timer_rebuild_received_segmaps_     = 3;


  artifact_frame_ = uav_name_ + "/artifact_origin";


  nh->getParam("topology_mapping/enabled", topology_mapping_enabled_);
  if (topology_mapping_enabled_) {
    nh->getParam("topology_mapping/max_segment_radius", topology_mapping_settings_.segment_max_size_);
    nh->getParam("topology_mapping/compactness_delta", topology_mapping_settings_.compactness_delta_);
    nh->getParam("topology_mapping/convexity_threshold", topology_mapping_settings_.convexity_threshold);
    nh->getParam("topology_mapping/merging/enabled", topology_mapping_settings_.merging_enabled_);
    nh->getParam("topology_mapping/merging/max_segment_radius", topology_mapping_settings_.merged_segment_max_size_);
    nh->getParam("topology_mapping/merging/compactness_factor", topology_mapping_settings_.merged_compactness_factor_);
    nh->getParam("topology_mapping/merging/convexity_threshold", topology_mapping_settings_.merged_convexity_threshold_);

    nh->getParam("topology_mapping/num_max_grown", topology_mapping_settings_.num_max_grown);
    nh->getParam("topology_mapping/num_rays_for_growth", topology_mapping_settings_.num_rays_for_growth);
    nh->getParam("topology_mapping/growth_raydist", topology_mapping_settings_.growth_raydist);
    nh->getParam("topology_mapping/raycast_growing_safe_distance", topology_mapping_settings_.raycast_growing_safe_distance);
    nh->getParam("topology_mapping/num_expand_tries", topology_mapping_settings_.num_expand_tries);
    nh->getParam("topology_mapping/num_merge_tries", topology_mapping_settings_.num_merge_tries);
    nh->getParam("topology_mapping/voxel_min_safe_distance", topology_mapping_settings_.segmap_voxel_min_safe_distance_);
    nh->getParam("topology_mapping/segmap_default_depth_offset", topology_mapping_settings_.segmap_default_depth_offset_);
  }


  /* publishers */
  pub_frontier_clusters_    = nh->advertise<visualization_msgs::MarkerArray>("debug_spheremap_obstacle_kdtree", 1);
  pub_sent_segmap_          = nh->advertise<visualization_msgs::MarkerArray>("sent_ltvmap_visualization", 1);
  pub_spheremap_spheres_    = nh->advertise<visualization_msgs::MarkerArray>("spheremap_spheres", 10);
  pub_spheremap_points_     = nh->advertise<visualization_msgs::MarkerArray>("spheremap_graph", 10);
  pub_spheremap_topology_   = nh->advertise<visualization_msgs::MarkerArray>("spheremap_topology", 10);
  pub_spheremap_navigation_ = nh->advertise<visualization_msgs::MarkerArray>("spheremap_navigation", 10);
  pub_spheremap_debug_      = nh->advertise<visualization_msgs::MarkerArray>("spheremap_debug", 10);
  pub_markers_planning_     = nh->advertise<visualization_msgs::MarkerArray>("spheremap_planning_debug", 10);

  pub_visited_positions_map_ = nh->advertise<visualization_msgs::MarkerArray>("visited_positions_map", 10);

  /* subscribers */
  sub_odometry_                 = nh->subscribe("odom_uav_in", 1, &ExplorationMapper::callbackOdometryMessageReceived, this);
  sub_octomap_binary_const_ptr_ = nh->subscribe("octomap_binary_const_ptr_in", 1, &ExplorationMapper::callbackOctomapBinaryConstPtrReceived, this);
  sub_octomap_binary_           = nh->subscribe("octomap_binary_in", 1, &ExplorationMapper::callbackOctomapBinaryReceived, this);

  /* timers */
  timer_rebuild_received_segmaps_ =
      nh->createTimer(ros::Rate(rate_timer_rebuild_received_segmaps_), &ExplorationMapper::callbackTimerRebuildReceivedSegmaps, this);
  timer_update_visited_positions_map_ =
      nh->createTimer(ros::Rate(rate_timer_update_visited_positions_map_), &ExplorationMapper::callbackTimerUpdateVisitedPositionsMap, this);
  timer_publish_markers_ = nh->createTimer(ros::Rate(rate_timer_publish_markers_), &ExplorationMapper::callbackTimerPublishMarkers, this);
  timer_update_segmap_   = nh->createTimer(ros::Rate(rate_timer_update_segmap_), &ExplorationMapper::callbackTimerUpdateTopologyMap, this);
  timer_empty_topology_msgs_buffer_ =
      nh->createTimer(ros::Rate(rate_timer_empty_topology_msgs_buffer_), &ExplorationMapper::callbackTimerEmptyTopologyMsgsBuffer, this);
  timer_calculate_kdtree_ = nh->createTimer(ros::Rate(rate_timer_calculate_kdtree_), &ExplorationMapper::callbackTimerCalculateKdtree, this);

  /* topology information sending */
  pub_segmap_data_ = nh->advertise<SegmapMsg>("segmap_out", 1);

  std::vector<std::string> _robot_names_list;
  /* nh->getParam("robot_name_list", _robot_names_list); */
  /* for (size_t k = 0; k < _robot_names_list.size(); k++) { */
  /*   if (_robot_names_list[k] == robot_name_ || _robot_names_list[k] == "TEAMBASE") { */
  /*     continue; */
  /*   } */
  /*   robot_names_list_.push_back(_robot_names_list[k]); */
  /* } */

  // INIT RECEVING TOPICS FOR SEGMAPS
  segmap_rebuild_buffer                    = std::vector<std::vector<SegmapMsg>>(robot_names_list_.size(), std::vector<SegmapMsg>(0));
  segmap_rebuild_buffer_flags              = std::vector<bool>(robot_names_list_.size(), false);
  received_segmaps_                        = std::vector<std::shared_ptr<SegMap>>(robot_names_list_.size(), NULL);
  received_segmaps_fragment_buffers_       = std::vector<std::vector<SegmapMsg>>(robot_names_list_.size(), std::vector<SegmapMsg>(0));
  received_segmaps_latest_message_indices_ = std::vector<uint>(robot_names_list_.size(), 0);

  received_segmaps_num_rebuilt                  = std::vector<uint>(robot_names_list_.size(), 0);
  received_segmaps_num_unique_msg_ids_received_ = std::vector<uint>(robot_names_list_.size(), 0);

  topology_vis_publisher_list_ = {};
  std::string topic_name;

  for (uint i = 0; i < robot_names_list_.size(); i++) {
    /* robot_names_map_to_ids_[robot_names_list_[i]] = i; */
    topic_name = "/" + robot_names_list_[i] + "/" + "spheremap_server/topology_map";
    /* ping_topic_name                               = "/" + robot_names_list_[i] + "/ping_out"; */
    ROS_INFO("Subscribing to: %s", topic_name.c_str());
    topology_subscriber_list_.push_back(
        /* nh->subscribe<SegmapMsg>(topic_name.c_str(), 1000, */
        /*                                           boost::bind(&ExplorationMapper::callbackSegmapReceived, this, _1)));  // TODO: solve this issue */
        nh->subscribe<SegmapMsg>(topic_name.c_str(), 10,
                                 boost::bind(&ExplorationMapper::callbackSegmapMsgReceived, this, _1,
                                             i)));  // TODO: solve this issue

    topic_name = "/" + uav_name_ + "/" + "spheremap_server_vis/topology_maps_received/" + robot_names_list_[i];
    topology_vis_publisher_list_.push_back(nh->advertise<visualization_msgs::MarkerArray>(topic_name, 1));
    ROS_INFO("Publishing on: %s", topic_name.c_str());
  }

  /* topology */
  spheremap_ =
      std::shared_ptr<SphereMap>(new SphereMap(spheremap_min_safe_dist_, spheremap_fully_safe_dist_, topology_mapping_settings_, &staging_area_settings_));
  spheremap_->max_update_box_size_              = absolute_max_map_update_dist - 5;  // redundant
  spheremap_->spheremap_planning_safety_weight_ = spheremap_planning_safety_weight_;

  visited_positions_map_ =
      std::shared_ptr<SphereMap>(new SphereMap(spheremap_min_safe_dist_, spheremap_fully_safe_dist_, topology_mapping_settings_, &staging_area_settings_));

  test_segmap_ = NULL;

  pcl_map_initalized     = false;
  last_segmap_send_time_ = ros::Time::now();
  nh->getParam("ltvmap_sending_interval", segmap_sending_interval_);
  nh->getParam("ltvmap_sending_max_data_bytes", segmap_sending_max_data_bytes_);

  /* if (!param_loader.loadedSuccessfully()) { */
  /*   ROS_ERROR("[mapper]: Could not load all non-optional parameters. Shutting down."); */
  /*   ros::requestShutdown(); */
  /* } */

  /* services */
  spheremap_planning_srv_        = nh->advertiseService("GetSphereMapPathSrv", &ExplorationMapper::callbackGetSphereMapPathSrv, this);
  spheremap_planning_params_srv_ = nh->advertiseService("SetSafetyPlanningParams", &ExplorationMapper::callbackSetSafetyPlanningParams, this);

  is_initialized_ = true;
  ROS_INFO("mapper init done");
}
//}

/* CALLBACKS //{ */

/* bool callbackGetSphereMapPathSrv() //{ */
bool ExplorationMapper::callbackGetSphereMapPathSrv(GetSphereMapPathSrv::Request& req, GetSphereMapPathSrv::Response& resp) {
  resp.success = false;
  if (spheremap_ == NULL) {
    resp.message = "spheremap not initialized";
    return true;
  }

  /* TRANSFORM INPUT POINTS INTO SPHEREMAP FRAME */
  std::string start_frame = req.header.frame_id;
  std::string end_frame   = map_frame_;

  /* ROS_INFO("[callbackGetSphereMapPathSrv]: transforming from %s to %s", start_frame, end_frame); */

  octomap::point3d start_pos_spheremap_frame;
  octomap::point3d goal_pos_spheremap_frame;

  tf2::Transform request_to_spheremap_transform;
  try {
    geometry_msgs::TransformStamped transform_stamped = tf_buffer.lookupTransform(end_frame, start_frame, ros::Time(0));
    tf2::fromMsg(transform_stamped.transform, request_to_spheremap_transform);

    tf2::Vector3 startpose_vec(req.start.x, req.start.y, req.start.z);
    tf2::Vector3 goalpose_vec(req.goal.x, req.goal.y, req.goal.z);

    tf2::Vector3 transformed_start = request_to_spheremap_transform * startpose_vec;
    tf2::Vector3 transformed_goal  = request_to_spheremap_transform * goalpose_vec;

    start_pos_spheremap_frame = octomap::point3d(transformed_start.x(), transformed_start.y(), transformed_start.z());
    goal_pos_spheremap_frame  = octomap::point3d(transformed_goal.x(), transformed_goal.y(), transformed_goal.z());
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("[callbackGetSphereMapPathSrv]: Transform failed: %s", ex.what());
    resp.message = "transform failed";
    return true;
  }

  // NOW TRY FINDING PATH IN SPHEREMAP
  std::optional<std::vector<SphereMapPath>> path_comp_res;
  {
    std::scoped_lock lock(mutex_spheremap_);

    // TODO this to parameter
    path_comp_res =
        spheremap_->computePathsToGoalsWithConnectingToGraph(start_pos_spheremap_frame, max_graph_dist_for_spheremap_planning_, {goal_pos_spheremap_frame},
                                                             max_graph_dist_for_spheremap_planning_, true, !req.ignore_precomputed_paths);
  }
  if (!path_comp_res || path_comp_res.value().size() == 0 || path_comp_res.value()[0].positions.size() == 0) {
    resp.message = "could not find path!";
    return true;
  }
  SphereMapPath respath        = path_comp_res.value()[0];
  unsigned int  pathlen        = respath.positions.size();
  resp.path                    = {};
  resp.path_obstacle_distances = {};

  ROS_INFO("[callbackGetSphereMapPathSrv]: DIST FROM GIVEN START AND END TO PATH: %f [m], %f [m]", (respath.positions[0] - start_pos_spheremap_frame).norm(),
           (respath.positions[pathlen - 1] - goal_pos_spheremap_frame).norm());

  // TRANSFORM PATH BACK INTO FRAME OF REQUESTED START AND GOAL
  tf2::Transform spheremap_to_request_transform = request_to_spheremap_transform.inverse();

  try {
    for (int i = 0; i < pathlen; i++) {
      tf2::Vector3 pointtotransform(respath.positions[i].x(), respath.positions[i].y(), respath.positions[i].z());
      tf2::Vector3 transformed_point = spheremap_to_request_transform * pointtotransform;

      geometry_msgs::Point respoint;
      respoint.x = transformed_point.x();
      respoint.y = transformed_point.y();
      respoint.z = transformed_point.z();
      resp.path.push_back(respoint);
      resp.path_obstacle_distances.push_back(respath.obstacle_dists[i]);
    }
  }
  catch (tf2::TransformException& ex) {
    resp.message = "transform back failed";
    ROS_WARN("[callbackGetSphereMapPathSrv]: Transform back failed: %s", ex.what());
    return true;
  }

  /* VISUALIZE */
  respath.original_start                          = start_pos_spheremap_frame;
  respath.original_goal                           = goal_pos_spheremap_frame;
  std::vector<visualization_msgs::Marker> markers = getSphereMapPathsMarkers({respath});
  publishMarkers(markers, &pub_markers_planning_, "srv_computed_paths", map_frame_, 0, -1);

  /* RETURN */
  resp.message = "success!";
  resp.success = true;
  return true;
}
//}

/* bool callbackSetSafetyPlanningParams() //{ */
bool ExplorationMapper::callbackSetSafetyPlanningParams(SetSafetyPlanningParamsSrv::Request& req, SetSafetyPlanningParamsSrv::Response& resp) {
  resp.success = false;
  std::scoped_lock lock(mutex_spheremap_);

  if (spheremap_ == NULL) {
    resp.message = "spheremap not initialized!";
    return true;
  }

  spheremap_->spheremap_planning_safety_weight_ = req.spheremap_planning_safety_weight;
  /* spheremap_->spheremap_planning_safety_bias_ = req.spheremap_planning_safety_bias; */  // TODO bias is not supported in cached paths computations as of now
  spheremap_->spheremap_planning_safety_bias_ = 0;

  /* RETURN */
  resp.message = "success!";
  resp.success = true;
  return true;
}
//}

/* callbackOdometryMessageReceived() //{ */
void ExplorationMapper::callbackOdometryMessageReceived(const nav_msgs::Odometry msg) {
  geometry_msgs::PoseStamped pose_tmp;
  /* std::string                start_frame = baselink_frame_; */
  std::string start_frame  = msg.header.frame_id;
  std::string end_frame    = map_frame_;
  pose_tmp.header.frame_id = start_frame;
  pose_tmp.header.stamp    = ros::Time::now();
  pose_tmp.pose            = msg.pose.pose;
  /* pose_tmp.pose.position.x               = 0; */
  /* pose_tmp.pose.position.y               = 0; */
  /* pose_tmp.pose.position.z               = 0; */
  try {
    /* tf2_ros::Buffer                 tf_buffer; */
    /* tf2_ros::TransformListener      tf_listener(tf_buffer); */
    /* ROS_INFO("Trying to transform from %s to %s ", start_frame.c_str(), end_frame.c_str()); */
    geometry_msgs::TransformStamped transform_stamped = tf_buffer.lookupTransform(end_frame, start_frame, ros::Time(0));
    tf2::Transform                  transform;
    tf2::fromMsg(transform_stamped.transform, transform);
    tf2::Vector3 position(pose_tmp.pose.position.x, pose_tmp.pose.position.y, pose_tmp.pose.position.z);
    tf2::Vector3 transformed_position = transform * position;
    pose_tmp.pose.position.x          = transformed_position.x();
    pose_tmp.pose.position.y          = transformed_position.y();
    pose_tmp.pose.position.z          = transformed_position.z();
    pose_tmp.header.frame_id          = end_frame;
    /* ROS_INFO("Transformation from %s to %s successful", start_frame.c_str(), end_frame.c_str()); */

    has_odometry_     = true;
    current_position_ = octomap::point3d(pose_tmp.pose.position.x, pose_tmp.pose.position.y, pose_tmp.pose.position.z);
    /* ROS_INFO("current_position: %f, %f, %f", current_position_.x(), current_position_.y(), current_position_.z()); */
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("[Odometry]: Transform of odometry msg failed: %s", ex.what());
  }
}
//}

/* callbackOctomapBinaryReceived() //{ */
void ExplorationMapper::callbackOctomapBinaryReceived(const octomap_msgs::Octomap octomap_msg) {
  /* setOctomapMsgPtr(octomap_msg); */
  /* ROS_INFO("got an octomap msg!"); */

  std::scoped_lock lock(octomap_mutex_);
  latest_octomap_msg                 = octomap_msg;
  has_new_occupancy_octomap_msg_     = true;
  octomap_msg_is_in_constptr_format_ = false;
}
//}

/* callbackOctomapBinaryConstPtrReceived() //{ */
void ExplorationMapper::callbackOctomapBinaryConstPtrReceived(const octomap_msgs::OctomapConstPtr octomap_msg) {
  setOctomapMsgPtr(octomap_msg);
  octomap_msg_is_in_constptr_format_ = true;
}
//}

/* callbackSegmapMsgReceived() //{ */
void ExplorationMapper::callbackSegmapMsgReceived(const boost::shared_ptr<const SegmapMsg> msg, int robot_index) {
  ROS_INFO_THROTTLE(0.5, "[TopologySending(throttled msg)]: SEGMAP MSG RECEIVED FROM ROBOT %d", robot_index);
  ROS_INFO_THROTTLE(0.5, "[TopologySending(throttled msg)]: SEGMAP MSG fragment index: %d / %d, message index: %d", msg->fragment_index,
                    msg->num_total_fragments, msg->message_index);

  std::scoped_lock lock(segmap_msg_received_mutex_);

  if (robot_index > (int)(received_segmaps_.size() - 1)) {
    ROS_WARN("[TopologySending]: index of robot is higher than the size of received segmapss array, this should not happen");
    return;
  }
  if (msg->message_index != received_segmaps_latest_message_indices_[robot_index]) {
    if (msg->message_index > received_segmaps_latest_message_indices_[robot_index]) {
      /* ROS_INFO("[TopologySending]: a fragment of a newer segmap received, clearing fragment buffer"); */
      received_segmaps_fragment_buffers_[robot_index].clear();
      received_segmaps_latest_message_indices_[robot_index] = msg->message_index;
      received_segmaps_num_unique_msg_ids_received_[robot_index]++;
    } else {
      /* DISCARD OLDER FRAGMENTS */
      return;
    }
  }
  uint fragment_index  = msg->fragment_index;
  bool is_fragment_new = true;
  for (uint i = 0; i < received_segmaps_fragment_buffers_[robot_index].size(); i++) {
    if (received_segmaps_fragment_buffers_[robot_index][i].fragment_index == fragment_index) {
      is_fragment_new = false;
      break;
    }
  }
  if (!is_fragment_new) {
    ROS_WARN_THROTTLE(1, "[TopologySending]: received fragment is duplicit");
    return;
  }
  received_segmaps_fragment_buffers_[robot_index].push_back(*msg);

  if (received_segmaps_fragment_buffers_[robot_index].size() == msg->num_total_fragments) {
    /* PUT FRAGMENTS TO SIDE BUFFER AND RAISE FLAG FOR SEGMAP FACTORY TO REBUILD IT */
    {
      /* ROS_INFO("[TopologySending]: all fragments of message index %d received, rebuilding segmap", msg->message_index); */
      std::scoped_lock lock(mutex_segmap_rebuild_buffer);
      segmap_rebuild_buffer[robot_index]       = received_segmaps_fragment_buffers_[robot_index];
      segmap_rebuild_buffer_flags[robot_index] = true;
    }
  }
}
//}

// TODO move this
/* publishAllTopologyData() //{ */
void ExplorationMapper::publishAllTopologyData() {
  /* ROS_INFO("[TopologySending]: extracting and sending LTV-Map as markers"); */
  if (spheremap_ == NULL) {
    ROS_WARN("[TopologySending]: cannot send spheremap as msgs, it is not initialized!");
    return;
  }

  std::vector<SegmapMsg> msgs = spheremap_->getSegmapMsgs(segmap_sending_index_, segmap_sending_max_data_bytes_, &tf_buffer, map_frame_, robots_shared_frame_);
  if (msgs.empty()) {
    ROS_ERROR("[TopologySending]: converting to msgs failed");
    return;
  }

  if (enable_sending_ltvmap_fragments_) {
    ROS_INFO("[TopologySending]: publishing %lu LTV-Map message fragments", msgs.size());
    for (uint i = 0; i < msgs.size(); i++) {
      ROS_INFO("[TopologySending]: adding segmap fragment to buffer %d", msgs[i].fragment_index);
      topology_messages_buffer_.push_back(msgs[i]);
    }
    segmap_sending_index_++;
  }

  /* NOW REBUILD IT AND SEND MARKERS*/
  if (enable_sending_ltvmap_markers_) {
    ROS_INFO("[TopologySending]: publishing LTV-Map markers");
    bool                    use_shared_frame   = false;
    std::shared_ptr<SegMap> transformed_segmap = SegMap::convertFragmentMsgsToSegmap(msgs, &tf_buffer, map_frame_, robots_shared_frame_, use_shared_frame);
    if (transformed_segmap != NULL) {
      last_sent_segmap_rebulit                        = transformed_segmap;
      std::vector<visualization_msgs::Marker> markers = getSegmapBlockMarkers(transformed_segmap, true);
      publishMarkers(markers, &pub_sent_segmap_, "ltv_map", map_frame_, 0, -1);
    }
  }
}
//}

/* getReceivedSegmapsPointers() //{ */
std::vector<std::shared_ptr<SegMap>> ExplorationMapper::getReceivedSegmapsPointers() {
  std::scoped_lock                     lock(mutex_received_segmaps_);
  std::vector<std::shared_ptr<SegMap>> res = {};
  for (uint i = 0; i < received_segmaps_.size(); i++) {
    if (received_segmaps_[i] != NULL && robot_names_list_[i] != robot_name_) {
      res.push_back(received_segmaps_[i]);
    }
  }
  return res;
}
//}

/* setOctomapMsgPtr() //{ */
void ExplorationMapper::setOctomapMsgPtr(const octomap_msgs::OctomapConstPtr msg_ptr) {
  std::scoped_lock lock(octomap_mutex_);
  octomap_msg_ptr_               = msg_ptr;
  has_new_occupancy_octomap_msg_ = true;
}
//}

//}

/* float getVisitationValueOfPosition() //{ */
float ExplorationMapper::getVisitationValueOfPosition(octomap::point3d cpos, std::shared_ptr<octomap::OcTree> occupancy_octree) {
  if (!is_initialized_ || !has_odometry_ || !has_occupancy_octomap_) {
    return 0;
  }
  float            blockdist2 = pow(visited_positions_dist_for_blocking_, 2);
  float            devaldist2 = pow(visited_positions_dist_for_value_decrease_, 2);
  std::scoped_lock lock(visited_positions_map_mutex_);

  float maxvisitval = 0;

  /* IF TIME OVER LIMIT AND IN BLOCKING AREA, RETURN BLOCKING VALUE */
  /* ELSE DEVALUATE LINEARLY WITH DIST */

  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys =
      visited_positions_map_->getMaxSearchBBXBorderKeys(cpos, visited_positions_dist_for_value_decrease_);
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = visited_positions_map_->nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second);
       it != visited_positions_map_->nodes->end_leafs_bbx(); it++) {
    float dist2 = (it->valuePtr()->pos - cpos).norm_sq();
    if (dist2 < blockdist2) {
      return 2;
    }

    if (dist2 < devaldist2) {
      float dist         = sqrt(dist2);
      float possible_val = 1 - (dist / visited_positions_dist_for_value_decrease_);
      if (maxvisitval < possible_val) {
        maxvisitval = possible_val;
      }
    }
  }

  return maxvisitval;
}
//}

// TIMERS //{

/* void       callbackTimerRebuildReceivedSegmaps() //{ */
void ExplorationMapper::callbackTimerRebuildReceivedSegmaps(const ros::TimerEvent& te) {
  std::scoped_lock lock(mutex_received_segmaps_);

  for (uint i = 0; i < received_segmaps_.size(); i++) {
    std::vector<SegmapMsg> tmp_rebuild_buf;
    std::vector<SegmapMsg> rebuild_buf;
    {
      std::scoped_lock lock(mutex_segmap_rebuild_buffer);
      if (!segmap_rebuild_buffer_flags[i]) {
        continue;
      }
      tmp_rebuild_buf                = segmap_rebuild_buffer[i];
      segmap_rebuild_buffer_flags[i] = false;
    }

    ROS_INFO("[ExplorationMapper]: rebuilding segmap for robot of id: %u", i);
    /* ORDER BUFFER */
    for (uint i = 0; i < tmp_rebuild_buf.size(); i++) {
      bool found_fragment = false;
      for (uint j = 0; j < tmp_rebuild_buf.size(); j++) {
        if (tmp_rebuild_buf[j].fragment_index == i) {
          found_fragment = true;
          rebuild_buf.push_back(tmp_rebuild_buf[j]);
        }
      }
      if (!found_fragment) {
        ROS_ERROR("[ExplorationMapper]: cannot rebuild map, fragments have wrong indices");
        for (uint j = 0; j < tmp_rebuild_buf.size(); j++) {
          ROS_INFO("frag index: %u / %u, msg index: %u", tmp_rebuild_buf[j].fragment_index, tmp_rebuild_buf[j].num_total_fragments,
                   tmp_rebuild_buf[j].message_index);
        }
        return;
      }
    }

    /* BUILD SEGMAP FROM FRAGMENTS */
    std::shared_ptr<SegMap> transformed_segmap = SegMap::convertFragmentMsgsToSegmap(rebuild_buf, &tf_buffer, map_frame_, robots_shared_frame_);
    received_segmaps_[i]                       = transformed_segmap;

    /* UPDATE OTHER SEGMAPS WITH THIS SEGMAP */
    for (uint i = 0; i < received_segmaps_.size(); i++) {
      std::vector<std::shared_ptr<SegMap>> tmp_segmaps = {};
      if (received_segmaps_[i] == NULL) {
        continue;
      }
      if (last_sent_segmap_rebulit != NULL) {
        tmp_segmaps.push_back(last_sent_segmap_rebulit);
      }
      for (uint j = 0; j < received_segmaps_.size(); j++) {
        if (received_segmaps_[j] == NULL) {
          continue;
        }
        if (j != i) {
          tmp_segmaps.push_back(received_segmaps_[j]);
        }
      }
      /* ROS_INFO("[TopologySending]: updating received map with other maps"); */
      /* received_segmaps_[i]->updateExplorednessProbabilitiesBasedOnOtherSegmaps(tmp_segmaps); */
    }
    received_segmaps_num_rebuilt[i]++;
  }

  /* PRINT STATISTICS */
  ros::Time time_now = ros::Time::now();
  if ((time_now - last_rebuilding_stats_print_time_).toSec() > 5) {
    last_rebuilding_stats_print_time_ = time_now;
    ROS_INFO("[TopologySending]: segmap sending stats:");
    for (uint i = 0; i < received_segmaps_.size(); i++) {
      ROS_INFO("[TopologySending]: robot id: %u, unique_msg_ids_received: %u, maps_rebuilt: %u", i, received_segmaps_num_unique_msg_ids_received_[i],
               received_segmaps_num_rebuilt[i]);
    }
  }
}
//}

/* callbackTimerUpdateVisitedPositionsMap() //{ */
void ExplorationMapper::callbackTimerUpdateVisitedPositionsMap(const ros::TimerEvent& te) {
  if (!is_initialized_ || !has_odometry_ || !has_occupancy_octomap_) {
    return;
  }

  std::shared_ptr<octomap::OcTree> occupancy_octree = getOccupancyOcTreeSharedPtr();
  std::shared_ptr<PCLMap>          pcl_map          = getPCLMapSharedPtr();


  std::scoped_lock   lock(visited_positions_map_mutex_);
  octomap::point3d   cpos          = current_position_;
  octomap::OcTreeKey ckey          = visited_positions_map_->nodes->coordToKey(cpos, visited_positions_map_->base_depth_);
  float              current_odist = fmin(getObstacleDist(cpos, pcl_map), visited_positions_map_->enforced_max_sphere_radius);
  if (!added_first_visited_position_) {  // GET FIRST POSITION IN AIR
    if (current_odist > 0.2) {
      octomap::SphereMapNodeData first_data(cpos, current_odist);
      visited_positions_map_->addNode(first_data);

      last_added_visited_position_        = cpos;
      last_visited_positions_update_time_ = ros::Time::now();
      added_first_visited_position_       = true;
    }
    return;
  }

  ros::Time time_now                  = ros::Time::now();
  float     stayed_time               = (time_now - last_visited_positions_update_time_).toSec();
  last_visited_positions_update_time_ = time_now;

  /* IF CURRENT POS IS UNCONNECTED OR CONNECTED AND NOT IN LAST SPHERE, ADD IT */
  float min_addition_dist  = visited_positions_min_addition_dist_;
  float min_addition_dist2 = pow(min_addition_dist, 2);

  bool should_add_pos = true;

  bool                            is_too_close = false;
  std::vector<octomap::OcTreeKey> connectable_keys;
  std::vector<octomap::OcTreeKey> nodes_that_robot_is_inside_now_keys;

  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys =
      visited_positions_map_->getMaxSearchBBXBorderKeys(cpos, visited_positions_min_addition_dist_ * 5);
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = visited_positions_map_->nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second);
       it != visited_positions_map_->nodes->end_leafs_bbx(); it++) {
    float dist2 = (it->valuePtr()->pos - cpos).norm_sq();
    if (visited_positions_map_->areConnectable(dist2, it->valuePtr()->radius, current_odist)) {
      connectable_keys.push_back(it.getKey());
    }

    float dist = sqrt(dist2);
    if (dist < visited_positions_zevling_dist_) {
      nodes_that_robot_is_inside_now_keys.push_back(it.getKey());
    }

    if (dist < visited_positions_min_addition_dist_) {
      is_too_close = true;
      break;
    }
  }

  /* UPDATE STAYED TIME OF NODES */
  for (uint i = 0; i < nodes_that_robot_is_inside_now_keys.size(); i++) {
    visited_positions_map_->nodes->search(nodes_that_robot_is_inside_now_keys[i])->addTimeStayed(stayed_time);
  }

  /* DONT ADD NODE IF TOO CLOSE TO OTHERS */
  if (is_too_close || visited_positions_map_->nodes->search(cpos, visited_positions_map_->base_safe_dist) != NULL) {
    return;
  }

  /* FORCE CONNECT TO LAST POSITION IF CURRENT POSIITON SOMEHOW MOVED TOO FAR */
  if (connectable_keys.empty()) {
    ROS_WARN("[VisitedMapUpdate]: connectable keys empty");
    octomap::OcTreeKey backup_key = visited_positions_map_->nodes->coordToKey(last_added_visited_position_, visited_positions_map_->base_depth_);
    connectable_keys.push_back(backup_key);

    auto node_ptr = visited_positions_map_->nodes->search(backup_key, visited_positions_map_->base_depth_);
    if (node_ptr == NULL) {
      ROS_ERROR("[VisitedMapUpdate]: last added position appears to not be in graph!");
    }
  }

  /* ADD NODE*/
  octomap::SphereMapNodeData node_data(cpos, current_odist);
  node_data.connected_keys = connectable_keys;
  visited_positions_map_->addNode(node_data);
  last_added_visited_position_ = cpos;

  /* CONNECT TO OTHERS */
  for (uint i = 0; i < connectable_keys.size(); i++) {
    auto node_ptr = visited_positions_map_->nodes->search(connectable_keys[i], visited_positions_map_->base_depth_);
    if (node_ptr == NULL) {
      ROS_ERROR("[VisitedMapUpdate]: cannot connect to position, node not found!!!!");
    }
    node_ptr->valuePtr()->connected_keys.push_back(ckey);
  }
}
//}

/* callbackTimerPublishMarkers() //{ */
void ExplorationMapper::callbackTimerPublishMarkers(const ros::TimerEvent& te) {
  if (!is_initialized_ || !has_odometry_ || !has_occupancy_octomap_) {
    return;
  }
  std::vector<visualization_msgs::Marker> markers;

  /* PUBLISH RECEIVED SEGMAPS' MARKERS */

  {
    std::scoped_lock lock(mutex_received_segmaps_);
    for (uint i = 0; i < received_segmaps_.size(); i++) {
      if (received_segmaps_[i] == NULL) {
        continue;
      }
      if (topology_vis_publisher_list_[i].getNumSubscribers() > 0) {
        publishMarkers(getSegmapBlockMarkers(received_segmaps_[i], true), &topology_vis_publisher_list_[i], "segmap_received", map_frame_, 0, -1);
      }
    }
  }

  /* PUBLISH SPHEREMAP MARKERS */
  if (spheremap_ != NULL) {
    mutex_spheremap_.lock();

    if (pub_spheremap_spheres_.getNumSubscribers() > 0) {
      markers = getSpheremapMarkers(current_position_, 20, spheremap_);
      publishMarkers(markers, &pub_spheremap_spheres_, "spheremap", map_frame_);
    }

    if (pub_spheremap_points_.getNumSubscribers() > 0) {
      markers = getSpheremapPointMarkers(current_position_, 20, spheremap_);
      publishMarkers(markers, &pub_spheremap_points_, "spheremap_graph", map_frame_, 0, -1);
    }

    if (pub_spheremap_topology_.getNumSubscribers() > 0) {
      markers = getSpheremapSegmentMarkers(spheremap_);
      publishMarkers(markers, &pub_spheremap_topology_, "spheremap_topology", map_frame_, 0, -1);
    }

    if (pub_spheremap_navigation_.getNumSubscribers() > 0) {
      markers = getSpheremapNavigationMarkers(spheremap_);
      publishMarkers(markers, &pub_spheremap_navigation_, "spheremap_navigation", map_frame_);
    }

    if (pub_spheremap_debug_.getNumSubscribers() > 0) {
      markers = getSpheremapDebugMarkers(spheremap_);
      publishMarkers(markers, &pub_spheremap_debug_, "spheremap_debug", map_frame_, 0, -1);
    }

    mutex_spheremap_.unlock();
  }

  /* PUBLISH VISITED POSITIONS MAP */
  if (visited_positions_map_ != NULL && pub_visited_positions_map_.getNumSubscribers() > 0) {
    markers = getSpheremapPointMarkers(current_position_, 500, visited_positions_map_, true, visited_position_max_time_stayed_);
    publishMarkers(markers, &pub_visited_positions_map_, "visited_positions_map", map_frame_, 0, -1);
  }
}
//}

/* callbackTimerUpdateTopologyMap() //{ */
void ExplorationMapper::callbackTimerUpdateTopologyMap(const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }
  if (!topology_mapping_enabled_) {
    ROS_WARN_THROTTLE(10, "topology mapping is disabled");
    return;
  }
  if (!has_odometry_) {
    ROS_WARN_THROTTLE(1, "[ExplorationMapper]: Odometry not received, cannot start topology mapping");
    return;
  }
  if (!has_occupancy_octomap_ || !pcl_map_initalized) {
    ROS_WARN_THROTTLE(1, "[ExplorationMapper]: OccupancyOctomap not received and kdtree not initialized, cannot start topology mapping");
    return;
  }

  /* UPDATE CURRENT MAPS WITH OWN INFO */
  std::scoped_lock lock(mutex_received_segmaps_);
  if (last_sent_segmap_rebulit != NULL) {
    for (uint i = 0; i < received_segmaps_.size(); i++) {
      ROS_INFO("[TopologySending]: updating received map %u with own map", i);
      if (received_segmaps_[i] != NULL) {
        mutex_spheremap_.lock();
        std::vector<std::shared_ptr<SegMap>> tmp_segmaps = {last_sent_segmap_rebulit};
        received_segmaps_[i]->updateExplorednessProbabilitiesBasedOnOtherSegmaps(tmp_segmaps);
        mutex_spheremap_.unlock();
      }
    }
  }

  // SEGMAP UPDATE HANDLES THE MUTEX ON ITS OWN
  mutex_spheremap_.lock();
  /* spheremap_->update(current_position_, current_heading_, getOccupancyOcTreeSharedPtr(), getPCLMapSharedPtr()); */
  /* mutex_cached_spheremap_.lock(); */
  /* std::shared_ptr<SphereMap> new_spheremap = std::shared_ptr<SphereMap>(new SphereMap()); */
  /* *new_spheremap                           = (*spheremap_); */
  /* cached_spheremap_                        = new_spheremap; */
  /* mutex_cached_spheremap_.unlock(); */
  if (!navigation_priority_flag_) {
    /* UPDATE MAX UPDATE BOX SIZE */
    spheremap_->max_update_box_size_ = hires_mode_ ? absolute_max_map_update_dist_highres : absolute_max_map_update_dist;
    spheremap_->update(current_position_, current_heading_, getOccupancyOcTreeSharedPtr(), getPCLMapSharedPtr());
  }
  mutex_spheremap_.unlock();

  mutex_spheremap_.lock();
  /* if (spheremap_->segments.size() > 2 && spheremap_->computed_cached_frontiers_atleast_once_) { */
  if (spheremap_->segments.size() > 1) {
    if ((ros::Time::now() - last_segmap_send_time_).toSec() > segmap_sending_interval_) {
      last_segmap_send_time_ = ros::Time::now();
      publishAllTopologyData();
    }
  }
  mutex_spheremap_.unlock();
}
//}

/* callbackTimerEmptyTopologyMsgsBuffer() //{ */
void ExplorationMapper::callbackTimerEmptyTopologyMsgsBuffer(const ros::TimerEvent& te) {
  if (!topology_messages_buffer_.empty()) {
    ROS_INFO("[TopologyMsgsBufferEmptying]: sending segmap fragment of index:%d / %d, msg index:%d", topology_messages_buffer_[0].fragment_index,
             topology_messages_buffer_[0].num_total_fragments, topology_messages_buffer_[0].message_index);
    pub_segmap_data_.publish(topology_messages_buffer_[0]);
    topology_messages_buffer_.erase(topology_messages_buffer_.begin());
  }
}
//}

/* callbackTimerCalculateKdtree()  //{ */
void ExplorationMapper::callbackTimerCalculateKdtree(const ros::TimerEvent& te) {
  if (!has_new_occupancy_octomap_msg_) {
    return;
  }

  octomap_mutex_.lock();
  octomap::AbstractOcTree* tree;
  if (octomap_msg_is_in_constptr_format_) {
    tree = octomap_msgs::binaryMsgToMap(*octomap_msg_ptr_);
  } else {
    tree = octomap_msgs::binaryMsgToMap(latest_octomap_msg);
  }
  octomap_mutex_.unlock();
  has_new_occupancy_octomap_msg_ = false;


  ros::WallTime start_, end_;
  start_ = ros::WallTime::now();
  double execution_time;

  std::shared_ptr<octomap::OcTree> occupancy_octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree));

  /* ROS_INFO("[SphereMap-kDtree]: occupancy octree resolution: %f", occupancy_octree->getResolution()); */
  if (occupancy_octree->getResolution() < 0.18) {
    hires_mode_ = true;
    ROS_INFO("[SphereMap-kDtree]: High resolution octomap detected - only doing spheremap update in smaller bbx of %f m instead of %f m",
             absolute_max_map_update_dist_highres, absolute_max_map_update_dist);
  } else {
    hires_mode_ = false;
  }
  BoundingBox generation_bbx(hires_mode_ ? absolute_max_map_update_dist_highres + 15 : absolute_max_map_update_dist + 15, current_position_);

  occupancy_octree_ptr_mutex_.lock();
  std::vector<pcl::PointXYZ> pcl_points = spheremap_server::octomapToPointcloud(occupancy_octree, generation_bbx);
  /* ROS_INFO("[SphereMap-kDtree]: kdtree has %lu points", pcl_points.size()); */
  occupancy_octree_ptr_mutex_.unlock();

  end_           = ros::WallTime::now();
  execution_time = (end_ - start_).toSec();
  /* ROS_INFO("[SphereMap-kDtree]: octomap conversion took %f seconds", execution_time); */
  start_ = ros::WallTime::now();

  std::shared_ptr<PCLMap> tmp_ptr = std::make_shared<PCLMap>();

  generateFrontierNodes(occupancy_octree);

  /* DEBUG DRAW KDTREE OCCUPIED MARKERS */
  {
    std::vector<visualization_msgs::Marker> markers = getPointsMarkers(pcl_points, octomap::point3d(0, 0, 1), 0.25);
    publishMarkers(markers, &pub_frontier_clusters_, "obstacle_kdtree", map_frame_);
  }

  /* ADD FRONTIER POSITIONS TO KDTREE */
  std::scoped_lock lock(frontier_border_node_keys_mutex_);
  octomap::point3d octopoint;
  pcl::PointXYZ    pclpoint;
  for (uint i = 0; i < frontier_border_node_keys_.size(); i++) {
    octopoint  = occupancy_octree->keyToCoord(frontier_border_node_keys_[i], 16);
    pclpoint.x = octopoint.x();
    pclpoint.y = octopoint.y();
    pclpoint.z = octopoint.z();
    pcl_points.push_back(pclpoint);
  }

  /* DEBUG DRAW KDTREE FRONTIER MARKERS */
  {
    std::vector<visualization_msgs::Marker> markers = getPointsMarkers(pcl_points, octomap::point3d(0, 1, 0), 0.1);
    publishMarkers(markers, &pub_frontier_clusters_, "obstacle_kdtree_with_frontiers", map_frame_);
  }

  end_           = ros::WallTime::now();
  execution_time = (end_ - start_).toSec();
  /* ROS_INFO("[SphereMap-kDtree]: frontier addition to pcl took %f seconds", execution_time); */
  start_ = ros::WallTime::now();

  /* INIT KDTREE SEARCH */
  pcl::PointCloud<pcl::PointXYZ>::Ptr simulated_pointcloud = PCLMap::pclVectorToPointcloud(pcl_points);

  end_           = ros::WallTime::now();
  execution_time = (end_ - start_).toSec();
  /* ROS_INFO("[SphereMap-kDtree]: conversion to  pcl took %f seconds", execution_time); */
  start_ = ros::WallTime::now();

  tmp_ptr->initKDTreeSearch(simulated_pointcloud);


  /* FIRST SET PCLMAP */
  pcl_map_ptr_mutex_.lock();
  pcl_map_           = tmp_ptr;
  pcl_map_initalized = true;
  pcl_map_ptr_mutex_.unlock();

  /* THEN SET OCCUPANCY OCTREE */
  occupancy_octree_ptr_mutex_.lock();
  occupancy_octree_      = occupancy_octree;
  has_occupancy_octomap_ = true;
  occupancy_octree_ptr_mutex_.unlock();

  end_           = ros::WallTime::now();
  execution_time = (end_ - start_).toSec();
  ROS_INFO("[SphereMap-kDtree]: obstacle kdtree creation took %f ms", execution_time * 1000);
  start_ = ros::WallTime::now();
}
//}

//}

/* SURFACE MAPPING //{ */

/* int getFrontierNodeKeys() //{ */
int ExplorationMapper::getFrontierNodeKeys(std::shared_ptr<octomap::OcTree>& occupancy_octree, octomap::OcTreeKey start_key, uint8_t num_voxels,
                                           std::vector<octomap::OcTreeKey>* res_keys) {

  int                num_added = 0;
  octomap::OcTreeKey iterator_key;
  int                end_key_x = start_key[0] + num_voxels;
  int                end_key_y = start_key[1] + num_voxels;
  int                end_key_z = start_key[2] + num_voxels;
  for (int x = start_key[0]; x < end_key_x; x++) {
    for (int y = start_key[1]; y < end_key_y; y++) {
      for (int z = start_key[2]; z < end_key_z; z++) {
        iterator_key[0]              = x;
        iterator_key[1]              = y;
        iterator_key[2]              = z;
        octomap::OcTreeNode* oc_node = occupancy_octree->search(iterator_key);
        if (oc_node == NULL) {
          bool is_frontier = false;

          for (int i = 0; i < 6; i++) {
            iterator_key[0] = x + neighborhood_6_dirs[i][0];
            iterator_key[1] = y + neighborhood_6_dirs[i][1];
            iterator_key[2] = z + neighborhood_6_dirs[i][2];
            oc_node         = occupancy_octree->search(iterator_key);
            if (oc_node != NULL && !occupancy_octree->isNodeOccupied(oc_node)) {
              is_frontier = true;
              break;
            }
          }

          if (is_frontier) {
            res_keys->push_back(iterator_key);
            num_added++;
          }
        }
        /* if (oc_node != NULL && !occupancy_octree->isNodeOccupied(oc_node)) { */
        /*   for (int i = 0; i < 6; i++) { */
        /*     iterator_key[0] = x + neighborhood_6_dirs[i][0]; */
        /*     iterator_key[1] = y + neighborhood_6_dirs[i][1]; */
        /*     iterator_key[2] = z + neighborhood_6_dirs[i][2]; */
        /*     oc_node         = occupancy_octree->search(iterator_key); */
        /*     if (oc_node == NULL) { */
        /*       res_keys->push_back(iterator_key); */
        /*       num_added++; */
        /*     } */
        /*   } */
        /* } */
      }
    }
  }
  return num_added;
}
//}

//}

/* TOPOLOGY MAPPING //{*/

/* generateFrontierNodes() //{ */
void ExplorationMapper::generateFrontierNodes(std::shared_ptr<octomap::OcTree> occupancy_octree, bool search_whole_map, BoundingBox search_bbx) {
  int                key_step                                = 4;
  int                search_depth                            = 14;
  float              occupancy_octree_res                    = occupancy_octree->getResolution();
  float              first_node_in_normal_calculation_offset = occupancy_octree_res * (0.8 / 2.0 - 0.5);
  octomap::OcTreeKey start_key, end_key;

  BoundingBox      bbx;
  octomap::point3d bbx_start_pos, bbx_end_pos;
  /* DETERMINE BOUNDING KEYS */
  if (search_whole_map) {
    /* double x, y, z; */
    /* occupancy_octree->getMetricMin(x, y, z); */
    /* bbx_start_pos = octomap::point3d(x, y, z); */
    /* occupancy_octree->getMetricMax(x, y, z); */
    /* bbx_end_pos = octomap::point3d(x, y, z); */
    bbx           = BoundingBox(hires_mode_ ? absolute_max_map_update_dist_highres + 15 : absolute_max_map_update_dist + 15, current_position_);
    bbx_start_pos = octomap::point3d(bbx.x1, bbx.y1, bbx.z1);
    bbx_end_pos   = octomap::point3d(bbx.x2, bbx.y2, bbx.z2);
  } else {
    bbx           = search_bbx;
    bbx_start_pos = octomap::point3d(bbx.x1, bbx.y1, bbx.z1);
    bbx_end_pos   = octomap::point3d(bbx.x2, bbx.y2, bbx.z2);
  }

  /* EDIT SEARCH BOX BOUNDS ACCORINDG TO STAGING AREA */
  bbx       = BoundingBox(bbx_start_pos.x(), bbx_end_pos.x(), bbx_start_pos.y(), bbx_end_pos.y(), bbx_start_pos.z(), bbx_end_pos.z());
  start_key = occupancy_octree->coordToKey(bbx.x1, bbx.y1, bbx.z1, search_depth);
  end_key   = occupancy_octree->coordToKey(bbx.x2, bbx.y2, bbx.z2, search_depth);

  std::vector<octomap::OcTreeKey>  dump_vector  = {};
  std::vector<octomap::OcTreeKey>* key_dump_ptr = &dump_vector;
  if (search_whole_map) {
    std::scoped_lock lock(frontier_border_node_keys_mutex_);
    frontier_border_node_keys_ = {};
    key_dump_ptr               = &frontier_border_node_keys_;
  }

  octomap::point3d   last_point = octomap::point3d(0, 0, 0);
  octomap::OcTreeKey iterator_key;
  for (octomap::OcTree::leaf_bbx_iterator it = occupancy_octree->begin_leafs_bbx(start_key, end_key, search_depth); it != occupancy_octree->end_leafs_bbx();
       it++) {
    iterator_key = it.getKey();
    /* octomap::OcTreeNode* oc_node = occupancy_octree->search(iterator_key, search_depth); */

    if (it == NULL || occupancy_octree->isNodeOccupied(*it)) {
      continue;
    }

    /* check if node is at least partially unknown */
    if (!spheremap_server::hasNodeUnknownChildren(&(*it), occupancy_octree, search_depth)) {
      continue;
    }

    octomap::point3d   s_node_coord    = occupancy_octree->keyToCoord(iterator_key, search_depth);
    octomap::point3d   normal_calc_pos = s_node_coord - octomap::point3d(1, 1, 1) * first_node_in_normal_calculation_offset;
    octomap::OcTreeKey normal_calc_key = occupancy_octree->coordToKey(normal_calc_pos);

    int num_nodes = getFrontierNodeKeys(occupancy_octree, normal_calc_key, 4, key_dump_ptr);
    /* new_frontier.normal                        = sdata.normal; */
    /* new_frontier.normal = octomap::point3d(1, 0, 0); */
    // remove vertical nodes
    /* if (abs(new_frontier.normal.z()) > 0.7) { */
    /*   continue; */
    /* } */
    if (num_nodes < 4) {
      continue;
    }
    /* ROS_INFO("frontier num edge nodes: %d", sdata.num_frontier_nodes); */

    /* CHECK IF NEAR OCCUPIED */
    /* TODO add smarter adding of frontier positions */
  }

  /* ROS_INFO("created %d frontier groups", frontier_nodes_.size()); */
}
//}

/* calculateFrontierGroups2() //{ */
void ExplorationMapper::calculateFrontierGroups2(bool disciard_small_frontier_groups, bool discard_staging_area_groups, bool debug_print) {
  ros::WallTime start_, end_;
  start_                                            = ros::WallTime::now();
  std::shared_ptr<PCLMap>          pclmap_ptr       = getPCLMapSharedPtr();
  std::shared_ptr<octomap::OcTree> occupancy_octree = getOccupancyOcTreeSharedPtr();
  frontier_groups_                                  = {};
  /* ROS_INFO("[FG]: discarding staging area groups: %d", discard_staging_area_groups ? 1 : 0); */

  if (pclmap_ptr == NULL) {
    return;
  }

  /* GROUP NODES TO GROUPS */
  std::vector<FrontierGroup> groups1             = {};
  std::vector<FrontierGroup> groups              = {};
  float                      node_grouping_dist  = frontier_node_filter_dist_;
  float                      node_grouping_dist2 = pow(node_grouping_dist, 2);
  for (uint i = 0; i < frontier_nodes_.size(); i++) {
    bool found_group = false;
    for (uint k = 0; k < groups1.size(); k++) {
      octomap::point3d deltavec = groups1[k].position - frontier_nodes_[i].position;
      if (deltavec.dot(deltavec) < node_grouping_dist2) {
        found_group = true;
        // TODO maybe calc number of ceiling, number of vertical nodes
        groups1[k].addNode(frontier_nodes_[i]);
        break;
      }
    }
    if (!found_group) {
      FrontierGroup new_fg(frontier_nodes_[i].position);
      new_fg.num_grouped_nodes  = frontier_nodes_[i].num_octomap_frontier_nodes;
      new_fg.num_grouped_groups = 1;
      new_fg.sum_position       = frontier_nodes_[i].position;
      groups1.push_back(new_fg);
    }
  }

  /* FILTER GROUPS BASED ON IF GLOBAL SEARCH BOX IS USED */
  if (use_global_search_bbx_) {
    std::vector<FrontierGroup> tmp_groups = {};
    for (uint i = 0; i < groups1.size(); i++) {
      if (global_search_bbx_.isPointInside(groups1[i].position)) {
        tmp_groups.push_back(groups1[i]);
      }
    }
    groups1 = tmp_groups;
  }

  /* FILTER GROUPED POINTS BASED ON NUM OF FILTERED NODES */
  /* THESE FRONTIER NODES ARE NOT EVEN NOISE */
  int frontier_elements_threshold1 = frontier_node_frontier_size_threshold_;
  for (uint i = 0; i < groups1.size(); i++) {
    if (groups1[i].num_grouped_nodes > frontier_elements_threshold1) {
      groups.push_back(groups1[i]);
    }
  }

  /* GROUP FILTERED POINTS */
  float dbscan_dist                 = dbscan_epsilon_;
  float dbscan_dist2                = pow(dbscan_dist, 2);
  int   dbscan_next_index           = -1;
  int   frontier_elements_threshold = frontier_cluster_frontier_size_threshold_;
  for (uint i = 0; i < groups.size(); i++) {
    if (groups[i].dbscan_group > -2) {
      continue;
    }
    /* GET NEIGHBORHOOD */
    std::vector<uint> seed_indices          = {};
    int               sum_frontier_elements = groups[i].num_grouped_nodes;
    for (uint k = 0; k < groups.size(); k++) {
      octomap::point3d deltavec = groups[k].position - groups[i].position;
      if (deltavec.dot(deltavec) < dbscan_dist2) {
        seed_indices.push_back(k);
        sum_frontier_elements += groups[k].num_grouped_nodes;
      }
    }

    /* DETERMINE IF NOISE */
    if (sum_frontier_elements < frontier_elements_threshold) {
      groups[i].dbscan_group = -1;  //-1 = noise
      continue;
    }
    /* NOT NOISE AND NOT LABELED, THEREFORE ADDD NEW CLUSTER */
    dbscan_next_index++;
    groups[i].dbscan_group = dbscan_next_index;
    frontier_cluster_indices_.push_back({i});

    /* ROS_INFO("dbscan num_seeds: %lu", seed_indices.size()); */
    /* UPDATE SEEDS UNTIL NO LEFT (OR TOO BIG?) */
    for (uint k = 0; k < seed_indices.size(); k++) {
      uint seed_index = seed_indices[k];
      if (groups[seed_index].dbscan_group == -1) {
        groups[seed_index].dbscan_group = dbscan_next_index;  // transform noise nodes nearby to nodes of this cluster
        frontier_cluster_indices_[dbscan_next_index].push_back(seed_index);
      }
      if (groups[seed_index].dbscan_group > -2) {
        continue;
      }
      groups[seed_index].dbscan_group = dbscan_next_index;  // add seed point Q to cluster
      frontier_cluster_indices_[dbscan_next_index].push_back(seed_index);

      /* GET NEIGHBORHOOD2 */
      sum_frontier_elements           = groups[seed_index].num_grouped_nodes;
      std::vector<uint> neighborhood2 = {};
      for (uint l = 0; l < groups.size(); l++) {
        octomap::point3d deltavec = groups[l].position - groups[seed_index].position;
        if (deltavec.dot(deltavec) < dbscan_dist2) {
          neighborhood2.push_back(l);
          sum_frontier_elements += groups[l].num_grouped_nodes;
        }
      }
      if (sum_frontier_elements < frontier_elements_threshold) {
        continue;  // point Q is not a core point but reachable from current cluster, we are done here
      }

      /* ADD NEIGHBORHOOD INDICES OF Q TO SEEDSET IF THEY ARE NOT ALREADY IN IT */
      uint seed_indices_size_before_merge = seed_indices.size();
      for (uint l = 0; l < neighborhood2.size(); l++) {
        bool already_in_seedset = false;
        for (uint m = 0; m < seed_indices_size_before_merge; m++) {
          if (seed_indices[m] == neighborhood2[l]) {
            already_in_seedset = true;
            break;
          }
        }
        if (!already_in_seedset) {
          seed_indices.push_back(neighborhood2[l]);
        }
      }
    }
  }
  int num_clusters = dbscan_next_index + 1;

  end_ = ros::WallTime::now();
  /* ROS_INFO("dbscan time: %f ms", (end_ - start_).toSec() * 1000); */
  /* ROS_INFO("dbscan created %d clusters", num_clusters); */
  if (pub_frontier_clusters_.getNumSubscribers() > 0) {
    geometry_msgs::Point       pos2;
    std_msgs::ColorRGBA        vis_color;
    visualization_msgs::Marker marker;
    marker.type    = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    for (uint i = 0; i < groups.size(); i++) {
      octomap::point3d color(0, 0, 0);
      if (groups[i].dbscan_group > -1) {
        color = getSegmentColor(groups[i].dbscan_group);
      }
      vis_color.r = color.x();
      vis_color.g = color.y();
      vis_color.b = color.z();
      vis_color.a = 1;

      pos2.x = groups[i].position.x();
      pos2.y = groups[i].position.y();
      pos2.z = groups[i].position.z();
      marker.points.push_back(pos2);
      marker.colors.push_back(vis_color);
    }
    std::vector<visualization_msgs::Marker> markers = {marker};
    publishMarkers(markers, &pub_frontier_clusters_, "frontier_clusters", map_frame_, 0, -1);
  }


  /* start_ = ros::WallTime::now(); */
  /* GENERATE EXPLORATION POINTS NEAR EACH GROUP */
  int   num_points_circle = 18;
  float delta_z           = 0.4;
  float delta_r           = 0.4;

  int   fep_num_rays      = 100;
  float seg_rays_dist     = 20;
  float infoval_threshold = 0.05;

  int                        num_circles   = 6;
  int                        num_layers    = 6;
  std::vector<FrontierGroup> generated_fgs = {};
  ros::WallTime              start2_, end2_;
  /* start2_                                   = ros::WallTime::now(); */
  /* std::vector<octomap::point3d> deltapoints = spheremap_server::getCylinderSamplingPoints(num_points_circle, delta_r, delta_z, num_circles, num_layers); */
  /* end2_                                     = ros::WallTime::now(); */
  /* ROS_INFO("finding cylinder points took %f ms", (end2_ - start2_).toSec() * 1000); */

  int   max_segment_queries_per_point = 20;
  float far_cluster_dist              = 30;
  /* float far_cluster_dist2 = pow(far_cluster_dist, 2); */

  frontier_groups_ = groups;
  /* markers          = {}; */
  /* for (uint i = 0; i < generated_fgs.size(); i++) { */
  /*   markers.push_back(getMarkerSphere(generated_fgs[i].position, 0.5, octomap::point3d(0.8, 0, 0.8))); */
  /* } */
  /* publishMarkers(markers, &pub_frontier_clusters_, "debug_dbscan", map_frame_); */

  /* CALCULATE SIZES OF FRONTIERS */
  for (uint i = 0; i < frontier_cluster_indices_.size(); i++) {
    float sum_frontier_elements = 0;

    /* ITERATE OVER SPHEREMAP NODES IN BBX */
    for (uint j = 0; j < frontier_cluster_indices_[i].size(); j++) {
      sum_frontier_elements += frontier_groups_[frontier_cluster_indices_[i][j]].num_grouped_nodes;
    }
    frontier_cluster_sizes_.push_back(sum_frontier_elements);
  }


  /* SAMPLE POINTS IN BBX */
}  // namespace spheremap_server
   //}

/* calculateFrontierGroupsForSegmapSending() //{ */
void ExplorationMapper::calculateFrontierGroupsForSegmapSending() {
  frontier_groups_          = {};
  frontier_cluster_indices_ = {};

  /* generateFrontierNodes(); */
  calculateFrontierGroups2(false, true);

  std::shared_ptr<octomap::OcTree> occupancy_octree = getOccupancyOcTreeSharedPtr();
  std::shared_ptr<PCLMap>          pcl_map          = getPCLMapSharedPtr();

  segmap_->frontier_groups_ = {};
  for (uint i = 0; i < segmap_->segment_octree_->segments.size(); i++) {
    segmap_->segment_octree_->segments[i].frontier_value = 0;
  }

  bool discard_staging_area_groups = true;
  /* GENERATE EXPLORATION POINTS NEAR EACH GROUP */
  int   num_points_circle = 18;
  float delta_z           = 0.4;
  float delta_r           = 0.4;

  int   fep_num_rays      = 100;
  float seg_rays_dist     = 20;
  float infoval_threshold = 0.05;

  int num_circles = 6;
  int num_layers  = 6;
  /* std::vector<FrontierGroup> generated_fgs = {}; */
  ros::WallTime start2_, end2_;

  std::vector<octomap::point3d> deltapoints = spheremap_server::getCylinderSamplingPoints(num_points_circle, delta_r, delta_z, num_circles, num_layers);
  /* end2_                                     = ros::WallTime::now(); */
  /* ROS_INFO("finding cylinder points took %f ms", (end2_ - start2_).toSec() * 1000); */

  int   max_segment_queries_per_point = 20;
  float far_cluster_dist              = 30;
  /* float far_cluster_dist2 = pow(far_cluster_dist, 2); */
  std::vector<octomap::point3d> checked_positions = {};
  float                         filter_dist       = 4;
  float                         filter_dist2      = pow(filter_dist, 2);

  int n_filtered_pts    = 0;
  int n_found_frontiers = 0;
  for (uint i = 0; i < frontier_cluster_indices_.size(); i++) {
    std::vector<octomap::point3d> cluster_points  = {};
    int                           num_viable_feps = 0;
    int                           f_distance      = 0;
    int                           f_infoval       = 0;
    int                           f_seg           = 0;

    bool found_viable_fep = false;
    /* FOR EACH POINT SEARCH IN CYLINDRICAL SPACE AROUND FOR FEPS */
    for (uint j = 0; j < frontier_cluster_indices_[i].size(); j++) {
      octomap::point3d gpos = frontier_groups_[frontier_cluster_indices_[i][j]].position;

      bool should_filter_gpos = false;
      for (uint k = 0; k < checked_positions.size(); k++) {
        octomap::point3d deltavec = gpos - checked_positions[k];
        if (deltavec.dot(deltavec) < filter_dist2) {
          should_filter_gpos = true;
        }
      }
      if (should_filter_gpos) {
        n_filtered_pts++;
        continue;
      }

      int  segment_queries                = 0;
      bool found_fep_tied_to_staging_area = false;
      /* TEST POINTS IN CYLINDERS OF INCREASING WIDTH */
      for (uint k = 0; k < deltapoints.size(); k++) {
        octomap::point3d test_point = gpos + deltapoints[k];

        octomap::OcTreeNode* ocnode = occupancy_octree->search(test_point);
        if (ocnode == NULL || occupancy_octree->isNodeOccupied(ocnode)) {
          continue;
        }
        pcl::PointXYZ pcl_point;
        pcl_point.x         = test_point.x();
        pcl_point.y         = test_point.y();
        pcl_point.z         = test_point.z();
        float obstacle_dist = pcl_map->getDistanceFromNearestPoint(pcl_point);
        if (obstacle_dist < 0.8) {
          f_distance++;
          continue;
        }
        auto _fep = getFrontierExplorationData(test_point, fep_num_rays, seg_rays_dist, occupancy_octree);
        if (!_fep || _fep->perc_frontier_hits < infoval_threshold) {
          f_infoval++;
          continue;
        }
        /* segment_queries++; */
        /* if (segment_queries > max_segment_queries_per_point) { */
        /*   found_viable_fep = true;  // FIXME */
        /*   break; */
        /* } */
        int fep_seg_id = getNearestSegmentRaycasting(test_point, 50, 30, occupancy_octree, segmap_->segment_octree_);
        if (fep_seg_id < 0) {
          f_seg++;
          continue;
        }
        octomap::Segment* seg_ptr = segmap_->segment_octree_->getSegmentPtr(fep_seg_id);
        if (seg_ptr == NULL) {
          ROS_ERROR("seg ptr null");
          break;
        }
        if (discard_staging_area_groups && seg_ptr->is_staging_area) {
          /* ROS_INFO("[FG]: frontier cluster is tied to staging area, discarding it"); */
          found_fep_tied_to_staging_area = true;
          break;
        }

        if (seg_ptr->frontier_value < 0.1) {
          ROS_INFO("[SegmapSending]: adding frontier for sending");
        }
        FrontierGroup fg;
        fg.viable_fep.pos    = test_point;
        fg.viable_fep.seg_id = fep_seg_id;
        fg.score             = 1;
        segmap_->frontier_groups_.push_back(fg);
        n_found_frontiers++;
        found_viable_fep = true;
        /* seg_ptr->frontier_value = 1; */
        break;
      }
      if (found_fep_tied_to_staging_area || found_viable_fep) {
        break;
      }
    }
    /* ONLY GEN ONE FRONTIER FOR EACH CLUSTER */
    /* ROS_INFO("found %d viable feps for cluster whose one group has position %f, %f, %f", num_viable_feps, groups[cluster_indices[i][0]].position.x(), */
    /*          groups[cluster_indices[i][0]].position.y(), groups[cluster_indices[i][0]].position.z()); */
    /* ROS_INFO("f_dist: %d, f_infoval: %d, f_seg: %d", f_distance, f_infoval, f_seg); */
    // TODO FIX WHY IT CRASHES AFTER THIS IN THE LOOP
    /* ROS_INFO("[SegmapSending]: f_dist %d, f_infoval %d f_seg %d", f_distance, f_infoval, f_seg); */
  }
  /* ROS_INFO("[SegmapSending]: kocka %d, nff: %d", n_filtered_pts, n_found_frontiers); */
}
//}

/* filterFrontierGroups() //{ */
std::vector<FrontierGroup> ExplorationMapper::filterFrontierGroups(std::vector<FrontierGroup> groups1, float grouping_proj_threshold) {
  std::vector<std::vector<FrontierGroup>> grouped_groups = {};
  int                                     num_grouped    = 0;
  /* GROUP GROUPS THAT ARE OF ONE SEGMENT ID AND ARE IN A SIMILAR DIRECTION */
  /* ROS_INFO("[FrontierGroupFiltration]: starting filtration of %lu frontier groups", groups1.size()); */
  for (uint i = 0; i < groups1.size(); i++) {
    bool              added_to_group = false;
    int               seg1           = groups1[i].viable_fep.seg_id;
    octomap::Segment* seg_ptr        = segmap_->segment_octree_->getSegmentPtr(seg1);
    if (seg_ptr == NULL) {
      ROS_WARN("[FrontierGroupFiltration]: cannot add group, its segment does not exist");
    }
    octomap::point3d seg_pos   = seg_ptr->center;
    octomap::point3d deltavec1 = (groups1[i].viable_fep.pos - seg_pos).normalized();
    for (uint k = 0; k < grouped_groups.size(); k++) {
      int seg2 = grouped_groups[k][0].viable_fep.seg_id;
      if (seg2 != seg1) {
        continue;
      }
      octomap::point3d deltavec2 = (grouped_groups[k][0].viable_fep.pos - seg_pos).normalized();
      float            vec_dot   = deltavec1.dot(deltavec2);
      if (vec_dot > grouping_proj_threshold) {
        num_grouped++;
        grouped_groups[k].push_back(groups1[i]);
        added_to_group = true;
        break;
      }
    }
    if (!added_to_group) {
      grouped_groups.push_back({groups1[i]});
    }
  }
  std::vector<FrontierGroup> res = {};
  for (uint k = 0; k < grouped_groups.size(); k++) {
    /* res.push_back(grouped_groups[k][0]); */
    float best_score       = grouped_groups[k][0].score;
    int   best_group_index = 0;
    for (uint l = 1; l < grouped_groups[k].size(); l++) {
      if (grouped_groups[k][l].score > best_score) {
        best_group_index = l;
        best_score       = grouped_groups[k][l].score;
      }
    }
    res.push_back(grouped_groups[k][best_group_index]);
  }
  /* ROS_INFO("[FrontierGroupFiltration]: filtered %d frontier groups. Resulting size of frontier group vector is %lu", num_grouped, res.size()); */
  return res;
}
//}

//}

}  // namespace spheremap_server
