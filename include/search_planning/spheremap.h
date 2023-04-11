#pragma once
#ifndef SPHEREMAP_H
#define SPHEREMAP_H

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
#include <search_planning/pcl_map.h>

#include <search_planning/SegmapMsg.h>
#include <search_planning/mapping_structures.h>

namespace octomap
{

/* SphereMapNode  //{ */
struct SphereMapNodeData
{
  octomap::point3d                pos;
  float                           radius;
  uint                            segment_id;
  std::vector<octomap::OcTreeKey> connected_keys;
  float                           frotier_infoval = 0;
  float                           path_len        = 0;
  float                           path_min_safety = 0;
  bool                            is_frontier     = false;
  bool                            magic_flag      = false;
  bool                            magic_flag2     = false;
  bool                            is_safe         = true;
  bool                            is_stale        = false;
  /* std:: */

  SphereMapNodeData(octomap::point3d p, float r) {
    pos            = p;
    radius         = r;
    segment_id     = 0;
    connected_keys = {};
  }
  SphereMapNodeData() {
    segment_id     = 0;
    radius         = -1;
    connected_keys = {};
  }
};

class SphereMapNode : public OcTreeDataNode<SphereMapNodeData> {
public:
  // has to be initialized from the outside
  SphereMapNode() : OcTreeDataNode<SphereMapNodeData>(){};

  SphereMapNodeData* valuePtr() {
    return &(this->value);
  }

  int getSegmentID() {
    return this->value.segment_id;
  }

  void setSegmentID(int x) {
    this->value.segment_id = x;
  }

  float getTimeStayed() {
    return this->value.frotier_infoval;
  }

  void addTimeStayed(float t) {
    this->value.frotier_infoval += t;
  }

  bool operator==([[maybe_unused]] const SphereMapNode& rhs) const {
    return false;
  }

  void updateOccupancyChildren() {
  }
};
//}

/* SphereMapOcTree  //{ */
class SphereMapOcTree : public DataOcTree<SphereMapNode> {
public:
  SphereMapOcTree(double res) : DataOcTree<SphereMapNode>(res) {
    ROS_INFO("spheremap octree created");
  }

  /* OcTree mandatory functions //{ */
  /// virtual constructor: creates a new object of same type
  /// (Covariant return type requires an up-to-date compiler)
  SphereMapOcTree* create() const {
    return new SphereMapOcTree(this->resolution);
  }

  std::string getTreeType() const {
    return "OcTreeBase";
  }

  bool isNodeCollapsible([[maybe_unused]] const SphereMapNode* node) {
    return false;
  }
  //}
};
//}


};  // namespace octomap

namespace search_planning
{

/* SphereMapPath //{ */
class SphereMapPath {
public:
  std::vector<uint>             seg_ids;
  std::vector<octomap::point3d> positions;
  std::vector<float>            obstacle_dists;
  float                         getEulerDist();
  float                         length_metres;
  float                         approax_safety;
  float                         min_safedist;
  bool                          reaches_goal = false;

  octomap::point3d original_start;
  octomap::point3d original_goal;

  float calculateSafetyCost(float min_safedist, float base_safedist) {
    float sum = 0;
    if (obstacle_dists.empty()) {
      return 0;
    }
    /* octomap::point3d last_point; */
    float safety;
    float interval = base_safedist - min_safedist;
    for (uint i = 0; i < obstacle_dists.size(); i++) {
      if (obstacle_dists[i] < base_safedist) {
        safety = fmax((obstacle_dists[i] - min_safedist) / interval, 0);  // CAPPED
        sum += 1 - safety;
      }
    }
    return sum;
  }

  float getApproxTravelDistance() {
    float sumdist = 0;
    if (positions.empty()) {
      return 0;
    }
    octomap::point3d last_pos = positions[0];
    for (uint i = 1; i < positions.size(); i++) {
      sumdist += (last_pos - positions[i]).norm();
      last_pos = positions[i];
    }
    return sumdist;
  }

  float computeMinSafedist() {
    if (obstacle_dists.empty()) {
      return 0;
    }
    min_safedist = obstacle_dists[0];
    for (uint i = 1; i < obstacle_dists.size(); i++) {
      if (obstacle_dists[i] < min_safedist) {
        min_safedist = obstacle_dists[i];
      }
    }
    return min_safedist;
  }

  void addPath(SphereMapPath& to_add) {
    if (to_add.positions.empty()) {
      ROS_WARN_THROTTLE(1, "path to add is empty! this should not happen");
      return;
    }
    seg_ids.insert(seg_ids.end(), to_add.seg_ids.begin(), to_add.seg_ids.end());
    positions.insert(positions.end(), to_add.positions.begin(), to_add.positions.end());
    obstacle_dists.insert(obstacle_dists.end(), to_add.obstacle_dists.begin(), to_add.obstacle_dists.end());
  }
  SphereMapPath reversed() {
    SphereMapPath res;
    res.reaches_goal = reaches_goal;
    for (int i = positions.size() - 1; i > -1; i--) {
      res.positions.push_back(positions[i]);
      res.obstacle_dists.push_back(obstacle_dists[i]);
      res.seg_ids.push_back(seg_ids[i]);
    }
    return res;
  }

  bool isSafelyTraversible(float safedist, float safedist_reserve, float start_dist_ignoring_dist = 3, float end_dist_ignoring_dist = 3, bool debug = false,
                           float cutoff_dist = 10000) {
    float dist_total = getApproxTravelDistance();
    if (positions.empty()) {
      return true;
    }

    octomap::point3d last_pos = positions[0];
    float            sumdist  = 0;
    for (uint i = 0; i < obstacle_dists.size(); i++) {
      sumdist += (last_pos - positions[i]).norm();
      last_pos = positions[i];

      if (sumdist > cutoff_dist) {
        return true;
      }

      if (obstacle_dists[i] < safedist - safedist_reserve && sumdist >= start_dist_ignoring_dist && sumdist <= (dist_total - end_dist_ignoring_dist)) {
        ROS_INFO_COND(debug, "encountered unacceptable node with odist %f at %u / %lu", obstacle_dists[i], i, obstacle_dists.size());
        return false;
      }
    }
    return true;
  }

  bool passesThroughBBX(BoundingBox& bbx) {
    for (uint i = 0; i < positions.size(); i++) {
      if (bbx.isPointInside(positions[i])) {
        return true;
      }
    }

    return false;
  }
};
//}

/* SphereMapFrontierPoint  //{ */
class SphereMapFrontierPoint {
public:
  octomap::point3d   pos;
  octomap::OcTreeKey adjacent_node_key;
  float              odist;
  float              information_value;
};
//}

/* SphereMapAstarNode //{ */
struct SphereMapAstarNode
{
  uint                seg_id1 = 0;
  uint                seg_id2;
  SphereMapAstarNode* parent_ptr;
  float               total_g_cost;
  float               h_cost;
  float               total_cost;
  octomap::point3d    pos1;
  octomap::point3d    pos2;
  octomap::OcTreeKey  key1;
  octomap::OcTreeKey  key2;
  uint                cached_path_index;
  SphereMapPath*      path_ptr    = NULL;
  bool                is_reversed = false;

  SphereMapAstarNode() {
  }
  /* SphereMapAstarNode(uint si, SphereMapAstarNode* pp, float gc, float hc, octomap::point3d p, float tc) { */
  SphereMapAstarNode(uint si, SphereMapAstarNode* pp, float gc, float hc, octomap::point3d p, float tc) {
    seg_id2      = si;
    parent_ptr   = pp;
    total_g_cost = gc;
    h_cost       = hc;
    pos2         = p;
    total_cost   = tc;
  }

  SphereMapAstarNode(uint si1, uint si2, SphereMapAstarNode* pp, float gc, float hc, octomap::point3d p1, octomap::point3d p2, octomap::OcTreeKey k1,
                     octomap::OcTreeKey k2, float tc) {
    seg_id1      = si1;
    seg_id2      = si2;
    parent_ptr   = pp;
    total_g_cost = gc;
    h_cost       = hc;
    pos1         = p1;
    pos2         = p2;
    key1         = k1;
    key2         = k2;
    total_cost   = tc;
  }
  bool operator<(const SphereMapAstarNode& a) {
    return total_cost < a.total_cost;
  }
};
//}

/* SphereMapDetailedAstarNode //{ */
struct SphereMapDetailedAstarNode
{
  uint                        seg_id;
  SphereMapDetailedAstarNode* parent_ptr;
  octomap::SphereMapNode*     map_node_ptr;
  float                       total_g_cost;
  float                       h_cost;
  float                       total_cost;
  octomap::point3d            pos;
  octomap::OcTreeKey          key;
  float                       cumulative_infoval = 0;
  float                       safety;

  SphereMapDetailedAstarNode() {
  }
  SphereMapDetailedAstarNode(uint si, SphereMapDetailedAstarNode* pp, float gc, float hc, octomap::point3d p, float tc, octomap::SphereMapNode* mnp) {
    seg_id       = si;
    parent_ptr   = pp;
    total_g_cost = gc;
    h_cost       = hc;
    pos          = p;
    total_cost   = tc;
    map_node_ptr = mnp;
  }
  bool operator<(const SphereMapDetailedAstarNode& a) {
    return total_cost < a.total_cost;
  }
};
//}


/* SphereMapSegmentConnection  //{ */
class SphereMapSegmentConnection {
public:
  /* uint               seg_id1, seg_id2; */
  octomap::OcTreeKey own_key, other_key;
  float              safety;
  SphereMapSegmentConnection() {
  }
  SphereMapSegmentConnection(octomap::OcTreeKey own_k, octomap::OcTreeKey other_k, float s) {
    own_key   = own_k;
    other_key = other_k;
    safety    = s;
  }
  bool shouldUpdateWith(const SphereMapSegmentConnection& conn) {
    return (own_key != conn.own_key || other_key != conn.other_key || safety < conn.safety);
  }
  SphereMapSegmentConnection getSwappedCopy() {
    return SphereMapSegmentConnection(other_key, own_key, safety);
  }
};
//}

/* SphereMapSegment //{ */
class SphereMapSegment {
public:
  std::vector<octomap::OcTreeKey>                keys;
  octomap::point3d                               center;
  float                                          bounding_sphere_radius;
  std::map<uint, SphereMapSegmentConnection>     connections;
  std::map<std::pair<uint, uint>, SphereMapPath> interportal_paths;

  std::vector<octomap::point3d> block_dirs;
  float                         block_alpha = 0;
  float                         block_beta  = 0;
  float                         block_a     = 1;
  float                         block_b     = 1;
  float                         block_c     = 1;

  bool  is_connected_to_home = true;
  float dist_from_home;
};
//}

/* SphereMap //{ */
class SphereMap {
public:
  uint                          base_depth_ = 16;
  TopologyMappingSettings       topology_mapping_settings_;
  std::vector<octomap::point3d> debug_points;
  BoundingBox                   generation_deny_bbx;

  std::shared_ptr<octomap::SphereMapOcTree> nodes;
  std::vector<std::pair<uint, uint>>        portals;
  std::vector<octomap::point3d>             comm_nodes;
  std::set<std::pair<uint, uint>>           potential_merge_pairs_;
  std::map<uint, SphereMapPath>             cached_search_start_paths_;

  std::vector<uint> changed_segments;

  std::vector<octomap::point3d>    cached_frontier_positions;
  std::vector<float>               cached_frontier_values;
  bool                             computed_cached_frontiers_atleast_once_ = false;
  std::shared_ptr<octomap::OcTree> current_octree_;
  SphereMap() {
  }

  float max_update_box_size_;

  float planning_min_safe_dist;
  float planning_base_safe_dist;
  float min_safe_dist;
  float base_safe_dist              = 1.6;
  float enforced_max_sphere_radius  = 3;
  float nearest_best_obstacle_dist_ = 3;
  void  updateNearestBestObstacleDist(octomap::point3d pos, float box_halfsize);
  float sphere_intersection_reserve     = 0.2;
  float sphere_intersection_reserve_big = 1;  // after r>2
  float largest_bounging_sphere_radius_;
  bool  transform_segmap_to_shared_frame_ = false;

  bool areConnectable(float dist2, float radius1, float radius2);
  bool isPinnedBy(float dist2, octomap::point3d smaller_pos, octomap::point3d larger_pos, float smaller_rad, float larger_rad);
  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> getMaxSearchBBXBorderKeys(octomap::point3d pos);
  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> getMaxSearchBBXBorderKeys(octomap::point3d pos, float box_halfsize);
  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> getMaxSearchBBXBorderKeys(BoundingBox& bbx);
  void                                update(octomap::point3d current_position_, float current_heading_, std::shared_ptr<octomap::OcTree> occupancy_octree_,
                                             std::shared_ptr<PCLMap> pcl_map_ptr);
  std::set<uint>                      seg_ids_for_interportal_recomputation;
  void                                updateConnectionsForNode(octomap::OcTreeKey key, bool check_connected_nodes = true);
  bool                                connectedNodesFormFullGraph(octomap::SphereMapNode* node_ptr);
  std::pair<octomap::OcTreeKey, bool> addNode(octomap::SphereMapNodeData data);
  bool                                removeNode(octomap::OcTreeKey key);
  void                                updateObstacleDists(std::shared_ptr<PCLMap> pcl_map_ptr, octomap::point3d box_pos, float box_halfsize);
  void                                expandAndPrune(std::shared_ptr<PCLMap> pcl_map_ptr, octomap::point3d box_pos, float box_halfsize);
  bool                                isPointInMap(octomap::point3d);
  bool                                isPointInMap(octomap::point3d, float box_halfsize);
  void growNodesSimple(octomap::point3d pos, float box_halfsize, std::shared_ptr<octomap::OcTree> occupancy_octree_, std::shared_ptr<PCLMap> pcl_map_ptr);
  void growNodesFull(octomap::point3d pos, float box_halfsize, std::shared_ptr<octomap::OcTree> occupancy_octree_, std::shared_ptr<PCLMap> pcl_map_ptr);
  SphereMap(float _min_safe_dist, float _base_safe_dist, TopologyMappingSettings topo_settings, StagingAreaSettings* _staging_area_settings);

  /* SEGMENTS */
  std::map<uint, SphereMapSegment> segments;

  uint next_seg_id = 1;
  /* SEG_ID 1 IS RESERVED FOR STAGING_AREA */

  void expansionsStep(octomap::point3d current_position_, std::shared_ptr<octomap::OcTree> occupancy_octree_, std::shared_ptr<PCLMap> pcl_map_ptr);
  void segmentationStep(octomap::point3d current_position_, std::shared_ptr<octomap::OcTree> occupancy_octree_, std::shared_ptr<PCLMap> pcl_map_ptr);

  std::vector<uint> getSegmentsIntersectingBBX(octomap::point3d pos, float box_halfsize);
  uint              spreadSegment(uint seg_id);
  uint              growSegment(octomap::OcTreeKey);
  void              rebuildSegments(octomap::point3d current_position_, float box_halfsize);
  void              removeSegment(std::map<uint, SphereMapSegment>::iterator it);
  uint              checkSegmentStateAfterKeyRemoval(uint seg_id, std::vector<std::vector<octomap::OcTreeKey>>& surviving_keys);
  uint              tryMergingSegments(std::map<uint, SphereMapSegment>::iterator it1, std::map<uint, SphereMapSegment>::iterator it2,
                                       std::shared_ptr<octomap::OcTree> occupancy_octree_);
  uint              updateConnectionsForSegment(std::map<uint, SphereMapSegment>::iterator it1);
  uint              updateConnectionsForSegmentWithFoundConnections(std::map<uint, SphereMapSegment>::iterator it1,
                                                                    std::map<uint, SphereMapSegmentConnection> safest_connections);
  uint mergeSegments(std::map<uint, SphereMapSegment>::iterator it1, std::map<uint, SphereMapSegment>::iterator it2, bool force_seg1_to_be_master = false);
  uint mergeSegments(std::map<uint, SphereMapSegment>::iterator it1, std::map<uint, SphereMapSegment>::iterator it2, octomap::point3d new_center,
                     float new_bounding_sphere_radius, bool force_seg1_to_be_master = false);
  struct segmapnode_pair_compare
  {
    inline bool operator()(const std::pair<float, octomap::SphereMapOcTree::leaf_bbx_iterator>& struct1,
                           const std::pair<float, octomap::SphereMapOcTree::leaf_bbx_iterator>& struct2) {
      return (struct1.first < struct2.first);
    }
  };
  void getUnsegmentedNodesWithObstacleDists(octomap::point3d current_position_, float box_halfsize,
                                            std::vector<std::pair<float, octomap::SphereMapOcTree::leaf_bbx_iterator>>& res_pairs);

  /* PRUNING */
  std::vector<octomap::OcTreeKey> latest_unsafe_node_keys_;
  float                           node_unsafe_time_until_purge = 2;
  float                           node_purging_max_dist;

  std::set<uint> segments_with_purged_nodes;
  void           purgeNodesSimple(octomap::point3d pos, float box_halfsize, bool perform_connectedness_check = false);
  void           purgeNodesWithTimecheck(octomap::point3d pos, float box_halfsize);
  bool           removeSegment(uint seg_id);

  /* STAGING AREA HANDLING */
  StagingAreaSettings* staging_area_settings_ptr_;
  bool                 left_staging_area = false;
  octomap::point3d     home_position_;
  bool                 initialized_home_position_;

  /* NAVIGATION */
  std::vector<octomap::OcTreeKey>                     getAdjacentNodesKeys(octomap::point3d test_point);
  std::optional<std::pair<float, octomap::OcTreeKey>> getNearestNodeKey(octomap::point3d test_point);
  std::optional<std::pair<float, octomap::OcTreeKey>> getNearestNodeKey(octomap::point3d test_point, float max_dist);
  std::vector<std::pair<float, octomap::OcTreeKey>>   getNearestNodeKeysForEachSegment(octomap::point3d test_point, float max_dist,
                                                                                       bool only_add_connectable = false, float goal_odist = 0);

  std::vector<octomap::OcTreeKey>                     getConnectableNodesKeys(octomap::point3d test_point, float odist);
  std::optional<std::pair<float, octomap::OcTreeKey>> getNearestConnectableNodeKey(octomap::point3d test_point, float odist);
  std::optional<std::pair<float, octomap::OcTreeKey>> getNearestVisibleNodeKey(octomap::point3d test_point, float odist);


  std::optional<std::vector<SphereMapPath>> computePathsToGoalsWithConnectingToGraph(octomap::point3d start_pos, float start_pos_obstacle_distance,
                                                                                     std::vector<octomap::point3d> goal_positions, float max_goal_pos_dist,
                                                                                     bool verbose, bool use_precomputed_paths);

  SphereMapPath computePath(octomap::point3d start_point, octomap::point3d goal_point, bool do_astar_in_segments = false, bool request_min_safedist = false,
                            float min_path_safedist = 2);
  SphereMapPath computePath(uint start_seg_id, octomap::point3d start_pos, uint goal_seg_id, octomap::point3d goal_point,
                            bool compute_astar_in_segments = false, octomap::OcTreeKey node_key_nearest_to_start = octomap::OcTreeKey(),
                            octomap::OcTreeKey node_key_nearest_to_goal = octomap::OcTreeKey(), bool compute_astar_economically = false,
                            bool request_min_safedist = false, float min_path_safedist = 2);
  SphereMapPath computeDetailedPathInSegment(octomap::OcTreeKey start_key, octomap::OcTreeKey goal_key, float min_odist);
  SphereMapPath computeDetailedPathInSegment(octomap::OcTreeKey start_key, octomap::OcTreeKey goal_key, std::map<uint, SphereMapSegment>::iterator seg_ptr,
                                             float min_odist, bool override_segment_necessity = false);

  /* FRONTIERS */
  std::vector<SphereMapFrontierPoint> frontier_exporation_points_;
  void updateFrontiers(bool search_whole_map, octomap::point3d pos, float box_halfsize, std::shared_ptr<octomap::OcTree> occupancy_octree_);

  /* SENDING */
  void                   computeBoundingBlockForSegment(std::map<uint, SphereMapSegment> seg_ptr, std::vector<octomap::SphereMapNode*>& points);
  std::vector<SegmapMsg> getSegmapMsgs(uint segmap_message_index, int max_data_bytes, tf2_ros::Buffer* tf_buffer, std::string segmap_frame,
                                       std::string shared_frame);
  void                   updateDistsFromHome();

  void checkIntegrity() {
    uint num_bad = 0;
    for (octomap::SphereMapOcTree::iterator it = nodes->begin_leafs(); it != nodes->end_leafs(); it++) {
      if (nodes->coordToKey(it->valuePtr()->pos, 16) != it.getKey()) {
        ROS_ERROR("vole2");
      }
      if (it.getDepth() != 16) {
        ROS_WARN("roswtf");
      }
      /* if (it->valuePtr()->connected_keys.size() > 100) { */
      /*   ROS_WARN("node has %lu connections", it->valuePtr()->connected_keys.size()); */
      /* } */
      for (uint i = 0; i < it->valuePtr()->connected_keys.size(); i++) {
        if (nodes->search(it->valuePtr()->connected_keys[i], 16) == NULL) {
          if (num_bad == 0) {
            ROS_WARN("found some bullshit thing");
          }
          num_bad++;
        } else {
          /* check reciprocity */
          octomap::SphereMapNode* adj_ptr = nodes->search(it->valuePtr()->connected_keys[i], 16);
          if (std::find(adj_ptr->valuePtr()->connected_keys.begin(), adj_ptr->valuePtr()->connected_keys.end(), nodes->coordToKey(it->valuePtr()->pos, 16)) ==
              adj_ptr->valuePtr()->connected_keys.end()) {
            ROS_ERROR("recipro error");
          }
        }
      }
    }
    ROS_WARN("found %u bullshit things", num_bad);
  }
};
//}

/* utils //{ */
bool reconstructSphereMapDetailedPath(SphereMapPath& res, SphereMapDetailedAstarNode goal_node);
//}

};  // namespace search_planning

#endif
