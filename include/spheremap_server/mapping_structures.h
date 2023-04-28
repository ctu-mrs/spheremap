#pragma once
#include <tf2_ros/transform_listener.h>
#ifndef MAPPINGSTRUCTURES_H
#define MAPPINGSTRUCTURES_H

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
#include <spheremap_server/pcl_map.h>

#include <spheremap_server/SegmapMsg.h>

namespace spheremap_server
{
struct CameraInfo
{
  bool             connected   = true;
  bool             initialized = false;
  float            max_visible_dist;
  octomap::point3d camera_FOV_top_left_corner;
  std::string      camera_info_topic;
  std::string      camera_frame;
  CameraInfo(std::string topic) {
    camera_info_topic = topic;
  }
};
struct SurfaceGroup
{
  float            num_grouped_facets        = 0;
  float            num_grouped_ground_facets = 0;
  octomap::point3d pos;
  SurfaceGroup(octomap::point3d p, float nf, float ng) {
    pos                       = p;
    num_grouped_facets        = nf;
    num_grouped_ground_facets = ng;
  }
};

/* SurfaceNode //{ */
struct SurfaceNode
{
  SurfaceNode(octomap::point3d p, octomap::point3d n) {
    pos    = p;
    normal = n;
  }
  octomap::point3d pos;
  octomap::point3d normal;
};
//}

/* BoundingBox //{*/
class BoundingBox {
public:
  float x1, x2, y1, y2, z1, z2;
  BoundingBox() {
    x1 = 0;
    x2 = 0;
    y1 = 0;
    y2 = 0;
    z1 = 0;
    z2 = 0;
  }
  BoundingBox(float tx1, float tx2, float ty1, float ty2, float tz1, float tz2) {
    x1 = tx1;
    x2 = tx2;
    y1 = ty1;
    y2 = ty2;
    z1 = tz1;
    z2 = tz2;
  }

  BoundingBox(float cube_half_length, octomap::point3d center = octomap::point3d(0, 0, 0)) {
    x1 = center.x() - cube_half_length;
    x2 = center.x() + cube_half_length;
    y1 = center.y() - cube_half_length;
    y2 = center.y() + cube_half_length;
    z1 = center.z() - cube_half_length;
    z2 = center.z() + cube_half_length;
  }
  bool isPointInside(octomap::point3d p) {
    if (p.x() > x1 && p.x() < x2 && p.y() > y1 && p.y() < y2 && p.z() > z1 && p.z() < z2) {
      return true;
    }
    return false;
  }
  void expand(float d) {
    x1 -= d;
    x2 += d;
    y1 -= d;
    y2 += d;
    z1 -= d;
    z2 += d;
  }
};
//}

/* StagingAreaSettings //{ */
struct StagingAreaSettings
{
  octomap::point3d wall_pos;
  octomap::point3d wall_dir_outward;
  bool             enabled;
  StagingAreaSettings() {
  }
  StagingAreaSettings(bool en, octomap::point3d pos = octomap::point3d(0, 0, 0), octomap::point3d walldir = octomap::point3d(0, 0, 0)) {
    enabled          = en;
    wall_pos         = pos;
    wall_dir_outward = walldir;
  }
};
//}

/* TopologyMappingSettings //{ */
struct TopologyMappingSettings
{
  int   segmap_default_depth_offset_;
  bool  merging_enabled_;
  float segment_max_size_;
  float compactness_delta_;
  float merged_segment_max_size_;
  float merged_compactness_factor_;
  float merged_convexity_threshold_;
  float convexity_threshold;
  float segmap_voxel_min_safe_distance_;

  int   num_expand_tries;
  int   num_merge_tries;
  int   num_max_grown;
  int   num_rays_for_growth;
  float growth_raydist;
  float raycast_growing_safe_distance;
};
//}

}  // namespace spheremap_server

namespace octomap
{
/* DataOcTree and helping structs //{ */
template <class NODE>
class DataOcTree : public OcTreeBaseImpl<NODE, AbstractOcTree> {
public:
  DataOcTree(double res)
      : OcTreeBaseImpl<NODE, AbstractOcTree>(res){
            /* ROS_INFO("YEET"); */
        };

  /// virtual constructor: creates a new object of same type
  /// (Covariant return type requires an up-to-date compiler)
  DataOcTree* create() const {
    return new DataOcTree(this->resolution);
  }

  std::string getTreeType() const {
    return "DataOcTree";
  }

  NODE* touchNode(double x, double y, double z, unsigned int target_depth = 0) {
    OcTreeKey key;
    if (target_depth == 0) {
      if (!this->coordToKeyChecked(x, y, z, key)) {
        ROS_INFO("checked failed");
        return NULL;
      }
      return touchNode(key);
    } else {
      if (!this->coordToKeyChecked(x, y, z, target_depth, key)) {
        ROS_INFO("checked failed");
        return NULL;
      }
      return touchNode(key, target_depth);
    }
  }

  NODE* touchNode(const OcTreeKey& key, unsigned int target_depth = 0) {
    // early abort (no change will happen).
    // may cause an overhead in some configuration, but more often helps
    NODE* leaf = this->search(key, target_depth);
    /* ROS_INFO("starting search"); */
    // no change: node already at threshold
    if (leaf) {
      return leaf;
    }

    bool createdRoot = false;
    if (this->root == NULL) {
      this->root = new NODE();
      this->tree_size++;
      createdRoot = true;
      /* ROS_INFO("creaated root, max depth: %d", this->tree_depth); */
    }

    return touchNodeRecurs(this->root, createdRoot, key, 0, target_depth);
  }

  NODE* touchNodeRecurs(NODE* node, bool node_just_created, const OcTreeKey& key, unsigned int depth, unsigned int max_depth = 0) {
    bool created_node = false;

    assert(node);

    // follow down to last level
    if (depth < this->tree_depth && (max_depth == 0 || depth < max_depth)) {
      unsigned int pos = computeChildIdx(key, this->tree_depth - 1 - depth);
      /* ROS_INFO("pos: %d", pos); */
      if (!this->nodeChildExists(node, pos)) {
        // child does not exist, but maybe it's a pruned node?
        if (!this->nodeHasChildren(node) && !node_just_created) {
          // current node does not have children AND it is not a new node
          // -> expand pruned node
          /* ROS_INFO("expanded node"); */
          this->expandNode(node);
        } else {
          // not a pruned node, create requested child
          this->createNodeChild(node, pos);
          /* ROS_INFO("added child"); */
          created_node = true;
        }
      }
      return touchNodeRecurs(this->getNodeChild(node, pos), created_node, key, depth + 1, max_depth);
    }

    // at last level, update node, end of recursion
    else {
      return node;
    }
  }

protected:
  /**
   * Static member object which ensures that this OcTree's prototype
   * ends up in the classIDMapping only once. You need this as a
   * static member in any derived octree class in order to read .ot
   * files through the AbstractOcTree factory. You should also call
   * ensureLinking() once from the constructor.
   */
  class StaticMemberInitializer {
  public:
    StaticMemberInitializer() {
      OcTree* tree = new OcTree(0.1);
      tree->clearKeyRays();
      AbstractOcTree::registerTreeType(tree);
    }

    /**
     * Dummy function to ensure that MSVC does not drop the
     * StaticMemberInitializer, causing this tree failing to register.
     * Needs to be called from the constructor of this octree.
     */
    void ensureLinking(){};
  };

  /// to ensure static initialization (only once)
  static StaticMemberInitializer ocTreeMemberInit;
};
//}

/* SurfaceOcTree and helping structs //{ */
struct SurfaceData
{
  octomap::point3d normal;
  float            dist_center_to_surface;
  int              num_frontier_nodes;
};

struct SurfaceOcNodeStruct
{
  std::vector<spheremap_server::SurfaceNode> explored;
  std::vector<spheremap_server::SurfaceNode> unexplored;
};

struct SurfaceNodeKey
{
  octomap::OcTreeKey key;
  unsigned int       index;
  SurfaceNodeKey(octomap::OcTreeKey k, unsigned int i) {
    key   = k;
    index = i;
  }
};

class SurfaceOcNode : public OcTreeDataNode<SurfaceOcNodeStruct> {
public:
  // has to be initialized from the outside
  SurfaceOcNode() : OcTreeDataNode<SurfaceOcNodeStruct>() {
    value.explored   = {};
    value.unexplored = {};
  };

  void clearUnexplored() {
    value.unexplored = {};
  }

  std::vector<spheremap_server::SurfaceNode>* getExploredPtr() {
    return &value.explored;
  }

  std::vector<spheremap_server::SurfaceNode>* getUnexploredPtr() {
    return &value.unexplored;
  }


  bool operator==([[maybe_unused]] const SurfaceOcNode& rhs) const {
    return false;
    // TODO create different isnodecollapsible function when i want to collapse the nodes
  }
};

class SurfaceOcTree : public DataOcTree<SurfaceOcNode> {
public:
  int   depth_offset;
  int   leaf_voxel_length;
  int   num_voxels_in_leaf;
  float leaf_side_length;

  SurfaceOcTree(double res, int _depth_offset) : DataOcTree<SurfaceOcNode>(res) {
    depth_offset       = _depth_offset;
    leaf_voxel_length  = pow(2, depth_offset);
    num_voxels_in_leaf = pow(leaf_voxel_length, 3);
    leaf_side_length   = this->getResolution() * leaf_voxel_length;
  };

  std::vector<OcTreeKey> getKeysInBBX(int depth, float x1, float x2, float y1, float y2, float z1, float z2) {
    std::vector<OcTreeKey> res = {};

    int       step     = pow(2, 16 - depth);
    OcTreeKey startkey = this->coordToKey(x1, y1, z1, depth);
    OcTreeKey endkey   = this->coordToKey(x2, y2, z2, depth);
    for (int x = startkey[0]; x <= endkey[0]; x += step) {
      for (int y = startkey[1]; y <= endkey[1]; y += step) {
        for (int z = startkey[2]; z <= endkey[2]; z += step) {
          OcTreeKey key = OcTreeKey(x, y, z);
          res.push_back(key);
        }
      }
    }
    return res;
  }

  std::vector<OcTreeKey> getKeysInBBX(int depth, spheremap_server::BoundingBox bbx) {
    return this->getKeysInBBX(depth, bbx.x1, bbx.x2, bbx.y1, bbx.y2, bbx.z1, bbx.z2);
  }

  std::vector<OcTreeKey> getKeysWithUnknownNodesInBBX(int depth, float x1, float x2, float y1, float y2, float z1, float z2) {
    std::vector<OcTreeKey> res = {};

    int       step     = pow(2, 16 - depth);
    OcTreeKey startkey = this->coordToKey(x1, y1, z1, depth);
    OcTreeKey endkey   = this->coordToKey(x2, y2, z2, depth);
    for (int x = startkey[0]; x <= endkey[0]; x += step) {
      for (int y = startkey[1]; y <= endkey[1]; y += step) {
        for (int z = startkey[2]; z <= endkey[2]; z += step) {
          OcTreeKey      key  = OcTreeKey(x, y, z);
          SurfaceOcNode* node = search(key, depth);
          if (node == NULL || node->getUnexploredPtr() == NULL || node->getUnexploredPtr()->size() == 0) {
            continue;
          }
          res.push_back(key);
        }
      }
    }
    return res;
  }

  std::vector<OcTreeKey> getKeysWithUnknownNodesInBBX(int depth, spheremap_server::BoundingBox bbx) {
    return this->getKeysWithUnknownNodesInBBX(depth, bbx.x1, bbx.x2, bbx.y1, bbx.y2, bbx.z1, bbx.z2);
  }

  void clearUnexplored() {
    for (octomap::SurfaceOcTree::iterator it = this->begin(), end = this->end(); it != end; ++it) {
      it->clearUnexplored();
    }
  }

  /// virtual constructor: creates a new object of same type
  /// (Covariant return type requires an up-to-date compiler)
  SurfaceOcTree* create() const {
    return new SurfaceOcTree(this->resolution, this->depth_offset);
  }

  std::string getTreeType() const {
    return "OcTreeBase";
  }

  bool isNodeCollapsible([[maybe_unused]] const SurfaceOcNode* node) {
    return false;
  }
};
//}

/* SegmentPath, SegmentAstarGoalCondition, SegmentAstarHeuristic //{ */
enum SegmentAstarHeuristic
{
  EULER_TO_POINT
};
enum SegmentAstarGoalCondition
{
  ONE_SEGMENT,
  MULTIPLE_SEGMENTS
};
struct SegmentPath
{
  std::vector<int>              ids;
  std::vector<octomap::point3d> positions;
  SegmentPath() {
    ids       = {};
    positions = {};
  }
  SegmentPath(std::vector<int> _ids, std::vector<octomap::point3d> _positions) {
    ids       = _ids;
    positions = _positions;
  }
  ~SegmentPath() {
  }
};
//}

/* SegmentPortal //{ */
struct SegmentPortal
{
  int              id1;
  int              id2;
  octomap::point3d position;
  /* std::vector<octomap::point3d> positions; */
  bool have_segs_changed_from_last_merge_attempt = true;
};
//}

/* Segment //{ */
class Segment {
public:
  int                    id;
  int                    num_nodes;
  std::vector<OcTreeKey> border_keys;
  std::vector<OcTreeKey> inside_keys;
  octomap::point3d       center;
  octomap::point3d       nav_center;
  bool                   nav_center_is_safe;
  std::vector<int>       connected_segments_ids;
  float                  exploredness;
  bool                   expansion_disabled;
  bool                   associated_with_current_map = false;
  bool                   is_staging_area             = false;
  bool                   is_staging_area_wall        = false;
  bool                   has_merged_atleast_once     = false;
  bool                   is_narrow_space             = false;
  float                  narrowness_factor           = 0;
  float                  bounding_sphere_radius      = 0;

  float distance_from_home;
  bool  is_connected_to_home = false;

  float frontier_value        = 0;
  float global_frontier_value = 0;

  std::vector<octomap::point3d> block_dirs;
  float                         block_alpha = 0;
  float                         block_beta  = 0;
  float                         block_a     = 1;
  float                         block_b     = 1;
  float                         block_c     = 1;

  float surface_coverage_local           = 0;
  float surface_coverage_global          = 0;
  float explored_surface_nodes_infoval   = 0;
  float unexplored_surface_nodes_infoval = 0;
  Segment(int _id) {
    nav_center             = octomap::point3d(0, 0, 0);
    nav_center_is_safe     = false;
    id                     = _id;
    num_nodes              = 0;
    connected_segments_ids = {};
    border_keys            = {};
    inside_keys            = {};
    exploredness           = 0;
    expansion_disabled     = false;
  }
  ~Segment() {
    /* ROS_INFO("segment deleted"); */
  }
};
//}

/* SegmentNode //{ */
struct SegmentNodeData
{
  int segment_id;
};

class SegmentNode : public OcTreeDataNode<SegmentNodeData> {
public:
  // has to be initialized from the outside
  SegmentNode() : OcTreeDataNode<SegmentNodeData>() {
    this->value.segment_id = -1;
  };

  int getSegmentID() {
    return this->value.segment_id;
  }

  void setSegmentID(int x) {
    this->value.segment_id = x;
  }

  bool operator==([[maybe_unused]] const SegmentNode& rhs) const {
    return false;
    // TODO create different isnodecollapsible function when i want to collapse the nodes
  }
};
//}

/* SegmentOcTree //{ */
struct SegmentAstarNode
{
  int               seg_id;
  SegmentAstarNode* parent_ptr;
  float             total_g_cost;
  float             h_cost;
  float             total_cost;
  octomap::point3d  pos;

  SegmentAstarNode() {
  }
  SegmentAstarNode(int si, SegmentAstarNode* pp, float gc, float hc, octomap::point3d p, float tc) {
    seg_id       = si;
    parent_ptr   = pp;
    total_g_cost = gc;
    h_cost       = hc;
    pos          = p;
    total_cost   = tc;
  }
  bool operator<(const SegmentAstarNode& a) {
    return total_cost < a.total_cost;
  }
};

class SegmentOcTree : public DataOcTree<SegmentNode> {
public:
  bool                                  verbose = false;
  int                                   depth_offset;
  int                                   leaf_voxel_length;
  int                                   num_voxels_in_leaf;
  std::vector<Segment>                  segments;
  std::vector<SegmentPortal>            portals;
  int                                   last_segment_id;
  float                                 voxel_min_safe_distance_;
  spheremap_server::StagingAreaSettings* staging_area_settings_ptr_;

  std::shared_ptr<octomap::OcTree>        occupancy_octree = NULL;
  std::shared_ptr<spheremap_server::PCLMap> pcl_map_ptr      = NULL;

  std::vector<octomap::point3d> debug_points_;

  SegmentOcTree(double res, int _depth_offset = 16) : DataOcTree<SegmentNode>(res) {
    depth_offset               = _depth_offset;
    leaf_voxel_length          = pow(2, depth_offset);
    num_voxels_in_leaf         = pow(leaf_voxel_length, 3);
    staging_area_settings_ptr_ = NULL;

    segments        = {};
    last_segment_id = 0;
  };


  int getSegmentID(octomap::point3d pt);

  int getSegmentID(octomap::OcTreeKey key);

  int getSegmentIndex(int id);

  Segment* getSegmentPtr(int id);

  bool isKeyInVector(OcTreeKey key, const std::vector<OcTreeKey>& ptr);

  std::optional<octomap::point3d> getBestPortalPositionBetweenSegments(int id1, int id2, float min_safe_distance);

  bool addPortal(SegmentPortal portal);

  int expandSegment(int seg_id, float max_radius, float compactness_delta = -1, float convexity_threshold = -1);

  int growSegment(octomap::point3d start_pt, float max_radius, bool growing_staging_area_segment = false, float compactness_delta = -1,
                  float convexity_threshold = -1);

  bool deleteSegment(int seg_id);

  bool deleteConnections(int id1, int id2);

  void updateConnectionsForSegment(int seg_id);

  void setPortalsOfSegmentMergeReadiness(int seg_id, bool val) {
    for (std::vector<SegmentPortal>::iterator it = portals.begin(); it != portals.end(); it++) {
      if (it->id1 == seg_id || it->id2 == seg_id) {
        it->have_segs_changed_from_last_merge_attempt = val;
      }
    }
  }

  void updateNavCenterForSegment(int seg_id);

  int getNearestSegment(octomap::point3d pos, float max_euclid_dist, bool plan_to_unreachable_point);

  bool areSegmentsConnected(int id1, int id2);

  SegmentPortal* getPortalPtr(int id1, int id2);

  float getHeuristicCost(octomap::point3d current_plan_position, SegmentAstarHeuristic heuristic_type, void* args);

  bool isSegmentGoal(int seg_id, SegmentAstarGoalCondition goal_type, void* args);

  std::vector<int> getSegmentsInSphere(octomap::point3d center, float radius);

  bool getSegmentsJoinedKeys(int id1, int id2, std::vector<octomap::OcTreeKey>& new_border_keys, std::vector<octomap::OcTreeKey>& new_inside_keys);

  bool tryMerge(int id1, int id2, float compactness_index, float max_joined_size = -1, float convexity_index = -1);

  bool tryGrowingFromSegmentsSide(int id1);  // todo cache if seg changed

  bool mergeSegments(int id1, int id2);

  void recalculateDistsFromHome();

  void spreadFrontierValue(octomap::point3d frontier_pos, int start_seg_id, float max_spread_dist);

  bool isSegmentCompletelyEnclosedByUnexpandableSpace(Segment* seg_ptr);

  bool isSegmentInsideBBXOfSegment(Segment* s1, Segment* s2);


  /* OcTree mandatory functions //{ */
  /// virtual constructor: creates a new object of same type
  /// (Covariant return type requires an up-to-date compiler)
  SegmentOcTree* create() const {
    return new SegmentOcTree(this->resolution, this->depth_offset);
  }

  std::string getTreeType() const {
    return "OcTreeBase";
  }

  bool isNodeCollapsible([[maybe_unused]] const SegmentNode* node) {
    return false;
  }
  //}
};
//}
};  // namespace octomap


namespace spheremap_server
{

/* Ellipsoid definition //{*/
class Ellipsoid {
public:
  Ellipsoid() {
  }
  Ellipsoid(octomap::point3d pos, octomap::point3d v1, octomap::point3d v2, octomap::point3d v3) {
    position  = pos;
    values[0] = v1.norm();
    axes[0]   = v1 * (1 / values[0]);
    values[1] = v2.norm();
    axes[1]   = v2 * (1 / values[1]);
    values[2] = v3.norm();
    axes[2]   = v3 * (1 / values[2]);
  }
  octomap::point3d position;
  // should be sorted
  octomap::point3d axes[3];
  float            values[3];
  bool             isPointInside(octomap::point3d point) {
    octomap::point3d delta = point - position;
    float            x     = delta.dot(axes[0]);
    float            y     = delta.dot(axes[1]);
    float            z     = delta.dot(axes[2]);

    float dist2 = pow(x / values[0], 2) + pow(y / values[1], 2) + pow(z / values[2], 2);
    if (dist2 > 1) {
      return false;
    }
    return true;
  }
  float getVolume() {
    return (0.75) * M_PI * values[0] * values[1] * values[2];
  }
  float getSphereRadius() {
    return cbrt(values[0] * values[1] * values[2]);
  }
  visualization_msgs::Marker getMarker(const ros::Time& rostime);
};

//}

/* FrontierExplorationPoint //{ */
class FrontierExplorationPoint {
public:
  octomap::point3d     pos;
  octomap::point3d     frontier_direction;
  float                perc_horizontal_frontier_hits;
  float                perc_vertical_frontier_hits;
  float                perc_frontier_hits;
  float                reward;
  octomap::SegmentPath segpath;
  int                  seg_id = -1;
  FrontierExplorationPoint() {
  }
};
//}

/* FrontierNode //{ */
class FrontierNode {
public:
  octomap::point3d position;
  /* octomap::point3d normal; */
  float radius;
  int   num_octomap_frontier_nodes;
  FrontierNode() {
  }
  FrontierNode(octomap::point3d pos, float r) {
    position = pos;
    radius   = r;
  }
  FrontierNode(octomap::point3d pos) {
    position = pos;
  }
};
//}

/* FrontierGroup //{ */
class FrontierGroup {
public:
  octomap::point3d position;
  octomap::point3d sum_position;
  /* octomap::point3d         normal; */
  FrontierExplorationPoint viable_fep;
  bool                     has_safe_exploration_point = false;
  int                      nearest_reachable_segment;
  float                    radius;
  int                      num_grouped_nodes       = 0;
  int                      num_grouped_groups      = 0;
  bool                     belongs_to_staging_area = false;
  int                      dbscan_group            = -2;

  float score;
  float probability_unexplored_by_any_robot = 1;

  FrontierGroup() {
    nearest_reachable_segment = -1;
    /* normal                    = octomap::point3d(0, 0, 0); */
  }
  FrontierGroup(octomap::point3d pos) {
    nearest_reachable_segment = -1;
    position                  = pos;
    sum_position              = pos;
    /* normal                    = octomap::point3d(0, 0, 0); */
  }
  void addNode(FrontierNode node) {
    /* normal += node.normal; */
    sum_position += node.position;
    /* num_grouped_nodes += 1; */
    num_grouped_nodes += node.num_octomap_frontier_nodes;
    num_grouped_groups += 1;
  }
};
//}

/* SegMap //{ */
class SegMap {
public:
  bool                          verbose                     = false;
  int                           last_seg_id                 = -1;
  bool                          initialized_segment_octree_ = false;
  octomap::point3d              home_position_;
  bool                          initialized_home_position = false;
  std::vector<octomap::point3d> comm_nodes;
  std::vector<FrontierGroup>    frontier_groups_           = {};
  StagingAreaSettings*          staging_area_settings_ptr_ = NULL;
  bool                          initialized_staging_area   = false;
  bool                          left_staging_area          = false;
  TopologyMappingSettings       topology_mapping_settings_;
  std::vector<int>              segments_with_some_frontier_group_ = {};  // ONLY FOR DECOMPRESSING SENT MSG

  float octomap_resolution;
  float max_segment_radius_      = 100;
  float voxel_min_safe_distance_ = 0.8;

  void                                initializeOctomaps(std::shared_ptr<octomap::OcTree> occupancy_octree_);
  void                                initializeOctomaps(float resolution, float safedist);
  void                                update(octomap::point3d current_position_, float current_heading_, std::shared_ptr<octomap::OcTree> occupancy_octree_,
                                             std::shared_ptr<PCLMap> pcl_map_ptr, std::vector<int>& segs_to_update);
  static std::shared_ptr<SegMap> convertFragmentMsgsToSegmap(std::vector<SegmapMsg>, tf2_ros::Buffer* tfbuf, std::string segmap_frame,
                                                             std::string shared_frame, bool use_shared_frame=false);
  std::vector<std::pair<float, int>> calculatePossibleSegmentsOfPointInReceivedSegmap(octomap::point3d point);
  float                              getProbabilityPointBelongsToSomeSegment(octomap::point3d point);
  void                               updateExplorednessProbabilitiesBasedOnOtherSegmap(std::shared_ptr<SegMap> segmap);
  void                               updateExplorednessProbabilitiesBasedOnOtherSegmaps(std::vector<std::shared_ptr<SegMap>>& segmaps);

  octomap::Segment* getSegmentPtrBBXsearch(octomap::point3d test_point);
  void              recalculateDistsFromHome();


  void updateWithExecutedTrajectory(std::vector<octomap::point3d>* traj, std::shared_ptr<octomap::OcTree> occupancy_octree_, float len_from_end);
  void getSegmentsIntersectingBBX(std::vector<int>& seg_ids, BoundingBox bbx);
  void getSegmentsAndBBXForVPCalc(std::vector<int>& seg_ids, BoundingBox bbx_in, BoundingBox& bbx_out);
  int checkNarrowPassagesNearSegments(BoundingBox bbx, std::shared_ptr<octomap::OcTree> occupancy_octree_, std::shared_ptr<PCLMap> pcl_map_ptr);

  std::optional<octomap::Segment*> getHomeSegmentPtr();

  std::shared_ptr<octomap::SegmentOcTree> segment_octree_;
  int                                     segment_octree_depth_offset_;

  std::mutex* mutex_ptr_;

  SegMap(float _segment_octree_depth_offset, StagingAreaSettings* staging_ptr, float octomap_resolution = 0.2, float safe_distance = 0.8,
         std::mutex* mutex_ptr = NULL) {
    voxel_min_safe_distance_     = safe_distance;
    mutex_ptr_                   = mutex_ptr;
    segment_octree_depth_offset_ = _segment_octree_depth_offset;
    initialized_segment_octree_  = false;
    comm_nodes                   = {};
    staging_area_settings_ptr_   = staging_ptr;
    initialized_staging_area     = false;
    if (staging_ptr == NULL || !staging_ptr->enabled) {
      initialized_staging_area = true;
    }
    initializeOctomaps(octomap_resolution, safe_distance);
  }

  SegMap(SegmapMsg msg);
};


//}
};  // namespace spheremap_server

#endif
