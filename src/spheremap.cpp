#include <spheremap_server/mapper.h>
#include <spheremap_server/utility_functions.h>
/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <random>
#include <spheremap_server/spheremap.h>

namespace spheremap_server
{

/* SphereMap //{ */
SphereMap::SphereMap(float _min_safe_dist, float _base_safe_dist, TopologyMappingSettings topo_settings, StagingAreaSettings* _staging_area_settings) {
  min_safe_dist                   = _min_safe_dist;
  planning_min_safe_dist          = _min_safe_dist;
  planning_base_safe_dist         = _base_safe_dist;
  staging_area_settings_ptr_      = _staging_area_settings;
  topology_mapping_settings_      = topo_settings;
  largest_bounging_sphere_radius_ = topo_settings.merged_segment_max_size_;
  nodes                           = std::shared_ptr<octomap::SphereMapOcTree>(new octomap::SphereMapOcTree(0.2));
}
//}

/* void SphereMap::updateNearestBestObstacleDist() //{ */
void SphereMap::updateNearestBestObstacleDist(octomap::point3d pos, float box_halfsize) {
  float max_odist = 0;

  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys       = getMaxSearchBBXBorderKeys(pos, box_halfsize);
  bool                                              found_near_key = false;
  octomap::OcTreeKey                                nearest_key;
  float                                             nearest_dist;
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
    if (it->valuePtr()->radius > max_odist) {
      max_odist = it->valuePtr()->radius;
    }
    found_near_key = true;
  }
  if (found_near_key) {
    nearest_best_obstacle_dist_ = max_odist;
  } else {
    nearest_best_obstacle_dist_ = enforced_max_sphere_radius;
  }
}
//}

/* bool SphereMap::areConnectable() //{ */
bool SphereMap::areConnectable(float dist2, float radius1, float radius2) {
  return dist2 < pow(radius1 + radius2 - sphere_intersection_reserve, 2);
}
//}

/* bool SphereMap::isPinnedBy() //{ */
bool SphereMap::isPinnedBy(float dist2, octomap::point3d smaller_pos, octomap::point3d larger_pos, float smaller_rad, float larger_rad) {
  float pinning_dist_mod   = 2;
  float pinning_size_mod   = 2;
  float remainder_dist_mod = 1;
  if (smaller_rad * pinning_size_mod > larger_rad) {
    return false;
  }
  if (dist2 < pow(larger_rad * pinning_dist_mod, 2)) {
    octomap::point3d hit_point;
    bool             ray_res        = current_octree_->castRay(larger_pos, smaller_pos - larger_pos, hit_point);
    float            ray_dist       = (hit_point - larger_pos).norm();
    float            remainder_dist = ray_dist - sqrt(dist2) - smaller_rad;  // after leaving smaller node
    if (remainder_dist > 0 && remainder_dist < smaller_rad * remainder_dist_mod) {
      return true;
    }
  }
  return false;
}
//}

/* std::pair<octomap::OcTreeKey, octomap::OcTreeKey> SphereMap::getMaxSearchBBXBorderKeys() //{ */
std::pair<octomap::OcTreeKey, octomap::OcTreeKey> SphereMap::getMaxSearchBBXBorderKeys(octomap::point3d pos) {
  return getMaxSearchBBXBorderKeys(pos, nearest_best_obstacle_dist_);
}
std::pair<octomap::OcTreeKey, octomap::OcTreeKey> SphereMap::getMaxSearchBBXBorderKeys(octomap::point3d pos, float box_halfsize) {
  BoundingBox bbx(box_halfsize, pos);
  return std::make_pair(nodes->coordToKey(octomap::point3d(bbx.x1, bbx.y1, bbx.z1), 16), nodes->coordToKey(octomap::point3d(bbx.x2, bbx.y2, bbx.z2), 16));
}
std::pair<octomap::OcTreeKey, octomap::OcTreeKey> SphereMap::getMaxSearchBBXBorderKeys(BoundingBox& bbx) {
  return std::make_pair(nodes->coordToKey(octomap::point3d(bbx.x1, bbx.y1, bbx.z1), 16), nodes->coordToKey(octomap::point3d(bbx.x2, bbx.y2, bbx.z2), 16));
}
//}

/* SphereMap::addNode() //{ */
std::pair<octomap::OcTreeKey, bool> SphereMap::addNode(octomap::SphereMapNodeData data) {
  octomap::OcTreeKey      key      = nodes->coordToKey(data.pos, 16);
  octomap::SphereMapNode* node_ptr = nodes->touchNode(key, 16);
  if (node_ptr->valuePtr()->radius >= 0) {  // TODO add initialized flag
    /* NODE WAS ALREADY THERE */
    ROS_WARN_THROTTLE(1, "trying to add spheremap node to voxel with one already");
    return std::make_pair(key, false);
  }
  node_ptr->setValue(data);
  return std::make_pair(key, true);
}
//}

/* bool SphereMap::removeNode(octomap::OcTreeKey key) //{ */
bool SphereMap::removeNode(octomap::OcTreeKey key) {
  nodes->deleteNode(key, 16);
  return true;
}
//}

/* void SphereMap::updateConnectionsForNode() //{ */
void SphereMap::updateConnectionsForNode(octomap::OcTreeKey key, bool check_connected_nodes) {
  octomap::SphereMapNode* node_ptr = nodes->search(key, 16);
  if (node_ptr == NULL) {
    ROS_ERROR("update connection error");
    return;
  }

  octomap::point3d deltavec;
  octomap::point3d pos = node_ptr->valuePtr()->pos;
  float            dist2;

  std::vector<octomap::OcTreeKey> old_connected   = node_ptr->valuePtr()->connected_keys;
  std::vector<octomap::OcTreeKey> now_connected   = {};
  std::vector<octomap::OcTreeKey> newly_connected = {};

  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys = getMaxSearchBBXBorderKeys(pos);
  std::vector<octomap::SphereMapNode*>              check_node_ptrs;
  std::vector<octomap::OcTreeKey>                   check_node_keys;

  /* GET KEYS BASED ON METHOD STYLE */
  if (check_connected_nodes) {
    for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
      if (it.getKey() == key) {
        continue;
      }
      check_node_ptrs.push_back(&(*it));

      check_node_keys.push_back(it.getKey());
    }
  } else {
    for (octomap::OcTreeKey conn_key : old_connected) {
      check_node_ptrs.push_back(nodes->search(conn_key, 16));
      check_node_keys.push_back(conn_key);
    }
  }

  /* ITERATE OVER OLD CONNECTED (OR OVER NEARBY NODES AND CHECK CONNECTIVITY) AND CHECK IF SHOULD DELETE NODE BECAUSE OF THEM */
  for (uint i = 0; i < check_node_ptrs.size(); i++) {
    octomap::SphereMapNode* it = check_node_ptrs[i];
    deltavec                   = pos - it->valuePtr()->pos;
    dist2                      = deltavec.dot(deltavec);

    if (!check_connected_nodes || areConnectable(dist2, node_ptr->valuePtr()->radius, it->valuePtr()->radius)) {
      now_connected.push_back(check_node_keys[i]);

      /* CHECK IF NOT REPLACABLE */
      /* ONLY GET KILLED BY LARGER AND NOT STALE */
      if (!it->valuePtr()->is_stale && it->valuePtr()->radius >= node_ptr->valuePtr()->radius - sphere_intersection_reserve) {
        /* CHECK MINIMAL DISTANCE */
        if (dist2 < pow(fmax(sphere_intersection_reserve, it->valuePtr()->radius / 1.5), 2)) {
          /* CHECK IF CONNECTED NODES FORM A FULL GRAPH */

          /* IF ALL OTHERS CONNECTED, DELETE NODE */
          if (connectedNodesFormFullGraph(node_ptr)) {
            debug_points.push_back(pos);
            node_ptr->valuePtr()->is_stale = true;
          }
        }
      }
    }

    /* ALSO CHECK IF NOT PINNED BY THIS NODE (IF NOT ALREADY MADE STALE) */
    if (!node_ptr->valuePtr()->is_stale && !it->valuePtr()->is_stale && it->valuePtr()->segment_id == node_ptr->valuePtr()->segment_id) {
      if (isPinnedBy(dist2, node_ptr->valuePtr()->pos, it->valuePtr()->pos, node_ptr->valuePtr()->radius, it->valuePtr()->radius)) {
        if (connectedNodesFormFullGraph(node_ptr)) {
          node_ptr->valuePtr()->is_stale = true;
        }
      }
    }
  }

  if (!check_connected_nodes) {
    return;
  }

  for (uint i = 0; i < old_connected.size(); i++) {
    if (std::find(now_connected.begin(), now_connected.end(), old_connected[i]) == now_connected.end()) {
      /* WAS CONNECTED, IS NOT */
      octomap::SphereMapNode* old_ptr       = nodes->search(old_connected[i], 16);
      auto                    backwards_ptr = std::find(old_ptr->valuePtr()->connected_keys.begin(), old_ptr->valuePtr()->connected_keys.end(), key);
      if (backwards_ptr == old_ptr->valuePtr()->connected_keys.end()) {
        ROS_ERROR("backw error");
      }
      old_ptr->valuePtr()->connected_keys.erase(backwards_ptr);
    }
  }
  for (uint i = 0; i < now_connected.size(); i++) {
    if (std::find(old_connected.begin(), old_connected.end(), now_connected[i]) == old_connected.end()) {
      /* WAS NOT CONNECTED, NOW IS */
      octomap::SphereMapNode* new_ptr = nodes->search(now_connected[i], 16);
      new_ptr->valuePtr()->connected_keys.push_back(key);
    }
  }
  /* WHAT ABOUT NEWLY CONNECTED? */

  node_ptr->valuePtr()->connected_keys = now_connected;
}
//}


/* bool                                SphereMap::connectedNodesFormFullGraph() //{ */
bool SphereMap::connectedNodesFormFullGraph(octomap::SphereMapNode* node_ptr) {
  std::vector<octomap::SphereMapNode*> node_ptr_vec;

  if (node_ptr->valuePtr()->connected_keys.size() > 1) {
    for (octomap::OcTreeKey g_conn_key : node_ptr->valuePtr()->connected_keys) {
      auto node2_ptr = nodes->search(g_conn_key);
      node_ptr_vec.push_back(node2_ptr);
      node2_ptr->valuePtr()->magic_flag2 = true;
    }
    /* PERFORM FLOODFILL */
    std::vector<octomap::SphereMapNode*> frontier   = {node_ptr_vec[0]};
    uint                                 num_popped = 0;
    while (!frontier.empty()) {
      octomap::SphereMapNode* g_node_ptr = (*frontier.begin());
      frontier.erase(frontier.begin());
      num_popped++;
      g_node_ptr->valuePtr()->magic_flag2 = false;

      for (octomap::OcTreeKey g_conn_key : g_node_ptr->valuePtr()->connected_keys) {
        octomap::SphereMapNode* adj_g_node_ptr = nodes->search(g_conn_key);
        if (adj_g_node_ptr->valuePtr()->magic_flag2) {
          adj_g_node_ptr->valuePtr()->magic_flag2 = false;
          frontier.push_back(adj_g_node_ptr);
        }
      }
    }

    /* CHECK IF ALL WERE FILLED */
    if (num_popped != node_ptr->valuePtr()->connected_keys.size()) {
      /* CLEAN UP */
      for (uint i = 0; i < node_ptr_vec.size(); i++) {
        node_ptr_vec[i]->valuePtr()->magic_flag2 = false;
      }
      return false;
    }
  }

  /* IF ALL OTHERS CONNECTED, DELETE NODE */
  return true;
}
//}

/* void SphereMap::updateObstacleDists() //{ */
void SphereMap::updateObstacleDists(std::shared_ptr<PCLMap> pcl_map_ptr, octomap::point3d box_pos, float box_halfsize) {
  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys = getMaxSearchBBXBorderKeys(box_pos, box_halfsize);
  ros::Time                                         now_time = ros::Time::now();

  latest_unsafe_node_keys_ = {};

  /* ROS_INFO("STARTING UPDATING DISTS"); */
  uint num_nodes = 0;
  /* octomap::OcTreeNode*                              ocnode; */

  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
    num_nodes++;
    float new_odist = getObstacleDist(it->valuePtr()->pos, pcl_map_ptr);
    /* if (new_odist < min_safe_dist) { */
    /* ocnode                        = occupancy_octree_->search(sample_point, 16); */
    /* if (ocnode == NULL || occupancy_octree_->isNodeOccupied(ocnode)) { */
    /*   continue; */
    /* } */
    if (new_odist < min_safe_dist) {
      it->valuePtr()->is_safe = false;
      latest_unsafe_node_keys_.push_back(it.getKey());
    } else {
      it->valuePtr()->is_safe = true;
    }

    if (it->valuePtr()->radius > new_odist) {
      /* ROS_INFO("[SphereMap]: lowered radius from %f to %f", it->second.radius, new_odist); */
    } else if (it->valuePtr()->radius < new_odist) {
      /* ROS_INFO("[SphereMap]: increased radius from %f to %f", it->second.radius, new_odist); */
    }
    float new_rad = fmin(new_odist, enforced_max_sphere_radius);

    updateConnectionsForNode(it.getKey());
    if (new_rad != it->valuePtr()->radius) {
      it->valuePtr()->radius = new_rad;

      if (std::find(changed_segments.begin(), changed_segments.end(), it->valuePtr()->segment_id) == changed_segments.end()) {
        changed_segments.push_back(it->valuePtr()->segment_id);
      }
    }
  }

  /* ROS_INFO("NUM SPHERES: %d", num_nodes); */
}
//}

/* bool SphereMap::isPointInMap(octomap::point3d) //{ */
bool SphereMap::isPointInMap(octomap::point3d test_point) {
  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys = getMaxSearchBBXBorderKeys(test_point);
  float                                             rad;
  float                                             dist2;
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
    dist2 = (it->valuePtr()->pos - test_point).norm_sq();
    rad   = it->valuePtr()->radius;
    if (dist2 < pow(rad - sphere_intersection_reserve, 2)) {
      return true;
    }
  }
  return false;
}

bool SphereMap::isPointInMap(octomap::point3d test_point, float box_halfsize) {
  octomap::point3d deltavec;

  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys = getMaxSearchBBXBorderKeys(test_point, box_halfsize);
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
    deltavec = it->valuePtr()->pos - test_point;
    if (deltavec.dot(deltavec) < pow(it->valuePtr()->radius - sphere_intersection_reserve, 2)) {
      return true;
    }
  }
  return false;
}
//}

/* void SphereMap::update() //{ */
void SphereMap::update(octomap::point3d current_position_, float current_heading_, std::shared_ptr<octomap::OcTree> occupancy_octree_,
                       std::shared_ptr<PCLMap> pcl_map_ptr) {
  /* ROS_INFO("update start"); */
  debug_points    = {};
  current_octree_ = occupancy_octree_;

  ros::WallTime start_, end_;
  start_ = ros::WallTime::now();
  float update_exec_time;
  float expansion_exec_time;
  float segmentation_exec_time;
  potential_merge_pairs_     = {};
  changed_segments           = {};
  segments_with_purged_nodes = {};

  /* UPDATE DISTS OF NEAR NODES */
  updateObstacleDists(pcl_map_ptr, current_position_, max_update_box_size_ / 2);
  /* ROS_INFO("[SphereMapUpdate]: max_update_box_size: %f", max_update_box_size_); */
  purgeNodesSimple(current_position_, max_update_box_size_ / 2, true);
  segments_with_purged_nodes = {};
  /* MUST PURGE AND UPDATE SIMULTANEOUSLY, CANT DELETE ALL AT END!!! */


  /* updateNearestBestObstacleDist(current_position_, 50); */
  end_             = ros::WallTime::now();
  update_exec_time = (end_ - start_).toSec() * 1000;
  start_           = ros::WallTime::now();


  /* HANDLE STAGING AREA */
  if (staging_area_settings_ptr_ != NULL && staging_area_settings_ptr_->enabled && !left_staging_area) {
    ROS_INFO("IN STAGING AREA");
    octomap::point3d deltavec  = current_position_ - staging_area_settings_ptr_->wall_pos;
    float            wall_proj = deltavec.dot(staging_area_settings_ptr_->wall_dir_outward);
    /* INITIALIZE AND EXPAND STAGING AREA SEGMENT */
    expansionsStep(current_position_, occupancy_octree_, pcl_map_ptr);
    purgeNodesSimple(current_position_, 0.8 * max_update_box_size_ / 2, true);
    if (next_seg_id == 1) {
      /* START SEG FROM FIRST NODE BEFORE STAGING AREA */
      bool found_good_spot = false;
      for (octomap::SphereMapOcTree::leaf_iterator it = nodes->begin_leafs(16); it != nodes->end_leafs(); it++) {
        deltavec        = it->valuePtr()->pos - staging_area_settings_ptr_->wall_pos;
        float wall_proj = deltavec.dot(staging_area_settings_ptr_->wall_dir_outward);
        if (wall_proj < 1) {
          found_good_spot = true;
          growSegment(it.getKey());
          break;
        }
      }
      if (!found_good_spot) {
        ROS_WARN("[SphereMap]: waiting for at least one node of spheremap behind the staging area wall, cannot init staging area segment without it");
        return;
      }
    } else {
      spreadSegment(1);
      /* segmentationStep(current_position_, occupancy_octree_, pcl_map_ptr); */
    }
    if (wall_proj > 0) {
      left_staging_area = true;
      ROS_INFO("[SphereMap]: robot has left the staging area");
    } else {
      return;
    }
  }

  /* EXPAND */
  /* ROS_INFO("[SphereMap]: expansion step"); */
  expansionsStep(current_position_, occupancy_octree_, pcl_map_ptr);
  end_                = ros::WallTime::now();
  expansion_exec_time = (end_ - start_).toSec() * 1000;
  start_              = ros::WallTime::now();

  /* ROS_INFO("[SphereMap]: purge step 2"); */
  purgeNodesSimple(current_position_, 0.8 * max_update_box_size_ / 2, true);

  /* SEGMENT */
  /* ROS_INFO("[SphereMap]: segmentation step"); */
  segmentationStep(current_position_, occupancy_octree_, pcl_map_ptr);

  end_                   = ros::WallTime::now();
  segmentation_exec_time = (end_ - start_).toSec() * 1000;
  ROS_INFO("[SphereMap]: spheremap update time: %f ms (%f / %f / %f)", update_exec_time + expansion_exec_time + segmentation_exec_time, update_exec_time,
           expansion_exec_time, segmentation_exec_time);

  /* TEST ASTAR TO HOME */

  /* uint          current_seg_id = nodes->search(nearest_node_query.value().second)->valuePtr()->segment_id; */
  /* uint          home_seg_id    = nodes->search(goal_nearest_node_query.value().second)->valuePtr()->segment_id; */
  /* SphereMapPath home_path      = computePath(current_seg_id, current_position_, home_seg_id, home_position_); */
  /* if (home_path.reaches_goal) { */
  /* ROS_INFO("[SphereMap]: path to home exists and has len: %f m", home_path.getApproxTravelDistance()); */
  /* } else { */
  /* ROS_INFO("[SphereMap]: path to home does not exist!"); */
  /* } */

  /* updateFrontiers(false, current_position_, 50, occupancy_octree_); */
}
//}

/* EXPANSION */

/* void SphereMap::growNodesSimple() //{ */
void SphereMap::growNodesSimple(octomap::point3d current_position_, float box_halfsize, std::shared_ptr<octomap::OcTree> occupancy_octree_,
                                std::shared_ptr<PCLMap> pcl_map_ptr) {
  uint                                              num_sampled_per_node = 10;
  float                                             radius_delta_minus   = 0.1;
  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys             = getMaxSearchBBXBorderKeys(current_position_, box_halfsize);
  octomap::OcTreeNode*                              ocnode;
  std::vector<octomap::OcTreeKey>                   nodes_for_connections_update;

  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
    float min_r   = it->valuePtr()->radius + radius_delta_minus;
    float delta_r = min_safe_dist / 4;
    for (uint i = 0; i < num_sampled_per_node; i++) {
      octomap::point3d sample_point = it->valuePtr()->pos + getRandomPointInSphere(min_r, delta_r);
      ocnode                        = occupancy_octree_->search(sample_point, 16);
      if (ocnode == NULL || occupancy_octree_->isNodeOccupied(ocnode)) {
        continue;
      }
      if (!isPointInMap(sample_point)) {
        float odist = getObstacleDist(sample_point, pcl_map_ptr);
        if (odist > min_safe_dist) {
          auto addition_res = addNode(octomap::SphereMapNodeData(sample_point, odist));
          if (addition_res.second) {
            /* ROS_INFO("ADDED SPHERE AT %f, %f, %f", sample_point.x(), sample_point.y(), sample_point.z()); */
            nodes_for_connections_update.push_back(addition_res.first);
          }
        }
      }
    }
  }

  for (uint i = 0; i < nodes_for_connections_update.size(); i++) {
    updateConnectionsForNode(nodes_for_connections_update[i]);
  }
}
//}

/* void SphereMap::growNodesFull() //{ */
void SphereMap::growNodesFull(octomap::point3d current_position_, float box_halfsize, std::shared_ptr<octomap::OcTree> occupancy_octree_,
                              std::shared_ptr<PCLMap> pcl_map_ptr) {
  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys = getMaxSearchBBXBorderKeys(current_position_, box_halfsize);

  ros::WallTime time_start, time_end;
  time_start = ros::WallTime::now();

  std::vector<octomap::OcTreeKey> nodes_for_connections_update;

  /* GET POINTS */
  std::vector<octomap::point3d>   saved_positions;
  std::vector<octomap::OcTreeKey> saved_keys;
  std::vector<float>              saved_odists;
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
    saved_positions.push_back(it->valuePtr()->pos);
    saved_odists.push_back(it->valuePtr()->radius);
    saved_keys.push_back(it.getKey());
  }

  /* LOOK */
  octomap::point3d                                engulfed_node_position;
  std::vector<std::pair<float, octomap::point3d>> safepoints;
  uint                                            num_engulfing = 0;
  std::vector<octomap::point3d>                   testpoints;
  std::vector<std::pair<float, uint>>             testpoint_pairs;

  BoundingBox near_points_box(box_halfsize, current_position_);
  float       near_points_filtering_dist2 = pow(1, 2);
  float       testpoint_filtering_dist2   = pow(2.2, 2);
  uint        num_sampled_by_raycasts     = 0;
  uint        num_sampled_by_nearness     = 0;

  uint sample_index = 0;

  /* ADD SAMPLEPOINTS FROM RAYCASTING */
  float interval_len = 1;
  for (uint i = 0; i < 50; i++) {
    octomap::point3d dir = getRandomPointInSphere(1).normalized();

    octomap::point3d hit_point;
    occupancy_octree_->castRay(current_position_, dir, hit_point, false, box_halfsize);
    float ray_len = (hit_point - current_position_).norm();

    float last_odist = 0;
    float len        = 0;
    while (len < ray_len) {
      len += fmax(interval_len, last_odist - sphere_intersection_reserve);
      octomap::point3d sample_point = current_position_ + dir * len;
      num_sampled_by_raycasts++;

      /* bool should_filter = false; */
      /* for (uint k = 0; k < testpoints.size(); k++) { */
      /*   if ((sample_point - testpoints[k]).norm_sq() < testpoint_filtering_dist2) { */
      /*     should_filter = true; */
      /*     break; */
      /*   } */
      /* } */
      /* if (should_filter) { */
      /*   break; */
      /* } */

      octomap::OcTreeNode* nodeptr = occupancy_octree_->search(sample_point);
      if (nodeptr == NULL || occupancy_octree_->isNodeOccupied(nodeptr)) {
        break;
      }

      float odist = getObstacleDist(sample_point, pcl_map_ptr);
      if (odist < min_safe_dist) {
        break;
      }
      last_odist = odist;
      testpoint_pairs.push_back(std::make_pair(-odist, sample_index));
      testpoints.push_back(sample_point);
      sample_index++;
    }
  }

  for (octomap::OcTree::leaf_bbx_iterator it = occupancy_octree_->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != occupancy_octree_->end_leafs_bbx();
       it++) {

    if (occupancy_octree_->isNodeOccupied(*it)) {
      continue;
    }
    if (it.getDepth() == 16) {
      continue;
    }
    /* octomap::point3d sample_point = it.getCoordinate() + getRandomPointInSphere(0.1 * pow(2, 16 - it.getDepth())); */
    float            leaf_voxel_half_sidelength = (0.1 * pow(2, 16 - it.getDepth()));
    octomap::point3d rand_point =
        octomap::point3d(rand() / (float)RAND_MAX - 0.5, rand() / (float)RAND_MAX - 0.5, rand() / (float)RAND_MAX - 0.5) * (2 * leaf_voxel_half_sidelength);
    octomap::point3d sample_point = it.getCoordinate() + rand_point;
    num_sampled_by_nearness++;


    float maxd2 = testpoint_filtering_dist2;
    if (near_points_box.isPointInside(sample_point)) {
      maxd2 = near_points_filtering_dist2;
    }

    bool should_filter = false;
    for (uint i = 0; i < testpoints.size(); i++) {
      if ((sample_point - testpoints[i]).norm_sq() < maxd2) {
        should_filter = true;
        break;
      }
    }
    if (should_filter) {
      continue;
    }

    float odist = getObstacleDist(sample_point, pcl_map_ptr);
    if (odist < min_safe_dist) {
      continue;
    }
    odist = fmin(odist, enforced_max_sphere_radius);

    testpoint_pairs.push_back(std::make_pair(-odist, sample_index));
    testpoints.push_back(sample_point);
    sample_index++;
  }


  bool debug_sampling = false;
  /* std::sort(testpoint_pairs.begin(), testpoint_pairs.end()); */
  for (uint i = 0; i < testpoint_pairs.size(); i++) {
    octomap::point3d sample_point = testpoints[testpoint_pairs[i].second];
    /* FILTER OUT POINTS IN DENY BBX */
    if (generation_deny_bbx.isPointInside(sample_point)) {
      ROS_INFO_COND(debug_sampling, "a");
      continue;
    }

    float                           odist = -testpoint_pairs[i].first;
    std::vector<octomap::OcTreeKey> ckeys = {};

    if (nodes->search(sample_point, 16) != NULL) {
      ROS_INFO_COND(debug_sampling, "x");
      continue;
    }

    bool is_in_map = false;

    /* CHECK IF CAN ADD */
    /* CONDITIONSS */
    for (uint j = 0; j < saved_positions.size(); j++) {
      float dist2 = (sample_point - saved_positions[j]).norm_sq();
      if (odist < saved_odists[j] - sphere_intersection_reserve && dist2 < pow(fmax(sphere_intersection_reserve, saved_odists[j] / 1.5), 2)) {
        debug_points.push_back(sample_point);
        is_in_map = true;
        ROS_INFO_COND(debug_sampling, "b");
        break;
      }
      /* if (odist < saved_odists[j] && dist2 < pow(saved_odists[j] - 2 * sphere_intersection_reserve, 2)) { */
      /*   is_in_map = true; */
      /*   break; */
      /* } */

      if (isPinnedBy(dist2, sample_point, saved_positions[j], odist, saved_odists[j])) {
        ROS_INFO_COND(debug_sampling, "c");
        is_in_map = true;
        break;
      }

      if (areConnectable(dist2, odist, saved_odists[j])) {
        ckeys.push_back(saved_keys[j]);
      }
    }
    if (is_in_map) {
      ROS_INFO_COND(debug_sampling, "e");
      continue;
    }

    octomap::SphereMapNodeData data_to_add(sample_point, odist);
    data_to_add.connected_keys = ckeys;  // THIS OK

    std::pair<octomap::OcTreeKey, bool> addition_res = addNode(data_to_add);
    if (addition_res.second) {
      /* ROS_INFO("ADDED SPHERE AT %f, %f, %f", sample_point.x(), sample_point.y(), sample_point.z()); */
      octomap::OcTreeKey new_key = addition_res.first;
      for (uint j = 0; j < ckeys.size(); j++) {
        nodes->search(ckeys[j], 16)->valuePtr()->connected_keys.push_back(new_key);
      }
      saved_positions.push_back(sample_point);
      saved_odists.push_back(odist);
      saved_keys.push_back(new_key);
      nodes_for_connections_update.push_back(new_key);

      nodes->search(new_key, 16)->valuePtr()->connected_keys = ckeys;

      /* ADD CONNECTION FOR CONNECTED NODES */
    }
  }


  time_end = ros::WallTime::now();
  ROS_INFO(
      "[SphereMap]: expansion time: %f.  testpoints size: %lu, points from raycasting: %lu, points from sampling octomap free voxels: %lu, tespoints total: "
      "%lu, added spheres in total: %lu",
      (time_end - time_start).toSec() * 1000, testpoints.size(), num_sampled_by_raycasts, num_sampled_by_nearness, testpoints.size(),
      nodes_for_connections_update.size());
  time_start = ros::WallTime::now();

  for (uint i = 0; i < nodes_for_connections_update.size(); i++) {
    updateConnectionsForNode(nodes_for_connections_update[i], false);
  }

  time_end = ros::WallTime::now();
  /* ROS_INFO("[SphereMap]: second updateconn time: %f", (time_end - time_start).toSec() * 1000); */
  /* checkIntegrity(); */
}
//}

/* void SphereMap::expansionsStep() //{ */
void SphereMap::expansionsStep(octomap::point3d current_position_, std::shared_ptr<octomap::OcTree> occupancy_octree_, std::shared_ptr<PCLMap> pcl_map_ptr) {
  /* CHECK IF CURRENT POS IN MAP */
  /* if (!isPointInMap(current_position_)) { */
  /*   float odist = getObstacleDist(current_position_, pcl_map_ptr); */
  /*   if (odist > min_safe_dist) { */
  /*     auto addition_res = addNode(octomap::SphereMapNodeData(current_position_, odist)); */
  /*     /1* if (addition_res.second) { *1/ */
  /*     /1*   updateConnectionsForNode(addition_res.first); *1/ */
  /*     /1*   ROS_INFO("[SphereMap]: grow segment at robot's position"); *1/ */
  /*     /1*   uint grow_id = growSegment(addition_res.first); *1/ */
  /*     /1* } *1/ */
  /*   } */
  /* } */

  /* growNodesSimple(current_position_, 10, occupancy_octree_, pcl_map_ptr); */
  /* growNodesSimple(current_position_, 30, occupancy_octree_, pcl_map_ptr); */
  /* ROS_INFO("[SphereMap]: expanding nodes"); */
  growNodesFull(current_position_, max_update_box_size_ / 2, occupancy_octree_, pcl_map_ptr);
  /* ROS_INFO("[SphereMap]: expanded nodes"); */
}
//}

/* PRUNING */

/* void SphereMap::purgeNodesSimple() //{ */
void SphereMap::purgeNodesSimple(octomap::point3d current_position_, float box_halfsize, bool perform_connectedness_check) {
  /* std::vector<octomap::OcTreeKey> nodes_to_purge = latest_unsafe_node_keys_; */
  std::vector<octomap::OcTreeKey> nodes_to_purge = {};

  /* checkIntegrity(); */

  uint num_split_segs = 0;

  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys = getMaxSearchBBXBorderKeys(current_position_, box_halfsize);
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
    /* float dist2_from_uav = (it->valuePtr()->pos - current_position_).norm_sq(); */
    /* if (dist2_from_uav < 2) { */
    /*   continue; */
    /* } */
    /* if (!it->valuePtr()->is_safe || (it->valuePtr()->connected_keys.size() > 5 && it->valuePtr()->radius > 1.2)) { */
    if (!it->valuePtr()->is_safe || (it->valuePtr()->is_stale && !it->valuePtr()->is_frontier)) {
      nodes_to_purge.push_back(it.getKey());
    }
  }

  uint num_purged = 0;
  for (octomap::OcTreeKey key_to_delete : nodes_to_purge) {
    octomap::SphereMapNode* node_ptr = nodes->search(key_to_delete, 16);

    uint seg_id = node_ptr->valuePtr()->segment_id;
    if (seg_id > 0) {
      segments_with_purged_nodes.insert(seg_id);
    }

    /* REMOVE CONNECTIONS OF OTHER NODES TO THIS NODE*/
    /* ROS_INFO("pruning key %d,%d,%d, node connections: %lu", key_to_delete[0],key_to_delete[1],key_to_delete[2], node_ptr->valuePtr()->connected_keys.size());
     */
    for (octomap::OcTreeKey connected_key : node_ptr->valuePtr()->connected_keys) {
      octomap::SphereMapNode* connected_node_ptr = nodes->search(connected_key, 16);
      uint                    connected_seg_id   = connected_node_ptr->valuePtr()->segment_id;
      if (connected_seg_id > 0) {
        segments_with_purged_nodes.insert(connected_seg_id);
      }

      /* ROS_INFO("adjacent node %d,%d,%d connections: %lu", connected_key[0],connected_key[1],connected_key[2],
       * connected_node_ptr->valuePtr()->connected_keys.size()); */
      std::vector<octomap::OcTreeKey>::iterator backwards_conn_ptr =
          std::find(connected_node_ptr->valuePtr()->connected_keys.begin(), connected_node_ptr->valuePtr()->connected_keys.end(), key_to_delete);
      if (backwards_conn_ptr == connected_node_ptr->valuePtr()->connected_keys.end()) {
        /* ONLY CONNECTED ONE WAY? */
        ROS_ERROR("node pruning error");
        continue;
      }
      /* ROS_INFO("deleted connection"); */
      connected_node_ptr->valuePtr()->connected_keys.erase(backwards_conn_ptr);

      backwards_conn_ptr =
          std::find(connected_node_ptr->valuePtr()->connected_keys.begin(), connected_node_ptr->valuePtr()->connected_keys.end(), key_to_delete);
      if (backwards_conn_ptr != connected_node_ptr->valuePtr()->connected_keys.end()) {
        ROS_ERROR("secondary node pruning error");
      }
    }

    /* ROS_INFO("deleting node"); */
    nodes->deleteNode(key_to_delete, 16);
    num_purged++;
    /* if (nodes->search(key_to_delete, 16) != NULL) { */
    /*   ROS_ERROR("node was not deleted!"); */
    /* } */
  }

  /* ROS_INFO("purged %u nodes", num_purged); */
  /* checkIntegrity(); */

  /* DELETE ALL SEGMENTS WHICH HAD DELETED NDOES */
  if (perform_connectedness_check) {
    uint num_deleted = 0;
    for (uint seg_id : segments_with_purged_nodes) {
      /* ROS_INFO("deleting segment %u", seg_id); */
      std::vector<std::vector<octomap::OcTreeKey>> key_splits;
      uint                                         check_res = checkSegmentStateAfterKeyRemoval(seg_id, key_splits);
      if (key_splits.size() > 1 || check_res > 0) {
        num_split_segs++;
      }
      /* IF SEGMENT DESTROYED OR NOT FOUND, TRY TO DESTROY IT */
      if (check_res > 0 || key_splits.empty()) {
        removeSegment(seg_id);
        /* TODO remove connections!!! */
        num_deleted++;
        /* ELSE DIVIDE ACCORDING TO SPLITS */
      } else {
        /* GET BIGGEST SPLIT */
        uint biggest_split_index = 0;
        uint biggest_split_size  = 0;
        /* ROS_INFO("splits sizes:"); */
        for (uint i = 0; i < key_splits.size(); i++) {
          if (key_splits[i].size() > biggest_split_size) {
            biggest_split_size  = key_splits[i].size();
            biggest_split_index = i;
          }
          /* ROS_INFO("split: %lu", key_splits[i].size()); */
        }

        /* SET ORIGINAL SEGMENT TO HAVE LARGEST SPLIT OF KEYS */
        std::map<uint, SphereMapSegment>::iterator seg_ptr = segments.find(seg_id);

        /* if (seg_ptr->second.keys.size() == key_splits[biggest_split_index].size()) { */
        /*   /1* BUT SOME NODES COULD HAVE BEEN DISCONNECTED BY NODEUPDATE!!! THEREFORE THESE KEYS CAN BE DIFFERENT! *1/ */
        /*   updateConnectionsForSegment(seg_ptr); */
        /*   continue; */
        /* } */

        seg_ptr->second.keys = key_splits[biggest_split_index];
        if (std::find(changed_segments.begin(), changed_segments.end(), seg_id) == changed_segments.end()) {
          changed_segments.push_back(seg_id);
        }
        octomap::point3d              new_center(0, 0, 0);
        std::vector<octomap::point3d> seg_points = {};
        float                         new_bounding_sphere_radius2;
        for (uint j = 0; j < key_splits[biggest_split_index].size(); j++) {
          octomap::SphereMapNode* node_ptr = nodes->search(key_splits[biggest_split_index][j], 16);

          octomap::point3d node_pos = node_ptr->valuePtr()->pos;
          new_center += node_pos;
          seg_points.push_back(node_pos);
        }
        new_center = new_center * (1 / ((float)key_splits[biggest_split_index].size()));

        /* RADIUS */
        float max_bounding_radius2 = 0;
        for (uint j = 0; j < seg_points.size(); j++) {
          float dist2          = (seg_points[j] - new_center).norm_sq();
          max_bounding_radius2 = fmax(max_bounding_radius2, dist2);
        }
        float new_bounding_radius              = sqrt(max_bounding_radius2);
        seg_ptr->second.center                 = new_center;
        seg_ptr->second.bounding_sphere_radius = new_bounding_radius;

        updateConnectionsForSegment(seg_ptr);

        /* CREATE NEW SEGMENTS FROM OTHER SPLITS */
        for (uint i = 0; i < key_splits.size(); i++) {
          if (i == biggest_split_index) {
            continue;
          }
          for (uint j = 0; j < key_splits[i].size(); j++) {
            octomap::SphereMapNode* node_ptr = nodes->search(key_splits[i][j], 16);
            node_ptr->valuePtr()->segment_id = 0;
          }
        }
      }
    }
    /* ROS_INFO("[SphereMapUpdate]: purge deleted %u / %lu updated segments. Split segs: %d", num_deleted, segments_with_purged_nodes.size(), num_split_segs);
     */
  }
}
//}

/* bool SphereMap::removeSegment() //{ */
bool SphereMap::removeSegment(uint seg_id) {
  std::map<uint, SphereMapSegment>::iterator seg_ptr = segments.find(seg_id);
  if (seg_ptr == segments.end()) {
    ROS_ERROR("segment to delete does not exist");
    return false;
  }

  /* RESET SEGMENT NODES SEG_IDS */
  for (octomap::OcTreeKey key_of_old_segment : seg_ptr->second.keys) {
    octomap::SphereMapNode* node_ptr = nodes->search(key_of_old_segment, 16);
    if (node_ptr == NULL) {
      continue;
    }

    node_ptr->valuePtr()->segment_id = 0;
  }

  /* DELETE SEGMENT CONNECTIONS */
  for (std::pair<uint, SphereMapSegmentConnection> connection : seg_ptr->second.connections) {
    std::map<uint, SphereMapSegment>::iterator other_seg_ptr = segments.find(connection.first);
    /* if (other_seg_ptr == segments.end()) { */
    /*   ROS_ERROR("adjacent segment to delete does not exist"); */
    /*   continue; */
    /* } */

    /* REMOVE FROM OTHER SEGMENT */
    other_seg_ptr->second.connections.erase(other_seg_ptr->second.connections.find(seg_id));

    uint min_id = fmin(seg_id, other_seg_ptr->first);
    uint max_id = fmax(seg_id, other_seg_ptr->first);

    /* REMOVE FROM PORTALS */
    bool removed_portal = false;
    for (std::vector<std::pair<uint, uint>>::iterator portals_it = portals.begin(); portals_it != portals.end(); portals_it++) {
      if (portals_it->first == min_id && portals_it->second == max_id) {
        removed_portal = true;
        portals.erase(portals_it);
        break;
      }
    }
    if (!removed_portal) {
      ROS_WARN("[removeSegment]: portal removing error");
    }
  }

  /* ERASE SEGMENT */
  segments.erase(seg_ptr);
  return true;
}
//}

/* bool SphereMap::checkSegmentStateAfterKeyRemoval() //{ */
uint SphereMap::checkSegmentStateAfterKeyRemoval(uint seg_id, std::vector<std::vector<octomap::OcTreeKey>>& splits) {
  std::map<uint, SphereMapSegment>::iterator seg_ptr = segments.find(seg_id);
  if (seg_ptr == segments.end()) {
    ROS_ERROR("segment to check does not exist");
    return 1;
  }

  std::vector<octomap::SphereMapNode*> node_ptrs;
  /* GET NON-NULL NODE PTRS AND SET MAGIC FLAG */
  for (octomap::OcTreeKey key_of_segment : seg_ptr->second.keys) {
    octomap::SphereMapNode* node_ptr = nodes->search(key_of_segment, 16);
    if (node_ptr == NULL) {
      continue;
    }
    node_ptr->valuePtr()->magic_flag = false;
    node_ptrs.push_back(node_ptr);
  }

  /* IF WHOLE SEGMENT WAS DESTROYED, RETURN */
  if (node_ptrs.empty()) {
    return 1;
  }

  /* START FILL */
  uint split_index = 0;
  for (std::vector<octomap::SphereMapNode*>::iterator it = node_ptrs.begin(); it != node_ptrs.end(); it++) {
    if ((*it)->valuePtr()->magic_flag) {
      continue;
    }

    std::vector<octomap::SphereMapNode*> frontier = {*it};
    (*node_ptrs.begin())->valuePtr()->magic_flag  = true;
    splits.push_back({nodes->coordToKey((*it)->valuePtr()->pos, 16)});

    while (!frontier.empty()) {
      octomap::SphereMapNode* expanded_node = *frontier.erase(frontier.begin());

      for (octomap::OcTreeKey connection_key : expanded_node->valuePtr()->connected_keys) {
        octomap::SphereMapNode* connected_node_ptr = nodes->search(connection_key, 16);  // THIS CRASHES
        if (connected_node_ptr == NULL) {
          ROS_ERROR("wrong connected key!!! connected keys: %lu ", expanded_node->valuePtr()->connected_keys.size());
        }
        if (connected_node_ptr->valuePtr()->segment_id != seg_id) {
          continue;
        }
        if (connected_node_ptr->valuePtr()->magic_flag) {
          continue;
        }
        connected_node_ptr->valuePtr()->magic_flag = true;
        frontier.push_back(connected_node_ptr);
        splits[split_index].push_back(connection_key);
      }
    }

    split_index++;
  }

  /* ONE OR MORE SPLITS, SEGMENT NEEDS RECOMPUTING */
  /* if (splits.size() > 1) { */
  /*   return 0; */
  /* } */

  /* /1* SEGMENT WAS NOT RIPPED, CHECK IF CONNECTIONS EXIST STILL *1/ */
  /* for (std::map<uint, SphereMapSegmentConnection>::iterator conn_it = seg_ptr->second.connections.begin(); conn_it != seg_ptr->second.connections.end(); */
  /*      conn_it++) { */
  /* } */

  return 0;
}
//}

/* SEGMENTATION */

/* void SphereMap::segmentationStep() //{ */
void SphereMap::segmentationStep(octomap::point3d current_position_, std::shared_ptr<octomap::OcTree> occupancy_octree_, std::shared_ptr<PCLMap> pcl_map_ptr) {
  /* EXPAND ALREADY CREATED SEGMENTS */
  ros::WallTime starttime, endtime;
  starttime                         = ros::WallTime::now();
  std::vector<uint> nearby_segments = getSegmentsIntersectingBBX(current_position_, max_update_box_size_ / 2);
  /* ROS_INFO("have %u nearby segments", nearby_segments.size()); */
  for (uint i = 0; i < nearby_segments.size(); i++) {
    uint new_nodes_added = spreadSegment(nearby_segments[i]);
    if (new_nodes_added > 0) {
      if (std::find(changed_segments.begin(), changed_segments.end(), nearby_segments[i]) == changed_segments.end()) {
        changed_segments.push_back(nearby_segments[i]);
      }
    }
  }
  endtime = ros::WallTime::now();
  /* ROS_INFO("expansion of segs took %f", (endtime - starttime).toSec() * 1000); */
  starttime = ros::WallTime::now();

  /* GROW SEGMENTS FROM THOSE NODES THAT WERE NOT SEGMENTED */
  /* TODO get maximum bbx that could have been expanded */
  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys           = getMaxSearchBBXBorderKeys(current_position_, 1.2 * max_update_box_size_ / 2);
  uint                                              num_segments_grown = 0;
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
    if (it->valuePtr()->segment_id > 0) {
      continue;
    }

    uint grow_res = growSegment(it.getKey());
    if (grow_res > 0) {
      if (std::find(changed_segments.begin(), changed_segments.end(), grow_res) == changed_segments.end()) {
        changed_segments.push_back(grow_res);
      }
      num_segments_grown++;
    }
  }
  endtime = ros::WallTime::now();
  /* ROS_INFO("growing of segs took %f", (endtime - starttime).toSec() * 1000); */
  starttime = ros::WallTime::now();

  /* ROS_INFO("[SphereMap]: grown %u segments", num_segments_grown); */

  /* TRY MERGING */
  /* for (std::set<std::pair<uint, uint>>::iterator it = potential_merge_pairs_.begin(); it != potential_merge_pairs_.end(); it++) { */
  /*   std::map<uint, SphereMapSegment>::iterator seg_ptr1 = segments.find(it->first); */
  /*   if (seg_ptr1 == segments.end()) { */
  /*     continue; */
  /*   } */
  /*   std::map<uint, SphereMapSegment>::iterator seg_ptr2 = segments.find(it->second); */
  /*   if (seg_ptr2 == segments.end()) { */
  /*     continue; */
  /*   } */

  /* } */

  uint num_merge_tries     = 0;
  uint num_merge_successes = 0;

  for (uint i = 0; i < changed_segments.size(); i++) {
    /* IF SEGMENT DISAPPEARED, CONTINUE */
    std::map<uint, SphereMapSegment>::iterator seg_ptr1 = segments.find(changed_segments[i]);
    if (seg_ptr1 == segments.end()) {
      continue;
    }
    uint seg_id1 = seg_ptr1->first;

    /* CACHE CONNECTED SEGS CUZ THEY CAN CHANGE BY MERGING */
    std::map<uint, SphereMapSegmentConnection> connections_to_try = seg_ptr1->second.connections;
    /* for (uint j = 0; j < connections_to_try.size(); j++) { */
    for (std::map<uint, SphereMapSegmentConnection>::iterator conn_iterator = connections_to_try.begin(); conn_iterator != connections_to_try.end();
         conn_iterator++) {
      /* IF SEGMENT DISAPPEARED, CONTINUE */
      std::map<uint, SphereMapSegment>::iterator seg_ptr2 = segments.find(conn_iterator->first);
      if (seg_ptr2 == segments.end()) {
        continue;
      }
      /* MERGING WILL HANDLE IF THE CONNECTION NO LONGER EXISTS */
      uint merge_res = 0;
      if (seg_ptr1->first < seg_ptr2->first) {
        merge_res = tryMergingSegments(seg_ptr1, seg_ptr2, occupancy_octree_);
      } else {
        merge_res = tryMergingSegments(seg_ptr2, seg_ptr1, occupancy_octree_);
      }
      num_merge_tries++;

      /* IF THIS SEGMENT WAS MERGED INTO ANOTHER ONE (AND NOT OTHER WAY AROUND), THEN BREAK */
      if (merge_res > 0) {
        num_merge_successes++;
        if (merge_res != seg_id1) {
          bool master_segment_staged = false;
          for (uint j = i + 1; j < changed_segments.size(); j++) {
            if (changed_segments[j] == merge_res) {
              master_segment_staged = true;
              break;
            }
          }
          if (!master_segment_staged) {
            /* ROS_INFO("adding merged master segment to segment ot try merge"); */
            changed_segments.push_back(merge_res);
          }
          break;
        }
      }
      /* AT LEAST UPDATE CONNECTIONS */
      else {
        updateConnectionsForSegment(seg_ptr1);
      }
    }
  }

  endtime = ros::WallTime::now();
  /* ROS_INFO("merging of segs took %f", (endtime - starttime).toSec() * 1000); */
  starttime = ros::WallTime::now();

  uint num_portal_recomputes = 0;
  /* COMPUTE INTERPORTAL PATHS */
  for (uint seg_id : seg_ids_for_interportal_recomputation) {
    auto seg_ptr = segments.find(seg_id);
    if (seg_ptr == segments.end()) {
      continue;
    }
    seg_ptr->second.interportal_paths = {};

    uint num_computations = 0;
    for (std::map<uint, SphereMapSegmentConnection>::iterator conn_it1 = seg_ptr->second.connections.begin(); conn_it1 != seg_ptr->second.connections.end();
         conn_it1++) {
      std::map<uint, SphereMapSegmentConnection>::iterator conn_it2 = conn_it1;
      conn_it2++;
      for (; conn_it2 != seg_ptr->second.connections.end(); conn_it2++) {
        /* COMPUTE PATH */
        /* ros::Time     start_time       = ros::Time::now(); */
        /* ROS_INFO("conn id: %u, conn id2: %u", conn_it1->first, conn_it2->first); */
        uint               min_id, max_id;
        octomap::OcTreeKey path_start_key, path_end_key;
        if (conn_it1->first < conn_it2->first) {
          min_id         = conn_it1->first;
          max_id         = conn_it2->first;
          path_start_key = conn_it1->second.own_key;
          path_end_key   = conn_it2->second.own_key;
        } else {
          min_id         = conn_it2->first;
          max_id         = conn_it1->first;
          path_start_key = conn_it2->second.own_key;
          path_end_key   = conn_it1->second.own_key;
        }
        /* uint min_id = fmin(conn_it1->first, conn_it2->first); */
        /* uint max_id = fmax(conn_it1->first, conn_it2->first); */

        SphereMapPath interportal_path = computeDetailedPathInSegment(path_start_key, path_end_key, seg_ptr, 0);
        interportal_path.computeMinSafedist();
        /* if(interportal_path */
        /* ros::Time     end_time         = ros::Time::now(); */
        /* ROS_INFO("detailed astar took %f ms", (end_time - start_time).toSec() * 1000); */
        seg_ptr->second.interportal_paths.insert(std::make_pair(std::make_pair(min_id, max_id), interportal_path));
        /* ROS_INFO("interportal path nodes len: %lu. Reaches goal: %u", interportal_path.positions.size(), interportal_path.reaches_goal); */
        num_computations++;
        num_portal_recomputes++;
      }
    }
    /* ROS_INFO("num portals: %lu, num computations: %u", seg_ptr->second.connections.size(), num_computations); */
  }
  /* ROS_INFO("seg ids for interportal size: %lu", seg_ids_for_interportal_recomputation.size()); */
  seg_ids_for_interportal_recomputation.clear();

  endtime = ros::WallTime::now();
  /* ROS_INFO("interportals of segs took %f", (endtime - starttime).toSec() * 1000); */

  /* ROS_INFO("[SphereMap]: num merge successes: %u/%u , num interportal paths recomputes: %u", num_merge_successes, num_merge_tries, */
  /* num_portal_recomputes); */

  /* uint num_random_seg_merge_tries = 0; */
  /* for(uint i = 0; i < */
}
//}

/* std::vector<uint> SphereMap::getSegmentsIntersectingBBX() //{ */
std::vector<uint> SphereMap::getSegmentsIntersectingBBX(octomap::point3d pos, float box_halfsize) {
  /* CHECK FOR SEARCH BBX INTERSECTION WITH ANY OF THE SEGMENTS*/
  std::vector<uint> res;
  BoundingBox       bbx(box_halfsize, pos);

  for (std::map<uint, SphereMapSegment>::iterator it = segments.begin(); it != segments.end(); it++) {
    float d = it->second.bounding_sphere_radius;
    if (it->second.center.x() > bbx.x1 - d && it->second.center.x() < bbx.x2 + d && it->second.center.y() > bbx.y1 - d && it->second.center.y() < bbx.y2 + d &&
        it->second.center.z() > bbx.z1 - d && it->second.center.z() < bbx.z2 + d) {
      res.push_back(it->first);
    }
  }
  return res;
}
//}

/* uint SphereMap::spreadSegment() //{ */
uint SphereMap::spreadSegment(uint seg_id) {
  /* ROS_INFO("spreading segment %u", seg_id); */
  bool                                       is_staging_area = staging_area_settings_ptr_->enabled && seg_id == 1;
  std::map<uint, SphereMapSegment>::iterator seg_ptr         = segments.find(seg_id);
  if (seg_ptr == segments.end()) {
    ROS_WARN("[SphereMap]: segment to expand not found in segments!");
    return 0;
  }

  bool debug_segment_error = false;


  /* FOR BLOCK COMPUTATION */
  std::vector<octomap::point3d> nodes_for_bbx_computation_positions;
  std::vector<float>            nodes_for_bbx_computation_radii;


  /* CHECK WHICH PORTALS SHOULD BE DELETED */
  std::map<uint, SphereMapSegmentConnection> safest_connections;

  /* float max_bounding_radius = fmin(topology_mapping_settings_.segment_max_size_, seg_ptr->second.bounding_sphere_radius); */
  float max_bounding_radius = topology_mapping_settings_.segment_max_size_;
  float max_dist2           = max_bounding_radius * max_bounding_radius;

  /* BEGIN EXPANSION */
  octomap::point3d                     new_center      = seg_ptr->second.center;
  std::vector<octomap::OcTreeKey>      new_keys        = {};
  std::vector<octomap::SphereMapNode*> expanded        = {};
  std::vector<octomap::SphereMapNode*> frontier        = {};  // FIXME
  std::vector<octomap::SphereMapNode*> new_frontier    = {};
  uint                                 num_nodes_added = 0;

  /* GET STILL ALIVE NODES TO FRONTIER*/
  /* GET MIN AND MAX OF SEGMENT SAFEDIST */
  if (seg_ptr->second.keys.empty()) {
    ROS_ERROR("segment spread error2 of seg %u. Segment used to have bs of radius %f", seg_ptr->first, seg_ptr->second.bounding_sphere_radius);
    debug_segment_error = true;
    return 0;
  }

  float min_safedist;
  float max_safedist;
  bool  found_some_node = false;
  for (uint i = 0; i < seg_ptr->second.keys.size(); i++) {
    octomap::SphereMapNode* node_ptr = nodes->search(seg_ptr->second.keys[i], 16);
    if (node_ptr != NULL) {
      float rad = node_ptr->valuePtr()->radius;
      if (!found_some_node) {
        min_safedist    = rad;
        max_safedist    = rad;
        found_some_node = true;
      } else {
        if (rad < min_safedist) {
          min_safedist = rad;
        } else if (rad > max_safedist) {
          max_safedist = rad;
        }
      }
      new_keys.push_back(seg_ptr->second.keys[i]);
      frontier.push_back(node_ptr);
      nodes_for_bbx_computation_positions.push_back(node_ptr->valuePtr()->pos);
      nodes_for_bbx_computation_radii.push_back(node_ptr->valuePtr()->radius);
    } else {
      ROS_ERROR("segment spread error of seg: %u", seg_ptr->first);
      debug_segment_error = true;
    }
  }

  if (debug_segment_error) {
    ROS_ERROR("had bs: %f", seg_ptr->second.bounding_sphere_radius);
    ROS_ERROR("had keys: %lu", seg_ptr->second.keys.size());
  }

  while (!frontier.empty()) {
    /* EXPANSION ITERATION BEGIN */
    for (uint j = 0; j < frontier.size(); j++) {
      /* CHECK CONNECTED */
      for (uint k = 0; k < frontier[j]->valuePtr()->connected_keys.size(); k++) {
        octomap::OcTreeKey      adjacent_node_key    = frontier[j]->valuePtr()->connected_keys[k];
        octomap::SphereMapNode* adjacent_node_ptr    = nodes->search(adjacent_node_key, 16);
        uint                    adjacent_node_seg_id = adjacent_node_ptr->valuePtr()->segment_id;
        if (adjacent_node_seg_id > 0) {
          /* FOUND ADJACENT NODE FROM OTHER SEGMENT, CHECK SAFETY AND ADD TO SAFE TRANSITIONS */
          if (adjacent_node_seg_id != seg_id) {
            float connection_safety = fmin(adjacent_node_ptr->valuePtr()->radius, frontier[j]->valuePtr()->radius);
            std::map<uint, SphereMapSegmentConnection>::iterator best_adj_seg_connection_ptr = safest_connections.find(adjacent_node_seg_id);
            if (best_adj_seg_connection_ptr == safest_connections.end()) {
              SphereMapSegmentConnection conn(nodes->coordToKey(frontier[j]->valuePtr()->pos, 16), adjacent_node_key, connection_safety);
              safest_connections.insert(std::make_pair(adjacent_node_seg_id, conn));
            } else {
              float best_safety_so_far = best_adj_seg_connection_ptr->second.safety;
              if (best_safety_so_far < connection_safety) {
                SphereMapSegmentConnection conn(nodes->coordToKey(frontier[j]->valuePtr()->pos, 16), adjacent_node_key, connection_safety);
                best_adj_seg_connection_ptr->second = conn;
              }
            }
          }
          continue;
        }

        /* IF STAGING AREA, ONLY CHECK IF NOT FLOWING THROUGH WALL */
        if (is_staging_area) {
          octomap::point3d delta_vec2 = adjacent_node_ptr->valuePtr()->pos - staging_area_settings_ptr_->wall_pos;
          float            wall_proj  = delta_vec2.dot(staging_area_settings_ptr_->wall_dir_outward);
          if (wall_proj >= 0) {
            continue;
          }
        } else {
          /* CHECK IF NOT FAR FROM CENTER */
          octomap::point3d deltavec = adjacent_node_ptr->valuePtr()->pos - new_center;
          if (deltavec.dot(deltavec) > max_dist2) {
            continue;
          }
        }

        /* ADD NODE */
        adjacent_node_ptr->valuePtr()->segment_id = seg_id;
        new_frontier.push_back(adjacent_node_ptr);
        new_keys.push_back(adjacent_node_key);
        num_nodes_added++;
      }
      expanded.push_back(frontier[j]);
    }
    /* COMPUTE NEW CENTER */
    octomap::point3d sum_pos(0, 0, 0);
    for (uint j = 0; j < expanded.size(); j++) {
      sum_pos += expanded[j]->valuePtr()->pos;
    }
    new_center = sum_pos * (1 / ((float)expanded.size()));
    /* SET NEW FRONTIER */
    frontier     = new_frontier;
    new_frontier = {};
  }

  float bounding_radius2 = 0;
  for (uint j = 0; j < expanded.size(); j++) {
    float center_dist2 = (new_center - expanded[j]->valuePtr()->pos).norm_sq();
    if (center_dist2 > bounding_radius2) {
      bounding_radius2 = center_dist2;
    }
  }
  seg_ptr->second.bounding_sphere_radius = sqrt(bounding_radius2);
  seg_ptr->second.center                 = new_center;
  seg_ptr->second.keys                   = new_keys;

  if (debug_segment_error) {
    ROS_ERROR("now has bs: %f", seg_ptr->second.bounding_sphere_radius);
    ROS_ERROR("now has keys: %lu", seg_ptr->second.keys.size());
  }
  /* TODO add bbx computation */

  /* ADD OR UPDATE CONNECTIONS */
  updateConnectionsForSegmentWithFoundConnections(seg_ptr, safest_connections);
  if (num_nodes_added > 0) {
    for (uint i = 0; i < nodes_for_bbx_computation_positions.size(); i++) {
      nodes_for_bbx_computation_positions[i] = nodes_for_bbx_computation_positions[i] - new_center;
    }
    calculateBlockParamsForSegment(seg_ptr, nodes_for_bbx_computation_positions, nodes_for_bbx_computation_radii);

    /* IF SEGMENT HAS CHANGED, TRY MERGING IT WITH OTHERS CONNECTED */
    for (std::map<uint, SphereMapSegmentConnection>::iterator conn_iter = seg_ptr->second.connections.begin(); conn_iter != seg_ptr->second.connections.end();
         conn_iter++) {
      if (conn_iter->first < seg_ptr->first) {
        potential_merge_pairs_.insert(std::make_pair(conn_iter->first, seg_ptr->first));
      } else {
        potential_merge_pairs_.insert(std::make_pair(seg_ptr->first, conn_iter->first));
      }
    }
  }

  return num_nodes_added;

  /* TODO try merging to staging area segment immediately */

  /* ROS_INFO("segment %u now has nodes: %lu , radius: %f m, connected segments: %lu.Connection safedists: ", seg_id, expanded.size(), */
  /*          seg_ptr->second.bounding_sphere_radius, seg_ptr->second.connections.size()); */
  /* for (std::map<uint, SphereMapSegmentConnection>::iterator it2 = safest_connections.begin(); it2 != safest_connections.end(); it2++) { */
  /*   ROS_INFO("id:%u, safety: %f m", it2->first, it2->second.safety); */
  /* } */
}
//}

/* uint SphereMap::updateConnectionsForSegmentWithFoundConnections() //{ */
uint SphereMap::updateConnectionsForSegmentWithFoundConnections(std::map<uint, SphereMapSegment>::iterator seg_ptr,
                                                                std::map<uint, SphereMapSegmentConnection> safest_connections) {
  /* DELETE CURRENT CONNECTIONS */
  for (std::map<uint, SphereMapSegmentConnection>::iterator conn_it = seg_ptr->second.connections.begin(); conn_it != seg_ptr->second.connections.end();
       conn_it++) {
    /* REMOVE FROM PORTALS */
    uint min_id                 = fmin(seg_ptr->first, conn_it->first);
    uint max_id                 = fmax(seg_ptr->first, conn_it->first);
    bool found_portal_to_delete = false;
    for (std::vector<std::pair<uint, uint>>::iterator portals_it = portals.begin(); portals_it != portals.end(); portals_it++) {
      if (portals_it->first == min_id && portals_it->second == max_id) {
        found_portal_to_delete = true;
        portals.erase(portals_it);
        break;
      }
    }
    if (!found_portal_to_delete) {
      ROS_ERROR("portal removing error");
    }

    /* DELETE ON OTHER SIDE */
    uint                                       other_seg_id = conn_it->first;
    std::map<uint, SphereMapSegment>::iterator seg_ptr2     = segments.find(other_seg_id);
    if (seg_ptr2 == segments.end()) {
      /* SEGMENT NO LONGER EXISTS (might have been merged or deleted) */
      continue;
    }
    std::map<uint, SphereMapSegmentConnection>::iterator conn_it2 = seg_ptr2->second.connections.find(seg_ptr->first);
    ;
    if (conn_it2 == seg_ptr2->second.connections.end()) {
      continue;
    }
    seg_ptr2->second.connections.erase(conn_it2);
  }
  seg_ptr->second.connections.clear();


  /* FIRST CHECK IF ANY OF THE PORTALS WERE DESTROYED */
  /* { */
  /*   bool found_destroyed_portal = true; */
  /*   while (found_destroyed_portal) { */
  /*     found_destroyed_portal = false; */
  /*     for (std::map<uint, SphereMapSegmentConnection>::iterator conn_iter = seg_ptr->second.connections.begin(); conn_iter !=
   * seg_ptr->second.connections.end(); */
  /*          conn_iter++) { */
  /*       if (nodes->search(conn_iter->second.own_key, 16) == NULL || nodes->search(conn_iter->second.other_key, 16) == NULL) { */
  /*         /1* SOME KEY WAS DESTROYED, REMOVE PORTAL FROM HERE AND FROM OTHER SEGMENT *1/ */
  /*         std::map<uint, SphereMapSegment>::iterator seg_ptr2 = segments.find(conn_iter->first); */
  /*         /1* REMOVE FROM OTHER SEGMENT *1/ */
  /*         seg_ptr2->second.connections.erase(seg_ptr2->second.connections.find(seg_ptr->first)); */
  /*         /1* REMOVE FROM THIS SEGMENT *1/ */
  /*         seg_ptr->second.connections.erase(conn_iter); */
  /*         found_destroyed_portal = true; */

  /*         /1* ALSO REMOVE FROM PORTALS  LIST *1/ */
  /*         uint min_id                 = fmin(seg_ptr->first, seg_ptr2->first); */
  /*         uint max_id                 = fmax(seg_ptr->first, seg_ptr2->first); */
  /*         bool found_portal_to_delete = false; */
  /*         for (std::vector<std::pair<uint, uint>>::iterator portals_it = portals.begin(); portals_it != portals.end(); portals_it++) { */
  /*           if (portals_it->first == min_id && portals_it->second == max_id) { */
  /*             found_portal_to_delete = true; */
  /*             portals.erase(portals_it); */
  /*             break; */
  /*           } */
  /*         } */
  /*         if (!found_portal_to_delete) { */
  /*           ROS_ERROR("portal removing error"); */
  /*         } */
  /*         break; */
  /*       } */
  /*     } */
  /*   } */
  /* } */

  if (safest_connections.empty()) {
    return 0;
  }
  uint seg_id = seg_ptr->first;
  for (std::map<uint, SphereMapSegmentConnection>::iterator conn_iter = safest_connections.begin(); conn_iter != safest_connections.end(); conn_iter++) {
    uint                       other_seg_id              = conn_iter->first;
    SphereMapSegmentConnection conn_to_add               = conn_iter->second;
    auto                       own_connections_query_res = seg_ptr->second.connections.find(other_seg_id);

    if (own_connections_query_res == seg_ptr->second.connections.end()) {
      /* PORTAL IS NEW, ADD IT HERE AND TO OTHER SEGMENT */
      std::map<uint, SphereMapSegment>::iterator seg_ptr2 = segments.find(other_seg_id);
      /* ROS_INFO("adding new connection from id: %u to seg_id: %u", seg_ptr->first, seg_ptr2->first); */
      seg_ptr->second.connections.insert(std::make_pair(other_seg_id, conn_to_add));
      seg_ptr2->second.connections.insert(std::make_pair(seg_id, conn_to_add.getSwappedCopy()));

      /* ADD ALSO TO PORTALS */
      if (seg_ptr->first > seg_ptr2->first) {
        portals.push_back(std::make_pair(seg_ptr2->first, seg_ptr->first));
      } else {
        portals.push_back(std::make_pair(seg_ptr->first, seg_ptr2->first));
      }
    } else {
      /* PORTAL IS NOT NEW, UPDATE IT FOR BOTH SEGMENTS, if it is safer than old*/
      if (own_connections_query_res->second.shouldUpdateWith(conn_to_add)) {
        /* CHANGE IN ADJACENT SEG CONNECTIONS */
        std::map<uint, SphereMapSegment>::iterator seg_ptr2 = segments.find(conn_iter->first);
        /* ROS_INFO("updating connection from id: %u to seg_id: %u", seg_ptr->first, seg_ptr2->first); */
        if (seg_ptr2 == segments.end()) {
          ROS_ERROR("[SphereMap]: adjacent segment not found!");
          continue;
        }
        std::map<uint, SphereMapSegmentConnection>::iterator adj_seg_connection_query = seg_ptr2->second.connections.find(seg_id);
        if (adj_seg_connection_query == seg_ptr2->second.connections.end()) {
          ROS_ERROR("[SphereMap]: connection that should exist not found in adjacent segment conections");
          continue;
        }
        adj_seg_connection_query->second = conn_to_add.getSwappedCopy();

        /* CHANGE IN OWN CONNECTIONS */
        own_connections_query_res->second = conn_to_add;
      }
    }
  }

  return 1;
}
//}

/* uint SphereMap::growSegment() //{ */
uint SphereMap::growSegment(octomap::OcTreeKey start_key) {
  octomap::SphereMapNode* start_node = nodes->search(start_key, 16);
  if (start_node == NULL) {
    ROS_WARN("[SphereMap]: key to start growth has no node at that key");
    return 0;
  }

  uint new_seg_id = next_seg_id;
  next_seg_id++;
  start_node->valuePtr()->segment_id = new_seg_id;

  SphereMapSegment new_seg;
  new_seg.center                 = start_node->valuePtr()->pos;
  new_seg.keys                   = {start_key};
  new_seg.bounding_sphere_radius = start_node->valuePtr()->radius;
  segments.insert(std::make_pair(new_seg_id, new_seg));
  spreadSegment(new_seg_id);
  return new_seg_id;
}
//}

/* void SphereMap::rebuildSegments() //{ */
void SphereMap::rebuildSegments(octomap::point3d current_position_, float box_halfsize) {
  std::vector<octomap::point3d> destroyed_segments_centers;
  std::vector<float>            destroyed_segments_radii;

  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys = getMaxSearchBBXBorderKeys(current_position_, box_halfsize);
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
    if (it->valuePtr()->segment_id > 0) {
      /* NODE IS IN SEGMENT, DELETE SEGMENT AND CONTINUE */
      std::map<uint, SphereMapSegment>::iterator seg_ptr = segments.find(it->valuePtr()->segment_id);
      if (seg_ptr == segments.end()) {
        ROS_WARN("[SphereMap]: segment %d not in segments! this should not happen", it->valuePtr()->segment_id);
        it->valuePtr()->segment_id = 0;
        continue;
      }
      /* GET POINT AND RADIUS OF SEGMENT TO HAVE BBX FOR LATER CREATION OF NEW SEGMENTS */
      destroyed_segments_centers.push_back(seg_ptr->second.center);
      destroyed_segments_radii.push_back(seg_ptr->second.bounding_sphere_radius);

      /* removeSegment(seg_ptr); */
    }
    /* octomap::SphereMapNode * node = it-> */
    /* float new_odist = getObstacleDist(it->valuePtr()->pos, pcl_map_ptr); */
    /* if (it->second.radius > new_odist) { */
    /* ROS_INFO("[SphereMap]: lowered radius from %f to %f", it->second.radius, new_odist); */
    /* } else if (it->second.radius < new_odist) { */
    /*   ROS_INFO("[SphereMap]: increased radius from %f to %f", it->second.radius, new_odist); */
    /* } */
    /* it->valuePtr()->radius = fmin(new_odist, enforced_max_sphere_radius); */
  }

  /* TODO create new segments more smart */
  bbx_keys = getMaxSearchBBXBorderKeys(current_position_, 50);
  std::vector<std::pair<float, octomap::SphereMapOcTree::leaf_bbx_iterator>> nodes_to_segment;
  getUnsegmentedNodesWithObstacleDists(current_position_, box_halfsize * 2, nodes_to_segment);
  /* ROS_INFO("nodes to segment size %u", nodes_to_segment.size()); */
  std::sort(nodes_to_segment.begin(), nodes_to_segment.end(), segmapnode_pair_compare());
  for (uint i = 0; i < nodes_to_segment.size(); i++) {
    if (nodes_to_segment[i].second->valuePtr()->segment_id > 0) {
      /* ROS_INFO("node to segment segmented"); */
      continue;
    }
    uint new_seg_id = next_seg_id;
    next_seg_id++;
    /* ROS_INFO("creating segment with seg_id %u", new_seg_id); */
    nodes_to_segment[i].second->valuePtr()->segment_id = new_seg_id;

    SphereMapSegment new_seg;
    new_seg.center                 = nodes_to_segment[i].second->valuePtr()->pos;
    new_seg.keys                   = {nodes_to_segment[i].second.getKey()};
    new_seg.bounding_sphere_radius = nodes_to_segment[i].second->valuePtr()->radius;
    float max_bounding_radius      = 10;
    float max_dist2                = max_bounding_radius * max_bounding_radius;

    /* BEGIN EXPANSION */
    std::vector<octomap::SphereMapNode*> expanded     = {};
    std::vector<octomap::SphereMapNode*> frontier     = {nodes->search(nodes_to_segment[i].second.getKey(), 16)};  // FIXME
    std::vector<octomap::SphereMapNode*> new_frontier = {};
    while (!frontier.empty()) {
      /* EXPANSION ITERATION BEGIN */
      /* ROS_INFO("expansion iter"); */
      for (uint j = 0; j < frontier.size(); j++) {
        /* CHECK CONNECTED */
        for (uint k = 0; k < frontier[j]->valuePtr()->connected_keys.size(); k++) {
          octomap::SphereMapNode* adjacent_node_ptr = nodes->search(frontier[j]->valuePtr()->connected_keys[k], 16);
          if (adjacent_node_ptr->valuePtr()->segment_id > 0) {
            /* ROS_INFO("connected node already segmented"); */
            continue;
          }

          /* CHECK IF NOT FAR FROM CENTER */
          octomap::point3d deltavec = adjacent_node_ptr->valuePtr()->pos - new_seg.center;
          if (deltavec.dot(deltavec) > max_dist2) {
            /* ROS_INFO("connected node too far"); */
            continue;
          }

          /* ADD NODE */
          adjacent_node_ptr->valuePtr()->segment_id = new_seg_id;
          new_frontier.push_back(adjacent_node_ptr);
          new_seg.keys.push_back(frontier[j]->valuePtr()->connected_keys[k]);
        }
        expanded.push_back(frontier[j]);
      }
      /* COMPUTE NEW CENTER */
      octomap::point3d sum_pos(0, 0, 0);
      for (uint j = 0; j < expanded.size(); j++) {
        sum_pos += expanded[j]->valuePtr()->pos;
      }
      new_seg.center = sum_pos * (1 / ((float)expanded.size()));
      /* SET NEW FRONTIER */
      frontier     = new_frontier;
      new_frontier = {};
    }
    /* ROS_INFO("created segment with %lu nodes", expanded.size()); */
    segments.insert(std::make_pair(new_seg_id, new_seg));
  }

  /* for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) { */
  /*   if (it->valuePtr()->segment_id > 0) { */
  /* } */
}
//}

/* void SphereMap::removeSegment() //{ */
/* void SphereMap::removeSegment(std::map<uint, SphereMapSegment>::iterator it) { */
/*   for (uint i = 0; i < it->second.keys.size(); i++) { */
/*     octomap::SphereMapNode* node_ptr = nodes->search(it->second.keys[i], 16); */
/*     if (node_ptr != NULL) { */
/*       node_ptr->valuePtr()->segment_id = 0; */
/*     } */
/*   } */
/*   segments.erase(it); */
/* } */
//}

/* uint SphereMap::updateConnectionsForSegment() //{ */
uint SphereMap::updateConnectionsForSegment(std::map<uint, SphereMapSegment>::iterator seg_ptr) {
  /* ROS_INFO("updating connections for seg %u", seg_ptr->first); */
  uint seg_id = seg_ptr->first;
  /* CHECK ALL NODES AND THEIR ADJACENTS */
  std::map<uint, SphereMapSegmentConnection> current_connections;
  for (octomap::OcTreeKey node_key : seg_ptr->second.keys) {
    octomap::SphereMapNode* node_ptr = nodes->search(node_key, 16);
    for (octomap::OcTreeKey adjacent_node_key : node_ptr->valuePtr()->connected_keys) {
      octomap::SphereMapNode* adjacent_node_ptr    = nodes->search(adjacent_node_key, 16);
      uint                    adjacent_node_seg_id = adjacent_node_ptr->valuePtr()->segment_id;

      if (adjacent_node_seg_id > 0 && adjacent_node_seg_id != seg_id) {
        /* CHECK IF ALREADY HAVE A CONNECTION FOR THIS SEGMENT */
        float                                                connection_safety = fmin(adjacent_node_ptr->valuePtr()->radius, node_ptr->valuePtr()->radius);
        std::map<uint, SphereMapSegmentConnection>::iterator best_adj_seg_connection_ptr = current_connections.find(adjacent_node_seg_id);
        /* IF DONT HAVE THIS CONNECTION, ADD IT */
        if (best_adj_seg_connection_ptr == current_connections.end()) {
          SphereMapSegmentConnection conn(node_key, adjacent_node_key, connection_safety);
          current_connections.insert(std::make_pair(adjacent_node_seg_id, conn));
          /* ELSE ONLY ADD IT IF IT IS SAFER */
        } else {
          float best_safety_so_far = best_adj_seg_connection_ptr->second.safety;
          if (best_safety_so_far < connection_safety) {
            SphereMapSegmentConnection conn(node_key, adjacent_node_key, connection_safety);
            best_adj_seg_connection_ptr->second = conn;
          }
        }
      }
    }
    /* std::vector<octomap::OcTreeKey */
    /* points1.push_back(point); */
    /* new_center += point; */
  }

  /* ROS_INFO("found %lu potential connections", current_connections.size()); */
  /* UPDATE PROTALS WITH THESE NEW PORTALS */
  updateConnectionsForSegmentWithFoundConnections(seg_ptr, current_connections);
  seg_ids_for_interportal_recomputation.insert(seg_ptr->first);

  /* UPDATE INTER-PORTAL SAFE PATHS */
  return 1;
}
//}

/* bool SphereMap::tryMergingSegments() //{ */
uint SphereMap::tryMergingSegments(std::map<uint, SphereMapSegment>::iterator it1, std::map<uint, SphereMapSegment>::iterator it2,
                                   std::shared_ptr<octomap::OcTree> occupancy_octree_) {
  /* ROS_INFO("[SphereMap]: trying to merge semgents %u and %u", it1->first, it2->first); */
  /* ASSUME IT1 HAS LOWER ID */
  /* AND ALSO THAT BOTH ARE CONNECTED */

  /* CHECK DUMB STUFF */
  if (it1->first == it2->first) {
    ROS_WARN("trying to merge segs of same id");
    return 0;
  }

  /* GET NEW CENTER AND BOUNDING SPHERE */
  std::vector<octomap::point3d> points1;
  std::vector<octomap::point3d> points2;

  octomap::point3d new_center(0, 0, 0);
  for (uint i = 0; i < it1->second.keys.size(); i++) {
    octomap::point3d point = nodes->keyToCoord(it1->second.keys[i], 16);
    points1.push_back(point);
    new_center += point;
  }
  for (uint i = 0; i < it2->second.keys.size(); i++) {
    octomap::point3d point = nodes->keyToCoord(it2->second.keys[i], 16);
    points2.push_back(point);
    new_center += point;
  }
  new_center = new_center * (1 / (float)(points1.size() + points2.size()));

  /* GET BBX RADIUS */
  float bounding_sphere_radius2 = 0;
  for (uint i = 0; i < points1.size(); i++) {
    float dist2 = (points1[i] - new_center).norm_sq();
    if (dist2 > bounding_sphere_radius2) {
      bounding_sphere_radius2 = dist2;
    }
  }
  float new_bounding_sphere_radius = sqrt(bounding_sphere_radius2);

  /* CHECK IF IS TRYING TO MERGE WITH STAGING AREA */
  if (it1->first == 1 && staging_area_settings_ptr_->enabled) {
    bool all_points_in_staging_area = true;
    /* for (octomap::OcTreeKey s2key : it2->second.keys) { */
    /*   octomap::point3d deltavec  = nodes->keyToCoord(s2key, 16) - staging_area_settings_ptr_->wall_pos; */
    /*   float            wall_proj = deltavec.dot(staging_area_settings_ptr_->wall_dir_outward); */
    /*   if (wall_proj > 0) { */
    /*     all_points_in_staging_area = false; */
    /*   } */
    /* } */
    octomap::point3d deltavec  = it2->second.center - staging_area_settings_ptr_->wall_pos;
    float            wall_proj = deltavec.dot(staging_area_settings_ptr_->wall_dir_outward);
    if (wall_proj < 0) {
      /* ROS_INFO("segment is connected to staging area and has all points behind wall, merging it."); */
      return mergeSegments(it1, it2, new_center, new_bounding_sphere_radius, true);
    } else {
      return 0;
    }
  }
  /* TEST VISIBILITY */
  /* if (!spheremap_server::arePointsMutuallyVisible2(it1->second.center, it2->second.center, occupancy_octree_)) { */
  /*   return 0; */
  /* } */

  /* TEST MAX SIZE */
  if (new_bounding_sphere_radius > topology_mapping_settings_.merged_segment_max_size_) {
    float dist = (it1->second.center - it2->second.center).norm();
    /* bool engulfed_1 = (it1->second.center - new_center).norm() + it1->second.bounding_sphere_radius < new_bounding_sphere_radius; */
    /* bool engulfed_2 = (it2->second.center - new_center).norm() + it2->second.bounding_sphere_radius < new_bounding_sphere_radius; */
    /* if (!engulfed_1 && !engulfed_2) { */
    /*   return 0; */
    /* } */
    if (!(dist + it1->second.bounding_sphere_radius < topology_mapping_settings_.merged_segment_max_size_ ||
          dist + it2->second.bounding_sphere_radius < topology_mapping_settings_.merged_segment_max_size_)) {
      return 0;
    }
    /* ROS_INFO("merging: rad1: %f rad2: %f dist: %f", it1->second.bounding_sphere_radius, it2->second.bounding_sphere_radius, dist); */

    /* if (new_bounding_sphere_radius > topology_mapping_settings_.merged_segment_max_size_ * 2) { */
    /*   return 0; */
    /* } */
  }

  if (new_bounding_sphere_radius > largest_bounging_sphere_radius_) {
    largest_bounging_sphere_radius_ = new_bounding_sphere_radius;
    /* ROS_INFO("[SphereMap]: new max bounding sphere radius is %f", largest_bounging_sphere_radius_); */
  }
  /* TEST COMPACTNESS */

  /* TEST CONVEXITY */

  /* MERGE */
  return mergeSegments(it1, it2, new_center, new_bounding_sphere_radius);
}
//}

/* uint SphereMap::mergeSegments() //{ */
uint SphereMap::mergeSegments(std::map<uint, SphereMapSegment>::iterator it1, std::map<uint, SphereMapSegment>::iterator it2, octomap::point3d new_center,
                              float new_bounding_sphere_radius, bool force_seg1_to_be_master) {

  if (!force_seg1_to_be_master) {
    if (it1->second.keys.size() < it2->second.keys.size()) {
      /* SWAP TO HAVE IT1 LARGER */
      std::map<uint, SphereMapSegment>::iterator tmp_it = it1;
      it1                                               = it2;
      it2                                               = tmp_it;
    }
  }

  /* TEST IF CONNECTED */
  std::map<uint, SphereMapSegmentConnection>::iterator mutual_connection_iter = it2->second.connections.find(it1->first);
  if (mutual_connection_iter == it2->second.connections.end()) {
    ROS_ERROR("[SphereMap]: trying to merge unconnected segments!");
    return 0;
  }

  std::vector<octomap::point3d> nodes_for_bbx_computation_positions;
  std::vector<float>            nodes_for_bbx_computation_radii;
  for (uint i = 0; i < it1->second.keys.size(); i++) {
    octomap::SphereMapNode* node = nodes->search(it1->second.keys[i], 16);
    nodes_for_bbx_computation_positions.push_back(node->valuePtr()->pos - new_center);
    nodes_for_bbx_computation_radii.push_back(node->valuePtr()->radius);
  }


  /* MERGE KEYS */
  it1->second.keys.insert(it1->second.keys.end(), it2->second.keys.begin(), it2->second.keys.end());
  it1->second.center                 = new_center;
  it1->second.bounding_sphere_radius = new_bounding_sphere_radius;

  /* SET NODES OF S2 TO HAVE CORRECT SEG_ID */
  for (uint i = 0; i < it2->second.keys.size(); i++) {
    octomap::SphereMapNode* node = nodes->search(it2->second.keys[i], 16);
    node->valuePtr()->segment_id = it1->first;
    nodes_for_bbx_computation_positions.push_back(node->valuePtr()->pos - new_center);
    nodes_for_bbx_computation_radii.push_back(node->valuePtr()->radius);
  }


  /* REMAKE CONNECTIONS */

  /* REMOVE CONNECTIONS OF S2 */
  for (std::map<uint, SphereMapSegmentConnection>::iterator conn_iterator = it2->second.connections.begin(); conn_iterator != it2->second.connections.end();
       conn_iterator++) {
    uint                                       other_seg_id = conn_iterator->first;
    std::map<uint, SphereMapSegment>::iterator it3          = segments.find(other_seg_id);
    if (it3 == segments.end()) {
      ROS_ERROR("removing connection to segment %u failed! segment not found in segments.", other_seg_id);
    }
    /* ERASE CONNECTION FROM SEGMENT CONNECTED TO S2 */
    std::map<uint, SphereMapSegmentConnection>::iterator conn_on_other_side = it3->second.connections.find(it2->first);
    it3->second.connections.erase(conn_on_other_side);


    std::pair<uint, uint> portal = it2->first < it3->first ? std::make_pair(it2->first, it3->first) : std::make_pair(it3->first, it2->first);
    std::vector<std::pair<uint, uint>>::iterator portal_it = std::find(portals.begin(), portals.end(), portal);
    if (portal_it != portals.end()) {
      portals.erase(portal_it);
    }
    /* WE DONT NEED TO ERASE IT FROM S2 CONNECTIONS BECAUSE ITS GONNA BE GONE SOON */
  }

  /* REMOVE PORTALS */
  /* for(std::vector<std::pair<uint,uint>>::iterator portal_it = portals.begin(); */

  /* UPDATE CONNECTIONS OF S1 */
  updateConnectionsForSegment(it1);
  calculateBlockParamsForSegment(it1, nodes_for_bbx_computation_positions, nodes_for_bbx_computation_radii);

  /* COMPUTE BBX */

  /* REMOVE SMALLER SEGMENT */
  segments.erase(it2);
  return it1->first;
}
//}

/* void SphereMap::getUnsegmentedNodesWithObstacleDists() //{ */
void SphereMap::getUnsegmentedNodesWithObstacleDists(octomap::point3d current_position_, float box_halfsize,
                                                     std::vector<std::pair<float, octomap::SphereMapOcTree::leaf_bbx_iterator>>& res_pairs) {
  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys = getMaxSearchBBXBorderKeys(current_position_, box_halfsize);
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
    if (it->valuePtr()->segment_id == 0) {
      res_pairs.push_back(std::make_pair(-it->valuePtr()->radius, it));
    }
  }
}
//}

/* NAVIGATION */

/* std::vector<octomap::OcTreeKey> SphereMap::getAdjacentNodesKeys() //{ */
std::vector<octomap::OcTreeKey> SphereMap::getAdjacentNodesKeys(octomap::point3d test_point) {
  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys = getMaxSearchBBXBorderKeys(test_point);
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
    /* deltavec = pos - it->valuePtr()->pos; */
    /* dist2    = deltavec.dot(deltavec); */

    /* if (dist2 < pow(node_ptr->valuePtr()->radius + it->valuePtr()->radius, 2)) { */
    /*   now_connected.push_back(it.getKey()); */
    /*   if (std::find(old_connected.begin(), old_connected.end(), it.getKey()) == old_connected.end()) { */
    /*     /1* FOUND NEW CONNECTION, ADD IT TO THE OTHER ONE AS WELL *1/ */
    /*     /1* old_connected->erase( *1/ */
    /*     newly_connected.push_back(it.getKey()); */
    /*     it->valuePtr()->connected_keys.push_back(key); */
    /*   } */
    /* } */
  }
  return std::vector<octomap::OcTreeKey>();  // FIXME
}
//}

/* std::optional<octomap::OcTreeKey> SphereMap::getNearestNodeKey() //{ */
std::optional<std::pair<float, octomap::OcTreeKey>> SphereMap::getNearestNodeKey(octomap::point3d test_point) {
  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys       = getMaxSearchBBXBorderKeys(test_point);
  bool                                              found_near_key = false;
  octomap::OcTreeKey                                nearest_key;
  float                                             nearest_dist;
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
    float dist = (test_point - it->valuePtr()->pos).norm() - it->valuePtr()->radius;
    if (!found_near_key || dist < nearest_dist) {
      nearest_key    = it.getKey();
      nearest_dist   = dist;
      found_near_key = true;
    }
  }
  if (found_near_key) {
    return std::make_pair(nearest_dist, nearest_key);
  }

  if (!found_near_key) {
    for (octomap::SphereMapOcTree::leaf_iterator it = nodes->begin_leafs(16); it != nodes->end_leafs(); it++) {
      float dist = (test_point - it->valuePtr()->pos).norm() - it->valuePtr()->radius;
      if (!found_near_key || dist < nearest_dist) {
        nearest_key    = it.getKey();
        nearest_dist   = dist;
        found_near_key = true;
      }
    }
    if (found_near_key) {
      return std::make_pair(nearest_dist, nearest_key);
    }
  }
  return std::nullopt;
}
//}

/* std::optional<octomap::OcTreeKey> SphereMap::getNearestNodeKey() //{ */
std::optional<std::pair<float, octomap::OcTreeKey>> SphereMap::getNearestNodeKey(octomap::point3d test_point, float max_point_dist) {
  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys       = getMaxSearchBBXBorderKeys(test_point, max_point_dist + enforced_max_sphere_radius);
  bool                                              found_near_key = false;
  octomap::OcTreeKey                                nearest_key;
  float                                             nearest_dist;
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
    float dist = (test_point - it->valuePtr()->pos).norm() - it->valuePtr()->radius;
    if (!found_near_key || dist < nearest_dist) {
      nearest_key    = it.getKey();
      nearest_dist   = dist;
      found_near_key = true;
    }
  }
  if (found_near_key) {
    return std::make_pair(nearest_dist, nearest_key);
  }
  return std::nullopt;
}
//}

/* std::vector<std::pair<float, octomap::OcTreeKey>>   SphereMap::getNearestNodeKeysForEachSegment() //{ */
std::vector<std::pair<float, octomap::OcTreeKey>> SphereMap::getNearestNodeKeysForEachSegment(octomap::point3d test_point, float max_point_dist,
                                                                                              bool only_add_connectable, float goal_odist) {
  std::map<uint, std::pair<float, octomap::OcTreeKey>> node_map;

  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys = getMaxSearchBBXBorderKeys(test_point, max_point_dist + enforced_max_sphere_radius);
  octomap::OcTreeKey                                nearest_key;
  float                                             nearest_dist;
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
    float dist2 = (test_point - it->valuePtr()->pos).norm_sq();
    if (only_add_connectable) {
      /* CHECK IF CONNECTABLE */
      if (!areConnectable(dist2, it->valuePtr()->radius, goal_odist)) {
        continue;
      }
    }
    float dist = sqrt(dist2) - it->valuePtr()->radius;
    if (dist > max_point_dist) {
      continue;
    }

    uint seg_id       = it->valuePtr()->segment_id;
    auto map_iterator = node_map.find(seg_id);
    if (map_iterator == node_map.end()) {
      /* ADD NODE FOR THIS SEG */
      node_map.insert(std::make_pair(seg_id, std::make_pair(dist, it.getKey())));
    } else {
      /* CHECK IF BETTER */
      float comp_dist = map_iterator->second.first;
      if (dist < comp_dist) {
        map_iterator->second.first  = dist;
        map_iterator->second.second = it.getKey();
      }
    }
  }

  std::vector<std::pair<float, uint>> sort_pairs;
  for (auto it = node_map.begin(); it != node_map.end(); it++) {
    sort_pairs.push_back(std::make_pair(it->second.first, it->first));
  }
  std::sort(sort_pairs.begin(), sort_pairs.end());

  std::vector<std::pair<float, octomap::OcTreeKey>> res;
  for (auto it = sort_pairs.begin(); it != sort_pairs.end(); it++) {
    auto nodemap_ptr = node_map.find(it->second);
    res.push_back(std::make_pair(nodemap_ptr->second.first, nodemap_ptr->second.second));
  }
  return res;
}
//}

/* std::vector<octomap::OcTreeKey> SphereMap::getConnectableNodesKeys() //{ */
std::vector<octomap::OcTreeKey> SphereMap::getConnectableNodesKeys(octomap::point3d test_point, float odist) {
  return std::vector<octomap::OcTreeKey>();  // FIXME
}
//}

/* std::optional<std::pair<float, octomap::OcTreeKey>> SphereMap::getNearestConnectableNodeKey() //{ */
std::optional<std::pair<float, octomap::OcTreeKey>> SphereMap::getNearestConnectableNodeKey(octomap::point3d test_point, float odist) {
  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys       = getMaxSearchBBXBorderKeys(test_point);
  bool                                              found_near_key = false;
  octomap::OcTreeKey                                nearest_key;
  float                                             nearest_dist;
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
    float dist2 = (test_point - it->valuePtr()->pos).norm_sq();
    if (areConnectable(dist2, it->valuePtr()->radius, odist)) {
      float sphere_dist = sqrt(dist2) - it->valuePtr()->radius - odist;
      if (!found_near_key || sphere_dist < nearest_dist) {
        nearest_key    = it.getKey();
        nearest_dist   = sphere_dist;
        found_near_key = true;
      }
    }
  }
  if (found_near_key) {
    return std::make_pair(nearest_dist, nearest_key);
  }
  return std::nullopt;
}
//}

/* SphereMapPath::computePathsToGoalsWithConnectingToGraph() //{ */
std::optional<std::vector<SphereMapPath>> SphereMap::computePathsToGoalsWithConnectingToGraph(octomap::point3d start_pos, float max_dist_start_from_graph,
                                                                                              std::vector<octomap::point3d> goal_positions,
                                                                                              float max_goal_pos_dist, bool verbose,
                                                                                              bool use_precomputed_paths) {

  std::vector<SphereMapPath> res = {};
  /* std::shared_ptr<darpa_planning::PCLMap> pclmap = exploration_mapper_->getPCLMapSharedPtr(); */

  auto nearest_node_query = getNearestNodeKey(start_pos, max_dist_start_from_graph);
  if (nearest_node_query == std::nullopt) {
    ROS_WARN_COND(verbose, "[SphereMapNavigation]: start pos is not connected to graph!");
    return std::nullopt;
  }

  /* NOW FIND PATH FOR EACH GOAL POSITION */
  for (uint goal_id = 0; goal_id < goal_positions.size(); goal_id++) {
    octomap::point3d goal_pos = goal_positions[goal_id];
    /* TRY CONNECTING THE POSITION TO SPHEREMAP GRAPH */
    std::vector<std::pair<float, octomap::OcTreeKey>> sorted_nodes_nearest_to_goal =
        /* getNearestNodeKeysForEachSegment(goal_vp.position, max_goal_pos_dist, true, getObstacleDist(goal_vp.position, pclmap)); */
        getNearestNodeKeysForEachSegment(goal_pos, max_goal_pos_dist);
    /* if (sorted_nodes_nearest_to_goal.empty()) { */
    /*   ROS_WARN_COND(verbose, "[SphereMapNavigation]: goal pos is not connected to graph!"); */
    /*   if (take_nearest_segment_if_no_segment_visible) { */
    /*     sorted_nodes_nearest_to_goal = spheremap->getNearestNodeKeysForEachSegment(goal_vp.position, max_goal_pos_dist); */
    /*     if (sorted_nodes_nearest_to_goal.empty()) { */
    /*       ROS_WARN_COND(verbose, "[SphereMapNavigation]: no nodes near to goal found, goal is in unknownspace"); */
    /*       last_topopath_computation_result_ = NCR_UNKNOWNSPACE; */
    /*       return std::nullopt; */
    /*     } */
    /*   } else { */
    /*     last_topopath_computation_result_ = NCR_UNKNOWNSPACE; */
    /*     return std::nullopt; */
    /*   } */
    /* } */
    if (sorted_nodes_nearest_to_goal.empty()) {
      ROS_WARN_COND(verbose, "[SphereMapNavigation]: no nodes found in %f radius for goal at pos: (%f, %f, %f) in spheremap frame.", max_goal_pos_dist,
                    goal_pos.x(), goal_pos.y(), goal_pos.z());
      // TODO add empty path
      continue;
    }

    ROS_INFO_COND(verbose, "[SphereMapNavigation]: num of nodes to try as end goals in the graph: %lu", sorted_nodes_nearest_to_goal.size());
    for (uint g = 0; g < sorted_nodes_nearest_to_goal.size(); g++) {
      /* ROS_INFO_COND(verbose, "[SphereMapNavigation]: computing path for %u. nearest segment", g); */

      uint          current_seg_id = nodes->search(nearest_node_query.value().second)->valuePtr()->segment_id;
      uint          goal_seg_id    = nodes->search(sorted_nodes_nearest_to_goal[g].second)->valuePtr()->segment_id;
      SphereMapPath res_path;

      if (use_precomputed_paths) {
        res_path = computePath(current_seg_id, start_pos, goal_seg_id, goal_pos, true, nearest_node_query.value().second,
                               sorted_nodes_nearest_to_goal[g].second, true);
      } else {
        res_path = computeDetailedPathInSegment(nearest_node_query.value().second, sorted_nodes_nearest_to_goal[g].second, segments.begin(),
                                                planning_min_safe_dist, true);
      }
      /* ROS_INFO_COND(verbose, "[SphereMapNavigation]: found path has %lu waypoints", res_path.positions.size()); */

      if (res_path.reaches_goal) {
        ROS_INFO_COND(verbose, "[SphereMapNavigation]: returning this path for this goal");
        res.push_back(res_path);
        break;
      }
    }
  }
  return res;
}
//}

/* SphereMapPath SphereMap::computePath() //{ */
SphereMapPath SphereMap::computePath(octomap::point3d start_point, octomap::point3d goal_point, bool do_astar_in_segments, bool request_min_safedist,
                                     float min_path_safedist) {
  /* octomap::Oc */
  /* auto nearest_node_query = getNearestConnectableNodeKey(start_point, */
  /* SphereMapPath res; */
  return SphereMapPath();  // FIXME
}

SphereMapPath SphereMap::computePath(uint start_seg_id, octomap::point3d start_pos, uint goal_seg_id, octomap::point3d goal_point,
                                     bool compute_astar_in_segments, octomap::OcTreeKey node_key_nearest_to_start, octomap::OcTreeKey node_key_nearest_to_goal,
                                     bool compute_astar_economically, bool request_min_safedist, float min_path_safedist) {
  /* INIT DEFAULT PATH WITH UNREACHED FLAG */
  SphereMapPath res;
  if (segments.empty()) {
    ROS_WARN("cannot compute spheremap paths when spheremap is empty!");
    return res;
  }
  octomap::SphereMapNode* start_node_ptr;
  octomap::SphereMapNode* goal_node_ptr;
  if (compute_astar_in_segments) {
    start_node_ptr = nodes->search(node_key_nearest_to_start, 16);
    if (start_node_ptr->valuePtr()->segment_id != start_seg_id) {
      ROS_WARN("[SphereMapNavigation]: node nearest to start has different seg_id than supplied!");
      start_seg_id = start_node_ptr->valuePtr()->segment_id;
    }
    goal_node_ptr = nodes->search(node_key_nearest_to_goal, 16);
    if (goal_node_ptr->valuePtr()->segment_id != goal_seg_id) {
      ROS_WARN("[SphereMapNavigation]: node nearest to goal has different seg_id than supplied!");
      goal_seg_id = goal_node_ptr->valuePtr()->segment_id;
    }
  }

  if (start_seg_id == 0 || goal_seg_id == 0) {
    ROS_WARN("trying to find Astar path to unsegmented node!");
    return res;
  }


  if (goal_seg_id == start_seg_id) {
    res.positions    = {start_pos, goal_point};
    res.seg_ids      = {start_seg_id, goal_seg_id};
    res.reaches_goal = true;
    if (compute_astar_in_segments) {
      /* ROS_INFO("[SpheremapNavigation]: goal is in same segment as start!"); */
      return computeDetailedPathInSegment(node_key_nearest_to_start, node_key_nearest_to_goal, segments.find(start_seg_id), 0);
    }
    return res;
    /* TODO astar in segment */
  }

  float              start_h_cost = (start_pos - goal_point).norm();
  SphereMapAstarNode start_node(start_seg_id, NULL, 0, start_h_cost, start_pos, start_h_cost);

  std::list<SphereMapAstarNode>   explored         = {};
  std::list<uint>                 explored_seg_ids = {start_seg_id};
  std::set<std::pair<uint, uint>> explored_portals_;
  std::vector<SphereMapAstarNode> frontier = {start_node};
  std::vector<SphereMapPath>      tmp_detailed_paths;

  bool               reached_goal = false;
  SphereMapAstarNode goal_node;
  while (!reached_goal && frontier.size() > 0) {
    /* SORT THE FRONTIER LIST AND SELECT LOWEST COST NODE */
    std::sort(frontier.begin(), frontier.end());
    SphereMapAstarNode expanded = *frontier.begin();

    /* ERASE NODE FROM FRONTIER, ADD IT TO EXPANDED, KEEP POINTER TO IT IN EXPANDED */
    frontier.erase(frontier.begin());
    explored.push_back(expanded);
    SphereMapAstarNode* parent_ptr = &(*(--explored.end()));
    /* SphereMapAstarNode* parent_ptr = NULL; */

    /* GET ADJACENT NODES */
    std::map<uint, SphereMapSegment>::iterator seg_ptr = segments.find(expanded.seg_id2);
    if (seg_ptr == segments.end()) {
      ROS_ERROR("segment of expanded astar node does not exist, this should not happen");
      return res;
    }
    for (std::map<uint, SphereMapSegmentConnection>::iterator conn_iter = seg_ptr->second.connections.begin(); conn_iter != seg_ptr->second.connections.end();
         conn_iter++) {

      std::pair<uint, uint> portal_id_pair =
          conn_iter->first < expanded.seg_id2 ? std::make_pair(conn_iter->first, expanded.seg_id2) : std::make_pair(expanded.seg_id2, conn_iter->first);
      uint connected_seg_id = conn_iter->first;
      /* IF ADJACENT SEGMENT ALREADY IN FRONTIER OR EXPLORED, IGNORE IT */
      /* if (std::find(explored_seg_ids.begin(), explored_seg_ids.end(), connected_seg_id) != explored_seg_ids.end()) { */
      /*   continue; */
      /* } */

      /* IF PORTAL ALREADY EXPANDED, CONTINUE */
      if (explored_portals_.find(portal_id_pair) != explored_portals_.end()) {
        continue;
      }
      explored_portals_.insert(portal_id_pair);

      float safety_cost_bonus          = 0;
      float interportal_min_safedist   = planning_base_safe_dist * 2;
      float interportal_total_unsafety = 0;

      /* IF AT START, COMPUTE DETAILED ASTAR */
      if (compute_astar_in_segments && compute_astar_economically) {
        if (expanded.parent_ptr == NULL) {
          /* COMPUTE PATH TO PORTAL, BUT IGNORE START SPHEREDIST, BECAUSE COULD BE STUCK */
          auto found_cached_path_ptr = cached_search_start_paths_.find(conn_iter->first);
          if (found_cached_path_ptr == cached_search_start_paths_.end()) {
            octomap::OcTreeKey portal_goal_key = conn_iter->second.own_key;
            auto               addition_res    = cached_search_start_paths_.insert(
                std::make_pair(conn_iter->first, computeDetailedPathInSegment(node_key_nearest_to_start, portal_goal_key, seg_ptr, 0)));
            interportal_min_safedist   = addition_res.first->second.computeMinSafedist();
            interportal_total_unsafety = addition_res.first->second.total_unsafety;
          } else {
            interportal_min_safedist   = found_cached_path_ptr->second.min_safedist;
            interportal_total_unsafety = found_cached_path_ptr->second.total_unsafety;
          }
        } else {
          /* FIND INTERPORTAL PATH */
          uint interportal_id1      = fmin(expanded.seg_id1, conn_iter->first);
          uint interportal_id2      = fmax(expanded.seg_id1, conn_iter->first);
          auto interportal_path_ptr = seg_ptr->second.interportal_paths.find(std::make_pair(interportal_id1, interportal_id2));
          if (interportal_path_ptr == seg_ptr->second.interportal_paths.end()) {
            ROS_WARN("interportal path not found from %u to %u. Segment has %lu paths: and following portals", interportal_id1, interportal_id2,
                     seg_ptr->second.interportal_paths.size());
            for (std::map<std::pair<uint, uint>, SphereMapPath>::iterator pit = seg_ptr->second.interportal_paths.begin();
                 pit != seg_ptr->second.interportal_paths.end(); pit++) {
              ROS_WARN("id1: %u, id2: %u", pit->first.first, pit->first.second);
            }
            for (auto pit = seg_ptr->second.connections.begin(); pit != seg_ptr->second.connections.end(); pit++) {
              ROS_WARN("portal id: %u", pit->first);
            }

            continue;
          }

          interportal_min_safedist   = interportal_path_ptr->second.min_safedist;
          interportal_total_unsafety = interportal_path_ptr->second.total_unsafety;

          /* cached_path_index = tmp_detailed_paths.size(); */
          /* tmp_detailed_paths.push_back(interportal_path_ptr->second); */
        }
      }

      if (interportal_min_safedist < planning_base_safe_dist) {
        /* float unsafety      = calculateUnsafetyOfNode(interportal_min_safedist ); */
        /* safety_cost_bonus = spheremap_planning_safety_bias_ + spheremap_planning_safety_weight_ * unsafety; */

        safety_cost_bonus = spheremap_planning_safety_bias_ + spheremap_planning_safety_weight_ * interportal_total_unsafety;
      }

      /* CREATE NEW ASTAR NODE AND STORE TO FRONTIER*/
      octomap::point3d portal_pos1 = nodes->keyToCoord(conn_iter->second.own_key);
      octomap::point3d portal_pos2 = nodes->keyToCoord(conn_iter->second.other_key);

      /* FIND INTERPORTAL PATH */
      float g_cost = (expanded.pos2 - portal_pos1).norm() + safety_cost_bonus;
      /* float g_cost = (expanded.pos2 - portal_pos1).norm(); */
      float h_cost = (portal_pos2 - goal_point).norm();

      /* SphereMapAstarNode new_node(connected_seg_id, parent_ptr, expanded.total_g_cost + g_cost, h_cost, portal_pos, expanded.total_g_cost + g_cost +
       * h_cost);
       */
      SphereMapAstarNode new_node(expanded.seg_id2, connected_seg_id, parent_ptr, expanded.total_g_cost + g_cost, h_cost, portal_pos1, portal_pos2,
                                  conn_iter->second.own_key, conn_iter->second.other_key, expanded.total_g_cost + g_cost + h_cost);
      /* new_node.path_ptr = */
      frontier.push_back(new_node);
      /* ADD NEW FRONTIER SEGMENT ID TO EXPLORED SEGMENT ID LIST */
      explored_seg_ids.push_back(connected_seg_id);

      /* CHECK IF NEW FRONTIER SEGMENT IS GOAL */
      /* if (isSphereMapGoal(connected_seg_id, goal_type, goal_args)) { */
      if (connected_seg_id == goal_seg_id) {
        goal_node    = new_node;
        reached_goal = true;
        break;
      }
    }
  }
  if (!reached_goal) {
    return res;
  }

  if (compute_astar_in_segments) {
    /* ROS_INFO("[SpheremapNavigation]: rough path found"); */
  }

  /* BEGIN BACKTRACK */
  std::vector<int>                ids_r              = {};
  std::vector<octomap::point3d>   positions_r        = {};
  std::vector<octomap::OcTreeKey> keys_r             = {};
  uint                            num_portals_passed = 0;
  SphereMapAstarNode              current_node       = goal_node;
  while (1) {
    /* ids_r.push_back(current_node.seg_id); */
    /* positions_r.push_back(current_node.pos); */
    /* keys_r.push_back(current_node.key); */
    if (current_node.parent_ptr == NULL) {
      break;
    } else {
      ids_r.push_back(current_node.seg_id2);
      positions_r.push_back(current_node.pos2);
      keys_r.push_back(current_node.key2);

      ids_r.push_back(current_node.seg_id1);
      positions_r.push_back(current_node.pos1);
      keys_r.push_back(current_node.key1);

      current_node = *(current_node.parent_ptr);
      num_portals_passed++;
    }
  }

  if (compute_astar_in_segments) {
    /* ROS_INFO("[SpheremapNavigation]: rough path has %lu nodes", keys_r.size()); */
  }

  /* CREATE ARRAY OF ALL KEYS AND POINTS PASSED INCLUDING START KEY AND GOAL KEY */
  std::vector<uint>               ids       = {start_seg_id};
  std::vector<octomap::point3d>   positions = {start_pos};
  std::vector<octomap::OcTreeKey> keys      = {node_key_nearest_to_start};
  for (int i = ids_r.size() - 1; i > -1; i--) {
    ids.push_back(ids_r[i]);
    positions.push_back(positions_r[i]);
    keys.push_back(keys_r[i]);
  }

  ids.push_back(goal_seg_id);
  positions.push_back(goal_point);
  keys.push_back(node_key_nearest_to_goal);
  num_portals_passed++;

  /* CONSTRUCT PATH */

  res.reaches_goal = true;
  if (compute_astar_in_segments) {

    /* TODO handle if astar in segments does nto work (but that shouldnt happen so fuck that)*/
    if (!compute_astar_economically) {
      for (uint i = 0; i < keys.size(); i++) {
        octomap::SphereMapNode* tmp = nodes->search(keys[i], 16);
        /* ROS_INFO("key %u, id %u, id_true: %u", i, tmp->valuePtr()->segment_id, ids[i]); */
      }

      /* ros::Time start_time = ros::Time::now(); */
      for (uint i = 0; i < num_portals_passed; i++) {
        SphereMapPath intermediate_path = computeDetailedPathInSegment(keys[2 * i], keys[2 * i + 1], segments.find(ids[2 * i]), 0);
        /* ROS_INFO("[SpheremapNavigation]: %u/%u. intermediate path has %lu nodes", i, num_portals_passed, intermediate_path.positions.size()); */
        /* ROS_INFO("[SpheremapNavigation]: intermediate path has %f minimal safedist", */
        /*          std::min_element(intermediate_path.obstacle_dists.begin(), intermediate_path.obstacle_dists.end())); */

        if (!intermediate_path.reaches_goal) {
          ROS_ERROR("[SpheremapNavigation]: INTERMEDIATE PATH DOES NOT REACH GOAL!");
          octomap::SphereMapNode* tmp = nodes->search(keys[2 * i], 16);
          if (tmp == NULL) {
            ROS_ERROR("[SpheremapNavigation]: 1st node null");
          } else {
            ROS_ERROR("[SpheremapNavigation]: id: %u, magic flag: %u", tmp->valuePtr()->segment_id, tmp->valuePtr()->magic_flag);
          }

          tmp = nodes->search(keys[2 * i + 1], 16);
          if (tmp == NULL) {
            ROS_ERROR("[SpheremapNavigation]: 2nd node null");
          } else {
            ROS_ERROR("[SpheremapNavigation]: id: %u, magic flag: %u", tmp->valuePtr()->segment_id, tmp->valuePtr()->magic_flag);
          }


          /* res.reaches_goal = false; */
          /* return res; */
        }
        res.addPath(intermediate_path);
      }
    } else {
      /* ADD FIRST DETAILED PATH */
      uint first_connected_seg_id = ids[2];
      res.addPath(cached_search_start_paths_.find(first_connected_seg_id)->second);
      /* ROS_INFO("eco: num of keys: %lu", keys.size()); */
      /* ROS_INFO("eco: first path is %lu nodes long", cached_search_start_paths_.find(first_connected_seg_id)->second.positions.size()); */

      /* ADD ALL INTERMEDIATE CACHED PATHS */
      for (uint i = 4; i < ids.size(); i += 2) {
        std::map<uint, SphereMapSegment>::iterator seg_ptr = segments.find(ids[i - 1]);
        uint                                       interportal_id1, interportal_id2;
        bool                                       should_reverse_path = false;
        if (ids[i - 3] < ids[i]) {
          interportal_id1 = ids[i - 3];
          interportal_id2 = ids[i];
        } else {
          interportal_id1     = ids[i];
          interportal_id2     = ids[i - 3];
          should_reverse_path = true;
        }
        auto interportal_path_ptr = seg_ptr->second.interportal_paths.find(std::make_pair(interportal_id1, interportal_id2));
        if (interportal_path_ptr == seg_ptr->second.interportal_paths.end()) {
          ROS_ERROR("interportal path not found! i: %u", i);
          continue;
        }
        /* ROS_INFO("[SpheremapNavigation]: %u/%u. intermediate path has %lu nodes", i, ids.size(), interportal_path_ptr->second.positions.size()); */
        /* CHECK IF PATH NEEDS REARRANGING */
        if (should_reverse_path) {
          SphereMapPath reversed_path = interportal_path_ptr->second.reversed();
          res.addPath(reversed_path);
        } else {
          res.addPath(interportal_path_ptr->second);
        }
        /* ROS_INFO("eco: intermediate path is %lu nodes long", interportal_path_ptr->second.positions.size()); */
      }

      /* COMPUTE PATH TO LAST GOAL */
      SphereMapPath last_part_of_path = computeDetailedPathInSegment(keys[keys.size() - 2], keys[keys.size() - 1], segments.find(ids[ids.size() - 1]), 0);
      if (!last_part_of_path.reaches_goal) {
        ROS_WARN("last part of path does not reach goal");
        res.reaches_goal = false;
        return res;
      }
      res.addPath(last_part_of_path);
      /* ROS_INFO("eco: last path is %lu nodes long", last_part_of_path.positions.size()); */
    }
    /* ros::Time end_time = ros::Time::now(); */
    /* ROS_INFO("detailed astar took %f ms", (end_time - start_time).toSec() * 1000); */

    return res;
  } else {
    res.seg_ids   = ids;
    res.positions = positions;


    /* ADD GOAL TO RES PATH */
    /* res.seg_ids.push_back(goal_node.seg_id2); */
    /* res.positions.push_back(goal_point); */

    /* RETURN */
    return res;
  }
}
//}

/* SphereMapPath SphereMap::computeDetailedPathInSegment() //{ */
SphereMapPath SphereMap::computeDetailedPathInSegment(octomap::OcTreeKey start_key, octomap::OcTreeKey goal_key,
                                                      std::map<uint, SphereMapSegment>::iterator seg_ptr, float min_odist, bool override_segment_necessity) {

  /* ROS_INFO("[SpheremapNavigation]: computing detialed path in segment %u", seg_ptr->first); */
  SphereMapPath res;
  if (start_key == goal_key) {
    /* ROS_INFO("start and goal in same segment!"); */
    octomap::SphereMapNode* start_node = nodes->search(start_key, 16);
    octomap::SphereMapNode* goal_node  = nodes->search(goal_key, 16);
    res.reaches_goal                   = true;
    res.positions                      = {start_node->valuePtr()->pos, goal_node->valuePtr()->pos};
    if (!override_segment_necessity) {
      res.seg_ids = {seg_ptr->first, seg_ptr->first};
    } else {
      res.seg_ids = {0, 0};
    }
    res.obstacle_dists = {start_node->valuePtr()->radius, goal_node->valuePtr()->radius};
    return res;
  }
  /* SET ALL NODES AS UNEXPANDED */
  if (!override_segment_necessity) {
    for (uint i = 0; i < seg_ptr->second.keys.size(); i++) {
      nodes->search(seg_ptr->second.keys[i], 16)->valuePtr()->magic_flag = false;
    }
  } else {
    for (octomap::SphereMapOcTree::leaf_iterator it = nodes->begin_leafs(); it != nodes->end_leafs(); it++) {
      it->valuePtr()->magic_flag = false;
    }
  }

  /* ROS_INFO("[SpheremapNavigation]: segment to search has %lu keys", seg_ptr->second.keys.size()); */

  /* START SEARCH */
  octomap::point3d start_pos     = nodes->keyToCoord(start_key, 16);
  octomap::point3d goal_pos      = nodes->keyToCoord(goal_key, 16);
  float            start_h_cost  = (start_pos - goal_pos).norm();
  uint             search_seg_id = override_segment_necessity ? 0 : seg_ptr->first;

  {
    octomap::SphereMapNode* start_node = nodes->search(start_key, 16);
    octomap::SphereMapNode* goal_node  = nodes->search(goal_key, 16);
    if (start_node == NULL) {
      ROS_ERROR("start node null");
      return res;
    }
    if (goal_node == NULL) {
      ROS_ERROR("goal node null");
      return res;
    }
    if (start_node->valuePtr()->segment_id != goal_node->valuePtr()->segment_id && !override_segment_necessity) {
      ROS_ERROR("start and end id mismatch! %u, %u", start_node->valuePtr()->segment_id, goal_node->valuePtr()->segment_id);
      return res;
    }
  }

  /* TODO constructor */
  SphereMapDetailedAstarNode start_node(search_seg_id, NULL, 0, start_h_cost, start_pos, start_h_cost, nodes->search(start_key, 16));

  std::list<SphereMapDetailedAstarNode>   explored = {};
  std::vector<SphereMapDetailedAstarNode> frontier = {start_node};

  bool                       reached_goal = false;
  SphereMapDetailedAstarNode goal_node;
  bool                       should_sort              = true;
  uint                       num_visited_nodes        = 0;
  uint                       num_visited_search_nodes = 0;
  while (frontier.size() > 0 && !reached_goal) {
    /* SORT THE FRONTIER LIST AND SELECT LOWEST COST NODE */
    if (should_sort) {
      std::sort(frontier.begin(), frontier.end());
    }
    should_sort = false;

    SphereMapDetailedAstarNode expanded = *frontier.begin();
    num_visited_search_nodes++;

    /* ERASE NODE FROM FRONTIER, ADD IT TO EXPANDED, KEEP POINTER TO IT IN EXPANDED */
    frontier.erase(frontier.begin());
    /* if (expanded.map_node_ptr->valuePtr()->magic_flag) { */
    /*   /1* NODE IN GRAPH ALREADY EXPANDED *1/ */
    /*   continue; */
    /* } */

    should_sort = true;
    num_visited_nodes++;
    explored.push_back(expanded);
    expanded.map_node_ptr->valuePtr()->magic_flag = true;
    SphereMapDetailedAstarNode* parent_ptr        = &(*(--explored.end()));

    /* GET ADJACENT NODES */
    bool added_some_nodes = false;
    for (std::vector<octomap::OcTreeKey>::iterator conn_iter = expanded.map_node_ptr->valuePtr()->connected_keys.begin();
         conn_iter != expanded.map_node_ptr->valuePtr()->connected_keys.end(); conn_iter++) {
      octomap::SphereMapNode* adj_node_ptr = nodes->search(*conn_iter, 16);
      /* CHECK IF ADJ NODE IN CORRECT SEGMENT */
      uint connected_seg_id = adj_node_ptr->valuePtr()->segment_id;
      if (connected_seg_id != search_seg_id && !override_segment_necessity) {
        continue;
      }

      /* CHECK IF ADJ NODE ALREADY EXPANDED */
      if (adj_node_ptr->valuePtr()->magic_flag) {
        continue;
      }

      float safety_cost_bonus = 0;
      if (adj_node_ptr->valuePtr()->radius < planning_base_safe_dist) {
        /* safety_cost_bonus = 200 + 200 * (1 - ((adj_node_ptr->valuePtr()->radius - min_safe_dist) / (base_safe_dist - min_safe_dist))); */
        float unsafety    = calculateUnsafetyOfNode(adj_node_ptr->valuePtr()->radius);
        safety_cost_bonus = spheremap_planning_safety_bias_ + spheremap_planning_safety_weight_ * unsafety;
      }
      float g_cost = (expanded.pos - adj_node_ptr->valuePtr()->pos).norm() + safety_cost_bonus;
      float h_cost = (adj_node_ptr->valuePtr()->pos - goal_pos).norm();

      /* TODO constructor */
      SphereMapDetailedAstarNode new_node(connected_seg_id, parent_ptr, expanded.total_g_cost + g_cost, h_cost, adj_node_ptr->valuePtr()->pos,
                                          expanded.total_g_cost + g_cost + h_cost, adj_node_ptr);
      added_some_nodes = true;
      frontier.push_back(new_node);
      adj_node_ptr->valuePtr()->magic_flag = true;

      /* CHECK IF NEW FRONTIER SEGMENT IS GOAL */
      if (*conn_iter == goal_key) {
        goal_node    = new_node;
        reached_goal = true;
        break;
      }
    }
    should_sort = added_some_nodes;
  }
  /* CLEAN UP */
  if (override_segment_necessity) {
    for (octomap::SphereMapOcTree::leaf_iterator it = nodes->begin_leafs(); it != nodes->end_leafs(); it++) {
      it->valuePtr()->magic_flag = false;
    }
  }
  if (!reached_goal) {
    ROS_WARN("DAS: not reached goal");
    if (!override_segment_necessity) {
      ROS_WARN("DAS: num expanded search nodes: %lu, num visited search nodes: %u, num keys in segment: %lu, num visited nodes in graph: %u", explored.size(),
               num_visited_search_nodes, seg_ptr->second.keys.size(), num_visited_nodes);

      std::vector<std::vector<octomap::OcTreeKey>> key_splits;
      uint                                         check_res = checkSegmentStateAfterKeyRemoval(seg_ptr->first, key_splits);
      ROS_WARN("DAS: num splits: %lu, res: %u", key_splits.size(), check_res);
    } else {
      ROS_WARN("DAS: num expanded search nodes: %lu, num nodes: %lu", explored.size(), nodes->getNumLeafNodes());
    }

    res.reaches_goal = false;
    return res;
  }

  /* BEGIN BACKTRACK */
  reconstructSphereMapDetailedPath(res, goal_node);

  /* ADD GOAL TO RES PATH */
  /* res.seg_ids.push_back(goal_node.seg_id); */
  /* res.positions.push_back(goal_pos); */

  /* ROS_INFO("DAS: num expanded search nodes: %lu, num visited search nodes: %u, num keys in segment: %lu, num visited nodes in graph: %u, path len: %lu", */
  /*          explored.size(), num_visited_search_nodes, seg_ptr->second.keys.size(), num_visited_nodes, res.positions.size()); */
  /* RETURN */
  res.reaches_goal = true;
  return res;
}
//}

/* FRONTIERS AND SEARCH */
/* void SphereMap::updateFrontiers() //{ */
void SphereMap::updateFrontiers(bool search_whole_map, octomap::point3d pos, float box_halfsize, std::shared_ptr<octomap::OcTree> occupancy_octree_) {

  /* CHECK OLD POINTS IN BBX*/
  /* std::vector<SphereMapFrontierPoint> new_frontier_points; */
  /* BoundingBox search_bbx(box_halfsize, pos); */
  /* for(uint i = 0; i <frontier_exporation_points_.size(); i++){ */
  /*   if(!search_bbx.isPointInside(frontier_exporation_points_[i].pos)){ */
  /*     new_frontier_points.push_back(frontier_exporation_points_[i]); */
  /*   } */
  /* } */

  int           fep_num_rays  = 100;
  float         seg_rays_dist = 20;
  ros::WallTime start_, end_;
  start_ = ros::WallTime::now();

  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys                 = getMaxSearchBBXBorderKeys(pos, box_halfsize);
  uint                                              num_segments_grown       = 0;
  float                                             frontier_info_threshold_ = 0.1;
  uint                                              found_above_threshold    = 0;
  /* ROS_INFO("found above % */
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second); it != nodes->end_leafs_bbx(); it++) {
    if (it->valuePtr()->segment_id == 1 && staging_area_settings_ptr_->enabled && left_staging_area) {
      it->valuePtr()->is_frontier = false;
      continue;
    }
    auto _fep = getFrontierExplorationData(it.getCoordinate(), fep_num_rays, seg_rays_dist, occupancy_octree_);
    if (!_fep || _fep.value().perc_frontier_hits < frontier_info_threshold_) {
      it->valuePtr()->frotier_infoval = 0;
      it->valuePtr()->is_frontier     = false;
      continue;
    }
    it->valuePtr()->frotier_infoval = _fep.value().perc_frontier_hits;
    it->valuePtr()->is_frontier     = true;
    found_above_threshold++;
  }

  end_                   = ros::WallTime::now();
  float update_exec_time = (end_ - start_).toSec() * 1000;
  /* ROS_INFO("[SphereMap]: found %u frontier nodes. Execution time: %f ms", found_above_threshold, update_exec_time); */
}

//}

/* SENDING */

/* void SphereMap::computeBoundingBlockForSegment() //{ */
void SphereMapcomputeBoundingBlockForSegment(std::map<uint, SphereMapSegment> seg_ptr, std::vector<octomap::SphereMapNode*>& nodes) {
  /* calculate */
}
//}

/* std::vector<SegmapMsg> SphereMap::getSegmapMsgs() //{ */
std::vector<SegmapMsg> SphereMap::getSegmapMsgs(uint segmap_message_index, int max_data_bytes, tf2_ros::Buffer* tf_buffer, std::string segmap_frame,
                                                std::string shared_frame) {
  /* ROS_INFO("[SphereMap]: converting to message fragments"); */
  uint header_bytes  = 10;
  uint segment_bytes = 13;
  uint portal_bytes  = 4;
  uint fg_bytes      = 2 + 12 + 1;

  if (header_bytes + std::max(std::max(segment_bytes, portal_bytes), fg_bytes) > max_data_bytes) {
    ROS_ERROR("[SpheremapSending]: required fragment message size too small, cannot send some topology objects");
    ROS_ERROR("[SpheremapSending]: max data bytes: %d", max_data_bytes);
    return {};
  }

  int                    fragment_index = 0;
  SegmapMsg              msg1;
  std::vector<SegmapMsg> res = {};
  res.push_back(msg1);
  uint byte_sum = header_bytes;

  std::map<uint, uint> ids_to_indices_map;
  uint                 segment_iter_index = 0;

  for (std::map<uint, SphereMapSegment>::iterator it = segments.begin(); it != segments.end(); it++) {

    /* DONT ADD SMALL UNCONNECTED SEGS */
    if (it->second.connections.empty() && it->second.keys.size() < 30) {
      continue;
    }

    /* TODO add transformations to shared frame here */
    float                  sent_alpha;
    geometry_msgs::Vector3 sent_pos;
    sent_pos.x = it->second.center.x();
    sent_pos.y = it->second.center.y();
    sent_pos.z = it->second.center.z();
    sent_alpha = it->second.block_alpha;

    while (sent_alpha < 0) {
      sent_alpha += 2 * M_PI;
    }
    if (sent_alpha > M_PI) {
      sent_alpha = M_PI - sent_alpha;
    }

    /* COMPRESS ALPHA AND SIZE OF BLOCK */
    uint8_t sent_alpha_compressed = 255 * (sent_alpha / M_PI);
    uint8_t a_compressed          = 255 * fmin((it->second.block_a / 50), 1);
    uint8_t b_compressed          = 255 * fmin((it->second.block_b / 50), 1);
    uint8_t c_compressed          = 255 * fmin((it->second.block_c / 50), 1);


    /* COMPRESS RADIUS AND COVERAGE */
    float   radius            = it->second.bounding_sphere_radius;
    uint8_t radius_compressed = 255 * (radius / topology_mapping_settings_.merged_segment_max_size_);
    /* uint8_t coverage_compressed = 255 * it->surface_coverage_local; */
    uint8_t coverage_compressed = 255;
    if (radius > topology_mapping_settings_.merged_segment_max_size_) {
      radius_compressed = 255;
    }

    /* CHECK BYTE SIZE OF CURRENT MSG AND CUT IF NECCESSARY */
    if (byte_sum + segment_bytes > max_data_bytes) {
      byte_sum = header_bytes;
      SegmapMsg new_msg;
      res.push_back(new_msg);
      fragment_index++;
    }
    byte_sum += segment_bytes;

    res[fragment_index].segment_positions.push_back(sent_pos);
    res[fragment_index].segment_surface_coverage.push_back(coverage_compressed);
    res[fragment_index].shape_data.push_back(sent_alpha_compressed);
    res[fragment_index].shape_data.push_back(a_compressed);
    res[fragment_index].shape_data.push_back(b_compressed);
    res[fragment_index].shape_data.push_back(c_compressed);

    ids_to_indices_map.insert(std::make_pair(it->first, segment_iter_index));
    segment_iter_index++;
  }

  for (std::vector<std::pair<uint, uint>>::iterator it = portals.begin(); it != portals.end(); it++) {
    uint id1, id2;
    id1 = it->first;
    id2 = it->second;
    uint index1, index2;

    auto query_res = ids_to_indices_map.find(id1);
    if (query_res == ids_to_indices_map.end()) {
      ROS_ERROR_THROTTLE(1, "portal computation error for id1: %u, id2: %u", id1, id2);
      continue;
    }
    index1 = query_res->second;

    query_res = ids_to_indices_map.find(id2);
    if (query_res == ids_to_indices_map.end()) {
      ROS_ERROR_THROTTLE(1, "portal computation error for id1: %u, id2: %u", id1, id2);
      continue;
    }
    index2 = query_res->second;


    if (byte_sum + portal_bytes > max_data_bytes) {
      byte_sum = header_bytes;
      SegmapMsg new_msg;
      res.push_back(new_msg);
      fragment_index++;
    }
    byte_sum += portal_bytes;
    res[fragment_index].portal_ids1.push_back(index1);
    res[fragment_index].portal_ids2.push_back(index2);
  }

  uint num_total_fragments = res.size();
  for (uint i = 0; i < res.size(); i++) {
    res[i].fragment_index      = (uint16_t)i;
    res[i].num_total_fragments = (uint16_t)num_total_fragments;
    res[i].message_index       = (uint32_t)segmap_message_index;
  }

  /* msg.portal_ids1.push_back(); */
  /* ROS_INFO("[SphereMap]: conversion to LTVMap message successful"); */
  return res;
}
//}

//

/* void SphereMap::updateDistsFromHome() //{ */
void SphereMap::updateDistsFromHome() {
  if (segments.empty()) {
    return;
  }
  uint num_connected_segs = 0;
  for (std::map<uint, SphereMapSegment>::iterator it = segments.begin(); it != segments.end(); it++) {
    it->second.is_connected_to_home = false;
    it->second.dist_from_home       = 10000;  // TODO do better
  }

  octomap::SphereMapNode* start_node_ptr;
  uint                    start_seg_id = 1;

  SphereMapAstarNode start_node(start_seg_id, NULL, 0, 0, octomap::point3d(0, 0, 0), 0);

  std::list<SphereMapAstarNode>   explored         = {};
  std::list<uint>                 explored_seg_ids = {start_seg_id};
  std::vector<SphereMapAstarNode> frontier         = {start_node};

  while (frontier.size() > 0) {
    /* SORT THE FRONTIER LIST AND SELECT LOWEST COST NODE */
    std::sort(frontier.begin(), frontier.end());
    SphereMapAstarNode expanded = *frontier.begin();

    /* ERASE NODE FROM FRONTIER, ADD IT TO EXPANDED, KEEP POINTER TO IT IN EXPANDED */
    frontier.erase(frontier.begin());
    explored.push_back(expanded);
    SphereMapAstarNode* parent_ptr = &(*(--explored.end()));
    /* SphereMapAstarNode* parent_ptr = NULL; */

    /* GET ADJACENT NODES */
    std::map<uint, SphereMapSegment>::iterator seg_ptr = segments.find(expanded.seg_id2);
    num_connected_segs++;
    seg_ptr->second.dist_from_home = expanded.total_g_cost;
    if (seg_ptr == segments.end()) {
      ROS_ERROR("segment of expanded astar node does not exist, this should not happen");
      return;
    }
    for (std::map<uint, SphereMapSegmentConnection>::iterator conn_iter = seg_ptr->second.connections.begin(); conn_iter != seg_ptr->second.connections.end();
         conn_iter++) {
      uint connected_seg_id = conn_iter->first;
      /* IF ADJACENT SEGMENT ALREADY IN FRONTIER OR EXPLORED, IGNORE IT */
      if (std::find(explored_seg_ids.begin(), explored_seg_ids.end(), connected_seg_id) != explored_seg_ids.end()) {
        continue;
      }
      /* CREATE NEW ASTAR NODE AND STORE TO FRONTIER*/
      /* SphereMapPortal* portal_ptr = getPortalPtr(expanded.seg_id, connected_seg_id); */
      octomap::point3d portal_pos1 = nodes->keyToCoord(conn_iter->second.own_key);
      octomap::point3d portal_pos2 = nodes->keyToCoord(conn_iter->second.other_key);

      float g_cost = (expanded.pos2 - portal_pos1).norm();
      /* float g_cost = (expanded.pos2 - portal_pos1).norm(); */

      /* SphereMapAstarNode new_node(connected_seg_id, parent_ptr, expanded.total_g_cost + g_cost, h_cost, portal_pos, expanded.total_g_cost + g_cost +
       * h_cost);
       */
      SphereMapAstarNode new_node(expanded.seg_id2, connected_seg_id, parent_ptr, expanded.total_g_cost + g_cost, 0, portal_pos1, portal_pos2,
                                  conn_iter->second.own_key, conn_iter->second.other_key, expanded.total_g_cost + g_cost);
      frontier.push_back(new_node);
      /* ADD NEW FRONTIER SEGMENT ID TO EXPLORED SEGMENT ID LIST */
      explored_seg_ids.push_back(connected_seg_id);
    }
  }
  ROS_INFO("[SpheremapDistFromHomeComputation]: %u/%lu segments are connected to home", num_connected_segs, segments.size());
}
//}

/* reconstructSphereMapDetailedPath() //{ */
bool SphereMap::reconstructSphereMapDetailedPath(SphereMapPath& res, SphereMapDetailedAstarNode goal_node) {
  std::vector<uint>             ids_r        = {};
  std::vector<octomap::point3d> positions_r  = {};
  std::vector<float>            odists_r     = {};
  SphereMapDetailedAstarNode    current_node = goal_node;
  while (1) {
    ids_r.push_back(current_node.seg_id);
    positions_r.push_back(current_node.pos);
    odists_r.push_back(current_node.map_node_ptr->valuePtr()->radius);
    if (current_node.parent_ptr == NULL) {
      break;
    } else {
      current_node = *(current_node.parent_ptr);
    }
  }

  /* CONSTRUCT PATH */
  std::vector<uint>             ids            = {ids_r[ids_r.size() - 1]};
  std::vector<octomap::point3d> positions      = {positions_r[ids_r.size() - 1]};
  std::vector<float>            odists         = {odists_r[ids_r.size() - 1]};
  octomap::point3d              last_pos       = positions[0];
  float                         len_sum        = 0;
  float                         total_unsafety = 0;
  for (int i = ids_r.size() - 2; i > -1; i--) {
    len_sum += (positions_r[i] - last_pos).norm();
    total_unsafety += calculateUnsafetyOfNode(odists_r[i]);
    last_pos = positions_r[i];

    positions.push_back(positions_r[i]);
    ids.push_back(ids_r[i]);
    odists.push_back(odists_r[i]);
  }
  res.length_metres  = len_sum;
  res.total_unsafety = total_unsafety;
  res.seg_ids        = ids;
  res.positions      = positions;
  res.obstacle_dists = odists;

  return true;
}
//}

}  // namespace spheremap_server
