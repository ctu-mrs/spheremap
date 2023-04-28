#include <spheremap_server/mapper.h>
#include <spheremap_server/utility_functions.h>

#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <random>


namespace spheremap_server
{

/* SegMap //{ */
/* SegMap::initializeOctomaps() //{ */
void SegMap::initializeOctomaps(std::shared_ptr<octomap::OcTree> occupancy_octree_) {
  segment_octree_ = std::shared_ptr<octomap::SegmentOcTree>(new octomap::SegmentOcTree(occupancy_octree_->getResolution(), segment_octree_depth_offset_));
  segment_octree_->staging_area_settings_ptr_ = staging_area_settings_ptr_;
  segment_octree_->voxel_min_safe_distance_   = voxel_min_safe_distance_;
  initialized_segment_octree_                 = true;
}
void SegMap::initializeOctomaps(float resolution, float safedist) {
  segment_octree_                             = std::shared_ptr<octomap::SegmentOcTree>(new octomap::SegmentOcTree(resolution, segment_octree_depth_offset_));
  segment_octree_->staging_area_settings_ptr_ = staging_area_settings_ptr_;
  segment_octree_->voxel_min_safe_distance_   = voxel_min_safe_distance_;
  initialized_segment_octree_                 = true;
}
//}

/* } */

/* SegMap::update() //{ */
void SegMap::update(octomap::point3d current_position_, [[maybe_unused]] float current_heading_, std::shared_ptr<octomap::OcTree> occupancy_octree_,
                    std::shared_ptr<PCLMap> pcl_map_ptr, std::vector<int>& segs_to_update) {

  /* std::vector<pcl::PointXYZ> pcl_points = spheremap_server::octomapToPointcloud(occupancy_octree_, ); */
  /* pcl::PointCloud<pcl::PointXYZ>::Ptr simulated_pointcloud = PCLMap::pclVectorToPointcloud(pcl_points); */
  /* tmp_ptr->initKDTreeSearch(simulated_pointcloud); */

  /* std::scoped_lock lock(*mutex_ptr_); */
  int voxel_depth = segment_octree_->getTreeDepth() - segment_octree_->depth_offset;
  mutex_ptr_->lock();

  if (!initialized_segment_octree_) {
    initializeOctomaps(occupancy_octree_);
  }

  ros::WallTime start_, end_, start1_, startm, endm;
  start_  = ros::WallTime::now();
  start1_ = start_;

  // TODO put into functions
  segment_octree_->occupancy_octree = occupancy_octree_;
  segment_octree_->pcl_map_ptr      = pcl_map_ptr;

  /* IF SHOULD INIT FLEXIBLE STAGING AREA, THEN INIT THE FIRST SEGMENT //{*/
  if (staging_area_settings_ptr_ != NULL && staging_area_settings_ptr_->enabled && !initialized_staging_area) {
    for (int i = 0; i < 5; i++) {
      octomap::point3d test_point = current_position_ + getRandomPointInSphere(3);
      int              grow_res   = segment_octree_->growSegment(test_point, 10, true, topology_mapping_settings_.compactness_delta_);
      if (grow_res > -1) {
        ROS_INFO("[TopologyMapping]: initialized flexible staging area");
        initialized_staging_area = true;
        break;
      }
    }
    if (!initialized_staging_area) {
      ROS_INFO_THROTTLE(3, "[TopologyMapping]: staging area segment not yet initialized");
      mutex_ptr_->unlock();
      return;
    }
  }
  //}

  /* ONLY GROW THE FLEXIBLE STAGING AREA SEGMENT UNTIL THE WALL IS PASSED TO AVOID OTHER SEGMENTS SEEPING INTO THE STAGING AREA //{*/
  if (staging_area_settings_ptr_ != NULL && staging_area_settings_ptr_->enabled && !left_staging_area) {
    octomap::point3d deltavec  = current_position_ - staging_area_settings_ptr_->wall_pos;
    float            wall_proj = deltavec.dot(staging_area_settings_ptr_->wall_dir_outward);
    if (wall_proj > 0) {
      left_staging_area = true;
      ROS_INFO("[TopologyMapping]: robot has passed the staging area wall");
    } else {
      ROS_INFO_THROTTLE(3, "[TopologyMapping]: robot hasnt passed starting wall, not creating other segments than staging area");
      /* IF NOT YET PASSED STAGING AREA, ONLY EXPAND THE FIRST SEGMENT, IF IT HAS BEEN CREATED */
      if (segment_octree_->segments.size() > 0) {
        ROS_INFO_COND(verbose, "[TopologyMapping]: expanding staging area segment");
        segment_octree_->expandSegment(segment_octree_->segments[0].id, 10);
        segment_octree_->updateNavCenterForSegment(segment_octree_->segments[0].id);
      }
      if (!initialized_home_position) {
        if (isPointSafe(current_position_, segment_octree_->voxel_min_safe_distance_, occupancy_octree_)) {
          home_position_ = current_position_;
          ROS_INFO("segmap home pos: %f %f %f", home_position_.x(), home_position_.y(), home_position_.z());
          initialized_home_position = true;
        }
      }

      mutex_ptr_->unlock();
      return;
    }
  }

  mutex_ptr_->unlock();
  //}

  /* IF HOME POS IS NOT INITIALIZED AND ROBOT IS IN SAFE SPOT, INITIALIZE IT TO CURRENT POSITION //{ */
  if (!initialized_home_position) {
    if (isPointSafe(current_position_, segment_octree_->voxel_min_safe_distance_, occupancy_octree_)) {
      home_position_ = current_position_;
      ROS_INFO("initialized segmap home pos: %f %f %f", home_position_.x(), home_position_.y(), home_position_.z());
      initialized_home_position = true;
    }
  }
  //}

  // TRY GROWING FROM CURRENT DRONE POSITION //{
  mutex_ptr_->lock();
  int seg_id = segment_octree_->getSegmentID(current_position_);
  if (seg_id < 0) {
    if (spheremap_server::isPointSafe(current_position_, segment_octree_->voxel_min_safe_distance_, occupancy_octree_)) {
      segment_octree_->growSegment(current_position_, topology_mapping_settings_.segment_max_size_, false, topology_mapping_settings_.compactness_delta_,
                                   topology_mapping_settings_.convexity_threshold);
    } else {
      // TRY GROWING FROM NEAR POSITIONS
      int   num_sampled_points_for_growing = 0;
      float max_growing_sampling_dist      = 8;
      for (int i = 0; i < num_sampled_points_for_growing; i++) {
        octomap::point3d     test_point = current_position_ + getRandomPointInSphere(max_growing_sampling_dist);
        octomap::OcTreeNode* oc_node    = occupancy_octree_->search(test_point, voxel_depth);
        if (oc_node == NULL || occupancy_octree_->isNodeOccupied(oc_node) || spheremap_server::hasNodeUnknownChildren(oc_node, occupancy_octree_, voxel_depth)) {
          continue;
        }
        octomap::SegmentNode* seg_node = segment_octree_->search(test_point);
        if (seg_node != NULL && seg_node->getSegmentID() > -1) {
          continue;
        }
        if (spheremap_server::isPointSafe(test_point, segment_octree_->voxel_min_safe_distance_, occupancy_octree_)) {
          segment_octree_->growSegment(test_point, topology_mapping_settings_.segment_max_size_, false, topology_mapping_settings_.compactness_delta_,
                                       topology_mapping_settings_.convexity_threshold);
          break;
        }
      }
    }
  } else {
    segment_octree_->expandSegment(seg_id, topology_mapping_settings_.segment_max_size_, topology_mapping_settings_.compactness_delta_,
                                   topology_mapping_settings_.convexity_threshold);
    spheremap_server::BoundingBox bbx(50, current_position_);
    /* checkNarrowPassagesNearSegments(bbx, occupancy_octree_, pcl_map_ptr); */
  }

  mutex_ptr_->unlock();
  //}

  /* TRY GROWING FROM RAYCASTED POSITIONS //{*/
  int   num_rays_for_growth = topology_mapping_settings_.num_rays_for_growth;
  float raydist             = topology_mapping_settings_.growth_raydist;
  int   max_grown           = topology_mapping_settings_.num_max_grown;
  int   grown_num           = 0;
  bool  grew_from_raycast   = false;
  for (int i = 0; i < num_rays_for_growth; i++) {
    int rand_index = rand() % ((int)segs_to_update.size());
    int rand_id    = segs_to_update[rand_index];
    /* octomap::point3d ray_start = */

    octomap::point3d test_point = current_position_ + getRandomPointInSphere(1).normalized() * raydist;
    octomap::KeyRay  ray;
    bool             last_pos_was_in_segment = false;
    occupancy_octree_->computeRayKeys(current_position_, test_point, ray);
    for (octomap::KeyRay::iterator it = ray.begin(), end = ray.end(); it != end; ++it) {
      octomap::OcTreeNode* ocnode     = occupancy_octree_->search(*it);
      octomap::point3d     iter_point = occupancy_octree_->keyToCoord(*it);
      if (ocnode == NULL || occupancy_octree_->isNodeOccupied(ocnode)) {
        break;
      }
      octomap::SegmentNode* seg_node = segment_octree_->search(iter_point);
      if (seg_node != NULL && seg_node->getSegmentID() > -1) {
        last_pos_was_in_segment = true;
        /* FIXME - disable when not using staging area!!!! */
        if (seg_node->getSegmentID() == 0) {
          break;
        }
        continue;
      }
      if (!last_pos_was_in_segment) {
        continue;
      }
      last_pos_was_in_segment = false;
      if (spheremap_server::isPointSafe(iter_point, topology_mapping_settings_.raycast_growing_safe_distance, occupancy_octree_)) {

        mutex_ptr_->lock();
        int               grown_id = segment_octree_->growSegment(iter_point, topology_mapping_settings_.segment_max_size_, false,
                                                    topology_mapping_settings_.compactness_delta_, topology_mapping_settings_.convexity_threshold);
        octomap::Segment* new_seg  = segment_octree_->getSegmentPtr(grown_id);
        if (new_seg != NULL) {
          /* TRY MERGING WITH NEAREST SEGMENTS IMMEDIATELY */
          for (uint j = 0; j < new_seg->connected_segments_ids.size(); j++) {
            int  other_id = new_seg->connected_segments_ids[j];
            bool merge_success =
                segment_octree_->tryMerge(other_id, grown_id, topology_mapping_settings_.merged_compactness_factor_,
                                          topology_mapping_settings_.merged_segment_max_size_, topology_mapping_settings_.merged_convexity_threshold_);
            if (merge_success) {
              ROS_INFO_COND(verbose, "[SegmapUpdate]: merged raycasted seg with segment %d!", other_id);
              break;
            }
          }
        }

        mutex_ptr_->unlock();
        /* ROS_INFO("growing segment from raycast"); */
        grew_from_raycast = true;
        grown_num++;
        break;
      }
    }
    if (grown_num == max_grown) {
      break;
    }
  }
  //}

  if (segs_to_update.empty()) {
    return;
  }

  // TRY GROWING NEW SEGMENTS FROM SIDES OF CURRENT SEGMENTS //{
  int max_sidegrown = topology_mapping_settings_.num_max_grown;
  int dirs[6][3]    = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

  if (segment_octree_->segments.size() > 0) {
    std::shuffle(segs_to_update.begin(), segs_to_update.end(), std::random_device());
    int num_grownfrom = 0;
    for (uint i = 0; i < segs_to_update.size(); i++) {
      std::scoped_lock lock(*mutex_ptr_);
      /* GET SEGMENT */
      octomap::Segment* seg_ptr = segment_octree_->getSegmentPtr(segs_to_update[i]);
      if (seg_ptr == NULL || seg_ptr->is_staging_area || (seg_ptr->border_keys.size() + seg_ptr->inside_keys.size() < 2)) {
        continue;
      }
      std::vector<octomap::point3d>      potential_positions;
      std::vector<std::pair<float, int>> pairs;
      int                                index = 0;

      /* GET ADJACENT SAFE VOXELS */
      octomap::OcTreeKey test_key;
      for (uint k = 0; k < seg_ptr->border_keys.size(); k++) {
        octomap::OcTreeKey node_key       = seg_ptr->border_keys[k];
        bool               is_node_border = false;
        for (int d = 0; d < 6; d++) {
          test_key[0]                 = node_key[0] + dirs[d][0] * segment_octree_->leaf_voxel_length;
          test_key[1]                 = node_key[1] + dirs[d][1] * segment_octree_->leaf_voxel_length;
          test_key[2]                 = node_key[2] + dirs[d][2] * segment_octree_->leaf_voxel_length;
          octomap::point3d test_point = occupancy_octree_->keyToCoord(test_key, voxel_depth);

          octomap::OcTreeNode* ocnode = occupancy_octree_->search(test_key, voxel_depth);

          if (ocnode == NULL || occupancy_octree_->isNodeOccupied(ocnode) || spheremap_server::hasNodeUnknownChildren(ocnode, occupancy_octree_, voxel_depth)) {
            continue;
          }
          octomap::SegmentNode* segnode = segment_octree_->search(test_key);
          if (segnode != NULL && segnode->getSegmentID() >= 0) {
            continue;
          }

          pcl::PointXYZ pcl_point;
          /* octomap::point3d pos = test_point + getRandomPointInSphere(0.4); */
          octomap::point3d pos = test_point;
          pcl_point.x          = pos.x();
          pcl_point.y          = pos.y();
          pcl_point.z          = pos.z();
          float obstacle_dist  = pcl_map_ptr->getDistanceFromNearestPoint(pcl_point);
          if (obstacle_dist < topology_mapping_settings_.segmap_voxel_min_safe_distance_) {
            continue;
          }
          pairs.push_back(std::make_pair(-obstacle_dist, index));
          potential_positions.push_back(pos);
          index++;

          /* ROS_INFO("[TopologyUpdate]: trying expanding segment of index: %d", segs_to_update[i]); */
        }
      }
      if (potential_positions.empty()) {
        continue;
      }

      /* TRY GROWING FROM SAFE ADJACENT POSITIONS */
      std::sort(pairs.begin(), pairs.end());
      int num_grown_from_this_segment = 0;
      int max_grown_from_this_segment = 2;
      for (uint p = 0; p < pairs.size(); p++) {
        octomap::SegmentNode* segnode = segment_octree_->search(potential_positions[pairs[p].second]);
        if (segnode != NULL && segnode->getSegmentID() >= 0) {
          continue;
        }
        int grown_id = segment_octree_->growSegment(potential_positions[pairs[p].second], topology_mapping_settings_.segment_max_size_, false,
                                                    topology_mapping_settings_.compactness_delta_, topology_mapping_settings_.convexity_threshold);
        if (grown_id >= 0) {
          num_grown_from_this_segment++;
          ROS_INFO_COND(verbose, "[SegmapUpdate]: grown segment from segment %d", seg_ptr->id);
          octomap::Segment* new_seg = segment_octree_->getSegmentPtr(grown_id);

          /* TRY MERGING WITH NEAREST SEGMENTS IMMEDIATELY */
          for (uint j = 0; j < new_seg->connected_segments_ids.size(); j++) {
            int  other_id = new_seg->connected_segments_ids[j];
            bool merge_success =
                segment_octree_->tryMerge(other_id, grown_id, topology_mapping_settings_.merged_compactness_factor_,
                                          topology_mapping_settings_.merged_segment_max_size_, topology_mapping_settings_.merged_convexity_threshold_);
            if (merge_success) {
              ROS_INFO_COND(verbose, "[SegmapUpdate]: and merged it with segment %d!", other_id);
              break;
            }
          }
          if (num_grown_from_this_segment >= max_grown_from_this_segment) {
            break;
          }
        }
      }

      if (num_grown_from_this_segment > 0) {
        num_grownfrom++;
      }
      if (num_grownfrom >= max_sidegrown) {
        break;
      }
    }
    ROS_INFO_COND(verbose, "[SegmapUpdate]: grown %d segments from sides of other segments", num_grownfrom);
    /* segment_octree_->expandSegment(segs_to_update[i], topology_mapping_settings_.segment_max_size_, topology_mapping_settings_.compactness_delta_, */
    /*                                topology_mapping_settings_.convexity_threshold); */
  }
  //}

  // TRY TO RECALCULATE PORTAL POSITIONS //{
  if (segment_octree_->portals.size() > 0) {
    // CHOOSE RANDOM NEAR SEGMENT
    int               rand_index = rand() % ((int)segs_to_update.size());
    int               rand_id    = segs_to_update[rand_index];
    octomap::Segment* seg_ptr    = segment_octree_->getSegmentPtr(rand_id);

    if (seg_ptr != NULL && !seg_ptr->connected_segments_ids.empty()) {
      /* CHOOSE RANDOM PORTAL FROM THAT SEGMENT */
      int                     rand_index2 = rand() % seg_ptr->connected_segments_ids.size();
      int                     rand_id2    = seg_ptr->connected_segments_ids[rand_index2];
      octomap::SegmentPortal* portal_ptr  = segment_octree_->getPortalPtr(rand_id, rand_id2);
      if (portal_ptr == NULL) {
        ROS_ERROR("error getting portal ptr for ids %d, %d, the portal should exist", rand_id, rand_id2);
      }
      auto best_portal_pos = segment_octree_->getBestPortalPositionBetweenSegments(rand_id, rand_id2, 0.2);
      // TODO FIND BETTER SOLUTION FOR HANDLING DANGEROUSLY NARROW PORTALS
      if (best_portal_pos) {

        mutex_ptr_->lock();
        portal_ptr->position = best_portal_pos.value();
        mutex_ptr_->unlock();
      } else {
        ROS_WARN("[TopologyUpdate]: could not recalculate portal position, no position safe?");
      }
    }
  }
  //}

  // TRY TO EXPAND RANDOM SEGMENTS //{
  int num_expand_tries = topology_mapping_settings_.num_expand_tries;
  if (segment_octree_->segments.size() > 0) {
    if (num_expand_tries > (int)segs_to_update.size()) {
      num_expand_tries = segs_to_update.size();
    }
    std::shuffle(segs_to_update.begin(), segs_to_update.end(), std::random_device());
    for (int i = 0; i < num_expand_tries; i++) {

      mutex_ptr_->lock();
      /* ROS_INFO("[TopologyUpdate]: trying expanding segment of index: %d", segs_to_update[i]); */
      segment_octree_->expandSegment(segs_to_update[i], topology_mapping_settings_.segment_max_size_, topology_mapping_settings_.compactness_delta_,
                                     topology_mapping_settings_.convexity_threshold);
      mutex_ptr_->unlock();
    }
  }
  //}

  // TRY MERGING SEGMENTS //{
  int num_merge_tries = topology_mapping_settings_.num_merge_tries;
  if (topology_mapping_settings_.merging_enabled_) {
    bool              last_merge_succesful           = true;
    std::vector<uint> viable_portal_indices          = {};
    uint              viable_portal_indices_iterator = 0;

    for (int i = 0; i < num_merge_tries; i++) {
      startm = ros::WallTime::now();
      if (last_merge_succesful) {
        /* RECALC NEAR CONNECTIONS THAT CAN BE MERGED */
        viable_portal_indices = {};
        for (uint j = 0; j < segment_octree_->portals.size(); j++) {
          if (!segment_octree_->portals[j].have_segs_changed_from_last_merge_attempt) {
            continue;
          }
          for (uint k = 0; k < segs_to_update.size(); k++) {
            if (segment_octree_->portals[j].id1 == segs_to_update[k] || segment_octree_->portals[j].id2 == segs_to_update[k]) {
              viable_portal_indices.push_back(j);
              break;
            }
          }
        }

        /* SHUFFLE THEM */
        std::shuffle(viable_portal_indices.begin(), viable_portal_indices.end(), std::random_device());
        viable_portal_indices_iterator = 0;
      }

      last_merge_succesful = false;
      if (viable_portal_indices.empty()) {
        /* ROS_INFO("[SegmapUpdate]: stopping merging, no near merge candidates"); */
        break;
      }
      if (viable_portal_indices_iterator == viable_portal_indices.size()) {
        /* ROS_INFO("[SegmapUpdate]: all available merge pairs unmergable"); */
        break;
      }
      int id1 = segment_octree_->portals[viable_portal_indices[viable_portal_indices_iterator]].id1;
      int id2 = segment_octree_->portals[viable_portal_indices[viable_portal_indices_iterator]].id2;

      mutex_ptr_->lock();
      bool merge_success =
          segment_octree_->tryMerge(id1, id2, topology_mapping_settings_.merged_compactness_factor_, topology_mapping_settings_.merged_segment_max_size_,
                                    topology_mapping_settings_.merged_convexity_threshold_);

      mutex_ptr_->unlock();
      if (merge_success) {
        /* ROS_INFO("[SegmapUpdate]: merge of %d, %d successful", id1, id2); */
        last_merge_succesful = true;
        viable_portal_indices_iterator++;
      } else {
        /* ROS_INFO("[SegmapUpdate]: merge of %d, %d unsuccessful", id1, id2); */
        last_merge_succesful                                                                                                      = true;
        segment_octree_->portals[viable_portal_indices[viable_portal_indices_iterator]].have_segs_changed_from_last_merge_attempt = false;
      }
      endm = ros::WallTime::now();
      /* ROS_INFO("[SegmapUpdate]: merge time: %f ms ", (endm - startm).toSec() * 1000); */
    }
  }

  //}

  end_ = ros::WallTime::now();
  ROS_INFO("[SegmapUpdate]: Execution time: %f ms ", (end_ - start_).toSec() * 1000);
  start_ = end_;

  // TRY CHECKING SAFE UNSEGMENTED SPACE //{
  /* BoundingBox bbx(30, current_position_); */

  /* octomap::OcTreeKey start_key = occupancy_octree_->coordToKey(octomap::point3d(bbx.x1, bbx.y1, bbx.z1), 16); */
  /* octomap::OcTreeKey end_key   = occupancy_octree_->coordToKey(octomap::point3d(bbx.x2, bbx.y2, bbx.z2), 16); */

  /* /1* ITERATE OVER SURFACES IN BBX AND PROJECT THEM. BOTH EXPLORED AND UNEXPLORED. *1/ */
  /* std::vector<octomap::point3d>      positions; */
  /* std::vector<float>                 radii; */
  /* int                                index = 0; */
  /* std::vector<std::pair<float, int>> pairs; */
  /* for (octomap::OcTree::leaf_bbx_iterator it = occupancy_octree_->begin_leafs_bbx(start_key, end_key, 14); it != occupancy_octree_->end_leafs_bbx(); it++) {
   */
  /*   octomap::OcTreeNode* ocnode = occupancy_octree_->search(it.getKey()); */
  /*   if (ocnode != NULL && occupancy_octree_->isNodeOccupied(ocnode)) { */
  /*     continue; */
  /*   } */
  /*   octomap::SegmentNode* segnode = segment_octree_->search(it.getKey()); */
  /*   if (segnode != NULL && segnode->getSegmentID() >= 0) { */
  /*     continue; */
  /*   } */

  /*   pcl::PointXYZ    pcl_point; */
  /*   octomap::point3d pos = it.getCoordinate() + getRandomPointInSphere(0.4); */
  /*   pcl_point.x          = pos.x(); */
  /*   pcl_point.y          = pos.y(); */
  /*   pcl_point.z          = pos.z(); */
  /*   float obstacle_dist  = pcl_map_ptr->getDistanceFromNearestPoint(pcl_point); */
  /*   if (obstacle_dist < 0.8) { */
  /*     continue; */
  /*   } */
  /*   positions.push_back(pos); */
  /*   pairs.push_back(std::make_pair(-obstacle_dist, index)); */
  /*   index++; */
  /* } */
  /* ROS_INFO("[SegmapUpdate]: found %lu safe positions outside of segmented space", positions.size()); */
  /* std::sort(pairs.begin(), pairs.end()); */
  /* for (uint i = 0; i < pairs.size() && i < 3; i++) { */
  /*   int rand_index = (((float)rand() - 1) / (float)RAND_MAX) * pairs.size(); */
  /*   segment_octree_->growSegment(positions[rand_index], topology_mapping_settings_.segment_max_size_, false, topology_mapping_settings_.compactness_delta_,
   */
  /*                                topology_mapping_settings_.convexity_threshold); */
  /* } */
  //}

  end_ = ros::WallTime::now();
  ROS_INFO("[SegmapUpdate]: 2Execution time: %f ms ", (end_ - start_).toSec() * 1000);
  // FIXME this frees the memory
  segment_octree_->occupancy_octree = NULL;
  segment_octree_->pcl_map_ptr      = NULL;
}  // namespace spheremap_server
//}

/* SegMap::convertMsgToSegmap() //{ */
std::shared_ptr<SegMap> SegMap::convertFragmentMsgsToSegmap(std::vector<SegmapMsg> msgs, tf2_ros::Buffer* tfbuf, std::string segmap_frame,
                                                            std::string shared_frame, bool use_shared_frame) {
  /* ROS_INFO("[SegMap]: converting from message"); */
  StagingAreaSettings dummy_settings;
  dummy_settings.enabled         = false;
  std::shared_ptr<SegMap> segmap = std::shared_ptr<SegMap>(new SegMap(2, NULL));

  /* SORT THE MESSAGES FROM FIRST FRAGMENT TO LAST */
  std::vector<std::pair<uint, uint>> tmp_vec = {};
  for (uint i = 0; i < msgs.size(); i++) {
    tmp_vec.push_back(std::make_pair(msgs[i].fragment_index, i));
  }
  std::sort(tmp_vec.begin(), tmp_vec.end());
  /* ROS_INFO("[SegMap]: messages sorted"); */

  int seg_id = 0;
  for (uint m = 0; m < msgs.size(); m++) {
    SegmapMsg msg = msgs[tmp_vec[m].second];
    /* ROS_INFO("[SegMap]: msg %u, fragment index:%u, segments: %lu, portals1: %lu, portals2: %lu, frontier groups: %lu", m, msg.fragment_index, */
    /*          msg.segment_positions.size(), msg.portal_ids1.size(), msg.portal_ids2.size(), msg.frontier_group_positions.size()); */
    // TODO add special flag for segmap to know it has no octree
    for (uint i = 0; i < msg.segment_positions.size(); i++) {
      int   si      = i * 4;
      float alpha   = M_PI * (float)msg.shape_data[si] / 255.0;
      float block_a = 50 * (float)msg.shape_data[si + 1] / 255.0;
      float block_b = 50 * (float)msg.shape_data[si + 2] / 255.0;
      float block_c = 50 * (float)msg.shape_data[si + 3] / 255.0;

      float                  coverage_decompressed = (float)((uint8_t)msg.segment_surface_coverage[i]) / 255.0;
      geometry_msgs::Vector3 _pos                  = msg.segment_positions[i];
      octomap::Segment       seg(seg_id);
      octomap::point3d       pos(_pos.x, _pos.y, _pos.z);

      /* TODO add transformations to shared frame here */
      seg.center     = pos;
      seg.nav_center = pos;

      seg.block_alpha                          = alpha;
      seg.block_beta                           = 0;
      seg.block_a                              = block_a;
      seg.block_b                              = block_b;
      seg.block_c                              = block_c;
      seg.bounding_sphere_radius               = sqrt(pow(block_a, 2) + pow(block_b, 2) + pow(block_c, 2));
      std::vector<octomap::point3d> block_dirs = spheremap_server::blockAnglesToDirections(alpha, 0);
      seg.block_dirs                           = block_dirs;


      segmap->segment_octree_->segments.push_back(seg);
      seg_id++;
    }
    /* ROS_INFO("[SegMap]: segments converted"); */

    if (msg.portal_ids1.size() != msg.portal_ids2.size()) {
      ROS_ERROR("[LVTMapSending]: portal ids vectors have different sizes!");
      return NULL;
    }
    for (uint i = 0; i < msg.portal_ids1.size(); i++) {
      int id1 = msg.portal_ids1[i];
      int id2 = msg.portal_ids2[i];
      if (id1 > (int)segmap->segment_octree_->segments.size() || id2 > (int)segmap->segment_octree_->segments.size()) {
        ROS_ERROR("[SegmapSending]: portal leads to nonexistent segment");
        return NULL;
      }
      octomap::point3d       portal_pos = (segmap->segment_octree_->segments[id1].nav_center + segmap->segment_octree_->segments[id2].nav_center) * 0.5;
      octomap::SegmentPortal portal;
      portal.position = portal_pos;
      portal.id1      = id1;
      portal.id2      = id2;
      segmap->segment_octree_->portals.push_back(portal);
      segmap->segment_octree_->segments[id1].connected_segments_ids.push_back(id2);
      segmap->segment_octree_->segments[id2].connected_segments_ids.push_back(id1);
    }

    /* ROS_INFO("[SegMap]: portals converted"); */
    /* for (uint i = 0; i < msg.frontier_group_seg_ids.size(); i++) { */
    /*   int seg_id = msg.frontier_group_seg_ids[i]; */
    /*   segmap->segments_with_some_frontier_group_.push_back(seg_id); */
    /* } */
    /* if (msg.frontier_group_seg_ids.size() != msg.frontier_group_positions.size() || msg.frontier_group_positions.size() != msg.frontier_group_values.size())
     * { */
    /*   ROS_ERROR("[SegmapSending]:  frontier vectors have different sizes!"); */
    /*   return NULL; */
    /* } */
    /* for (uint i = 0; i < msg.frontier_group_seg_ids.size(); i++) { */
    /*   int seg_id = msg.frontier_group_seg_ids[i]; */
    /*   if (seg_id > (int)segmap->segment_octree_->segments.size()) { */
    /*     ROS_ERROR("[SegmapSending]: frontier leads to nonexistent segment"); */
    /*     return NULL; */
    /*   } */
    /*   octomap::point3d pos(msg.frontier_group_positions[i].x, msg.frontier_group_positions[i].y, msg.frontier_group_positions[i].z); */
    /*   /1* TRANSFORM POS *1/ */
    /*   mrs_msgs::ReferenceStamped shared_block_transform; */
    /*   shared_block_transform.reference.position.x = pos.x(); */
    /*   shared_block_transform.reference.position.y = pos.y(); */
    /*   shared_block_transform.reference.position.z = pos.z(); */
    /*   shared_block_transform.header.frame_id      = shared_frame; */
    /*   shared_block_transform.header.stamp         = ros::Time::now(); */
    /*   auto local_block_transform                  = transformer.transformSingle(segmap_frame, shared_block_transform); */
    /*   if (local_block_transform == std::nullopt) { */
    /*     ROS_ERROR("[SegmapSending]: Transformation error while transforming from %s to %s, cannot convert segmap", segmap_frame.c_str(),
     * shared_frame.c_str()); */
    /*     return NULL; */
    /*   } */
    /*   octomap::point3d transformed_pos(local_block_transform.value().reference.position.x, local_block_transform.value().reference.position.y, */
    /*                                    local_block_transform.value().reference.position.z); */

    /*   /1* ADD FRONTIER TO BUILT SEGMAP *1/ */
    /*   FrontierGroup fg; */
    /*   fg.viable_fep.pos    = transformed_pos; */
    /*   fg.viable_fep.seg_id = seg_id; */
    /*   fg.score             = msg.frontier_group_values[i] / 255.0; */
    /*   segmap->frontier_groups_.push_back(fg); */

    /*   segmap->segment_octree_->spreadFrontierValue(transformed_pos, seg_id, 15 + 10 * fg.score); */
    /*   /1* segmap->segments_with_some_frontier_group_.push_back(seg_id); *1/ */
    /* } */
    /* ROS_INFO("[SegMap]: frontiers converted"); */
  }

  /* for (uint i = 0; i < segmap->segments_with_some_frontier_group_.size(); i++) { */
  /* segmap->segment_octree_->segments[segmap->segments_with_some_frontier_group_[i]].frontier_value        = 1; */
  /* segmap->segment_octree_->segments[segmap->segments_with_some_frontier_group_[i]].global_frontier_value = 1; */
  /* } */

  /* ROS_INFO("[SegMap]: conversion from message successful"); */
  /* ROS_INFO("[SegMap]: received %lu segments that have frontiers", segmap->segments_with_some_frontier_group_.size()); */
  return segmap;
}
//}

/* SegMap::calculatePossibleSegmentsOfPointInReceivedSegmap(); //{ */
std::vector<std::pair<float, int>> SegMap::calculatePossibleSegmentsOfPointInReceivedSegmap(octomap::point3d point) {
  std::vector<std::pair<float, int>> res                  = {};
  float                              p1_radius_multiplier = 0.8;
  float                              p0_radius_multiplier = 1.2;

  /* ROS_INFO("[PossibleSegments]: segmap num segments: %d", segmap->segment_octree_->segments.size()); */
  for (std::vector<octomap::Segment>::iterator it = segment_octree_->segments.begin(); it != segment_octree_->segments.end(); it++) {
    octomap::point3d deltavec = point - it->center;

    float dist = (deltavec).norm();
    if (dist > it->bounding_sphere_radius) {
      continue;
    }
    if (it->block_dirs.size() < 3) {
      ROS_WARN("somethings wrong with block directions");
      continue;
    }
    float a = abs(deltavec.dot(it->block_dirs[0])) / it->block_a;
    float b = abs(deltavec.dot(it->block_dirs[1])) / it->block_b;
    float c = abs(deltavec.dot(it->block_dirs[2])) / it->block_c;

    /* if (a > it->block_a || b > it->block_b || c > it->block_c) { */
    /*   continue; */
    /* } */
    float max_relative_dist = std::max(std::max(a, b), c);
    if (max_relative_dist < 1) {
      float prob = (1 - max_relative_dist) / 0.2;
      /* prob = std::max(1.0, prob); */
      prob = prob > 1 ? 1 : prob;
      res.push_back(std::make_pair(prob, it->id));
    }

    /* float relative_dist = */
    /*     (dist - p1_radius_multiplier * it->bounding_sphere_radius) / ((p0_radius_multiplier - p1_radius_multiplier) * it->bounding_sphere_radius); */
    /* if (relative_dist > 1) { */
    /*   continue; */
    /* } */
    /* float p = relative_dist < 0 ? 1 : 1 - relative_dist; */
    /* res.push_back(std::make_pair(p, it->id)); */
  }
  return res;
}
//}

/* SegMap::getProbabilityPointBelongsToSomeSegment() //{ */
float SegMap::getProbabilityPointBelongsToSomeSegment(octomap::point3d point) {
  std::vector<std::pair<float, int>> pos_segments = calculatePossibleSegmentsOfPointInReceivedSegmap(point);
  float                              res          = 0;
  for (uint i = 0; i < pos_segments.size(); i++) {
    if (pos_segments[i].first > res) {
      res = pos_segments[i].first;
    }
  }
  return res;
}
//}

/* SegMap::getSegmentPtrBBXsearch(std::shared_ptr<SegMap> segmap) //{ */
/* octomap::Segment* SegMap::getSegmentPtrBBXsearch(octomap::point3d test_point) { */
/*   for (uint i = 0; i < segment_octree_->segments.size(); i++) { */
/*     octomap::Segment * ptr = &(segment_octree_->segments[i]); */
/*     octomap::point3d deltavec = test_point - ptr->center; */
/*     if(abs(deltavec.dot(ptr->block_dirs[0])) > ptr->block_a){ */
/*       continue; */
/*     } */
/*     if(abs(deltavec.dot(ptr->block_dirs[1])) > ptr->block_b){ */
/*       continue; */
/*     } */
/*     if(abs(deltavec.dot(ptr->block_dirs[2])) > ptr->block_c){ */
/*       continue; */
/*     } */
/*     return ptr; */
/*   } */
/*   return NULL; */
/* } */
//}

/* SegMap::updateExplorednessProbabilitiesBasedOnOtherSegmap(std::shared_ptr<SegMap> segmap) //{ */
void SegMap::updateExplorednessProbabilitiesBasedOnOtherSegmap(std::shared_ptr<SegMap> segmap) {
  for (uint i = 0; i < segmap->segment_octree_->segments.size(); i++) {
  }
  for (uint i = 0; i < frontier_groups_.size(); i++) {
    float prob_belongs    = segmap->getProbabilityPointBelongsToSomeSegment(frontier_groups_[i].viable_fep.pos);
    float prob_unexplored = 1 - prob_belongs;
    if (frontier_groups_[i].probability_unexplored_by_any_robot > prob_unexplored) {
      frontier_groups_[i].probability_unexplored_by_any_robot = prob_unexplored;
    }
  }
}
//}

/* SegMap::updateExplorednessProbabilitiesBasedOnOtherSegmaps() //{ */
void SegMap::updateExplorednessProbabilitiesBasedOnOtherSegmaps(std::vector<std::shared_ptr<SegMap>>& segmaps) {
  /* ROS_INFO(" update start"); */

  for (uint i = 0; i < frontier_groups_.size(); i++) {
    for (uint s = 0; s < segmaps.size(); s++) {
      std::vector<std::pair<float, int>> possible = segmaps[s]->calculatePossibleSegmentsOfPointInReceivedSegmap(frontier_groups_[i].viable_fep.pos);
      if (possible.empty()) {
        continue;
      }
      float max_frontier_prob = 0;
      for (uint j = 0; j < possible.size(); j++) {
        /* ONLY KILL THE VALUE IF IT FALLS INTO A SEGMENT WITHOUT FRONTIERS */
        float segment_frontier_prob   = segmaps[s]->segment_octree_->getSegmentPtr(possible[j].second)->frontier_value;
        float prob_belongs_to_segment = possible[j].first;
        float prob_dontblock          = 1 - (1 - segment_frontier_prob) * prob_belongs_to_segment;
        if (prob_dontblock > max_frontier_prob) {
          max_frontier_prob = prob_dontblock;
        }
      }
      float prob_unexplored = max_frontier_prob;
      if (prob_unexplored < frontier_groups_[i].probability_unexplored_by_any_robot) {
        /* ROS_INFO("[FrontierExplorationValueUpdate]: new p_unexplored: %f", prob_unexplored); */
        /* ROS_INFO("[FrontierExplorationValueUpdate]: p_belongs: %f, frontier_value: %f, p_explored: %f, new p_unexplored: %f", prob_belongs_to_segment, */
        /*          segment_frontier_prob, prob_explored_by_this_segment, prob_unexplored); */
        frontier_groups_[i].probability_unexplored_by_any_robot = prob_unexplored;
      }
    }
  }
}  // namespace spheremap_server
//}

/* SegMap::updateWithExecutedTrajectory() //{ */
void SegMap::updateWithExecutedTrajectory(std::vector<octomap::point3d>* traj, std::shared_ptr<octomap::OcTree> occupancy_octree_, float len_from_end) {
  float dist_sum = 0;
  if (traj->size() < 2) {
    return;
  }
  octomap::point3d last_point  = (*traj)[traj->size() - 1];
  int              last_id     = segment_octree_->getSegmentID(last_point);
  int              voxel_depth = occupancy_octree_->getTreeDepth() - segment_octree_->depth_offset;
  for (uint i = traj->size() - 2; i > 0; i--) {
    octomap::point3d point  = (*traj)[i];
    int              seg_id = segment_octree_->getSegmentID(point);
    dist_sum += (point - last_point).norm();
    if (dist_sum > len_from_end) {
      return;
    }
    if (last_id > -1 && seg_id > -1 && last_id != seg_id) {
      if (!segment_octree_->areSegmentsConnected(last_id, seg_id)) {
        ROS_INFO("[TrajectoryPortalGenerator]: found two segments %d, %d connected by trajectory, adding portal.", last_id, seg_id);
        octomap::point3d     midpoint = (point + last_point) * 0.5;
        octomap::OcTreeNode* ocnode   = occupancy_octree_->search(midpoint);
        if (ocnode != NULL && !occupancy_octree_->isNodeOccupied(ocnode) && !spheremap_server::hasNodeUnknownChildren(ocnode, occupancy_octree_, voxel_depth)) {
          octomap::SegmentPortal portal;
          portal.position = midpoint;
          portal.id1      = std::min(seg_id, last_id);
          portal.id2      = std::max(seg_id, last_id);
          segment_octree_->addPortal(portal);
        } else {
          ROS_WARN("[TrajectoryPortalGenerator]: midpoint is in non-free space! cannot add portal");
        }
      }
    }

    last_point = point;
    last_id    = seg_id;
  }
}
//}

/* SegMap::getSegmentsIntersectingBBX() //{ */
void SegMap::getSegmentsIntersectingBBX(std::vector<int>& seg_ids, BoundingBox bbx) {
  for (std::vector<octomap::Segment>::iterator it = segment_octree_->segments.begin(); it != segment_octree_->segments.end(); it++) {
    float d = it->bounding_sphere_radius;
    if (it->center.x() > bbx.x1 - d && it->center.x() < bbx.x2 + d && it->center.y() > bbx.y1 - d && it->center.y() < bbx.y2 + d &&
        it->center.z() > bbx.z1 - d && it->center.z() < bbx.z2 + d) {
      seg_ids.push_back(it->id);
    }
  }
}
//}

/* SegMap::getSegmentsAndBBXForVPCalc() //{ */
void SegMap::getSegmentsAndBBXForVPCalc(std::vector<int>& seg_ids, BoundingBox bbx_in, BoundingBox& bbx_out) {
  getSegmentsIntersectingBBX(seg_ids, bbx_in);
  float                         max_bounding_sphere_radius = 0;
  std::vector<octomap::point3d> centers                    = {};
  for (int seg_id : seg_ids) {
    octomap::Segment* seg = segment_octree_->getSegmentPtr(seg_id);
    centers.push_back(seg->center);
    if (seg->bounding_sphere_radius > max_bounding_sphere_radius) {
      max_bounding_sphere_radius = seg->bounding_sphere_radius;
    }
  }
  bbx_out = getPointsBoundingBox(centers);
  bbx_out.expand(max_bounding_sphere_radius);
}
//}

/* SegmMap::getHomeSegmentPtr() //{ */
std::optional<octomap::Segment*> SegMap::getHomeSegmentPtr() {
  for (std::vector<octomap::Segment>::iterator it = segment_octree_->segments.begin(); it != segment_octree_->segments.end(); it++) {
    if (it->is_staging_area) {
      return &(*it);
    }
  }
  return std::nullopt;
}
//}

/* SegMap::recalculateDistsFromHome() //{ */
void SegMap::recalculateDistsFromHome() {
  if (initialized_segment_octree_) {
    segment_octree_->recalculateDistsFromHome();
  }
}
//}

/* SegMap::checkNarrowPassagesNearSegments()  //{ */
int SegMap::checkNarrowPassagesNearSegments(BoundingBox bbx, std::shared_ptr<octomap::OcTree> occupancy_octree, std::shared_ptr<PCLMap> pcl_map_ptr) {
  int   max_voxeldist_from_segment = 10;
  float max_obstacle_dist          = voxel_min_safe_distance_;

  uint search_depth = 16 - segment_octree_->depth_offset;
  /* TRY SAMPLING POINTS IN SAFE SPACE AROUND SEGMENT AND IF A SPACE IS FOUND THAT IS FAR FROM ALL SEGMENTS */
  octomap::OcTreeKey            start_key = occupancy_octree->coordToKey(octomap::point3d(bbx.x1, bbx.y1, bbx.z1), 16);
  octomap::OcTreeKey            end_key   = occupancy_octree->coordToKey(octomap::point3d(bbx.x2, bbx.y2, bbx.z2), 16);
  std::vector<octomap::point3d> test_points;
  std::vector<octomap::point3d> safepoints;
  pcl::PointXYZ                 pcl_point;
  octomap::point3d              test_point;
  octomap::point3d              deltavec;
  float                         grouping_dist2 = 1;
  int                           filtered_dist  = 0;

  ROS_INFO("start finding points");

  for (octomap::OcTree::leaf_bbx_iterator it = occupancy_octree->begin_leafs_bbx(start_key, end_key); it != occupancy_octree->end_leafs_bbx(); it++) {
    if (it.getDepth() > search_depth) {
      continue;
    }
    if (occupancy_octree->isNodeOccupied(*it)) {
      continue;
    }
    /* if (it.getDepth() < 14) { */
    /*   continue; */
    /* } */
    octomap::SegmentNode* node = segment_octree_->search(it.getKey(), search_depth);
    // do not add already segmented keys
    if (node != NULL && node->getSegmentID() >= 0) {
      continue;
    }

    test_point = occupancy_octree->keyToCoord(it.getKey(), search_depth);
    test_points.push_back(test_point);
  }
  ROS_INFO("found %lu possible sample points", test_points.size());

  if (test_points.empty()) {
    return -2;
  }

  uint max_sampled_points = 10000;
  uint n_sampled          = 0;
  /* std::vector<octomap::point3d> examined_points = {}; */
  for (uint i = 0; i < test_points.size() && safepoints.size() < max_sampled_points; i++) {
    std::vector<octomap::point3d>::iterator it = test_points.begin();
    std::advance(it, std::rand() % test_points.size());

    /* bool near_some_point = false; */
    /* for (uint j = 0; j < safepoints.size(); j++) { */
    /*   deltavec = safepoints[j] - *it; */
    /*   if (deltavec.dot(deltavec) < grouping_dist2) { */
    /*     near_some_point = true; */
    /*     break; */
    /*   } */
    /* } */
    /* if (near_some_point) { */
    /*   filtered_dist++; */
    /*   continue; */
    /* } */

    /* SAMPLE STARTS HERE */
    n_sampled++;
    pcl_point.x   = it->x();
    pcl_point.y   = it->y();
    pcl_point.z   = it->z();
    float obsdist = segment_octree_->pcl_map_ptr->getDistanceFromNearestPoint(pcl_point);
    if (obsdist < voxel_min_safe_distance_) {
      continue;
    }

    /* POINT IS SAFE, NOW TRY RAYCAST */
    /* int seg_id = getNearestSegmentRaycasting(*it, 100, 30, occupancy_octree, segment_octree_); */
    /* if (seg_id < 0) { */
    safepoints.push_back(*it);
    /*   continue; */
    /* } */
  }

  ROS_INFO("filtered by dist: %d", filtered_dist);
  ROS_INFO("num suspicious points: %lu", safepoints.size());
  segment_octree_->debug_points_ = safepoints;

  return 0;
}
//}

//}
};  // namespace spheremap_server

namespace octomap
{

/* SegmentOcTree //{ */
/* SegmentOcTree::getSegmentID() //{ */
int SegmentOcTree::getSegmentID(octomap::point3d pt) {
  SegmentNode* node = this->search(pt);
  if (node == NULL) {
    return -1;
  }
  return node->getSegmentID();
}
//}

/* SegmentOcTree::getSegmentID() //{ */
int SegmentOcTree::getSegmentID(octomap::OcTreeKey key) {
  SegmentNode* node = this->search(key);
  if (node == NULL) {
    return -1;
  }
  return node->getSegmentID();
}
//}

/* SegmentOcTree::getSegmentIndex() //{ */
int SegmentOcTree::getSegmentIndex(int id) {
  for (uint i = 0; i < segments.size(); i++) {
    if (segments[i].id == id) {
      return i;
    }
  }
  return -1;
}
//}

/* SegmentOcTree::getSegmentPtr() //{ */
Segment* SegmentOcTree::getSegmentPtr(int id) {
  for (uint i = 0; i < segments.size(); i++) {
    if (segments[i].id == id) {
      return &segments[i];
    }
  }
  ROS_ERROR("getSegmentPtr did not find segment in segments");
  return NULL;
}
//}

/* SegmentOcTree::isKeyInVector() //{ */
bool SegmentOcTree::isKeyInVector(OcTreeKey key, const std::vector<OcTreeKey>& ptr) {
  for (uint i = 0; i < ptr.size(); i++) {
    if (ptr[i] == key) {
      return true;
    }
  }
  return false;
}
//}

/* SegmentOcTree::getBestPortalPositionBetweenSegments() //{ */
std::optional<octomap::point3d> SegmentOcTree::getBestPortalPositionBetweenSegments(int id1, int id2, float min_safe_distance) {
  Segment* s1 = getSegmentPtr(id1);
  Segment* s2 = getSegmentPtr(id2);
  if (s1 == NULL || s2 == NULL) {
    ROS_ERROR("calling getBestPortalOfSegments on segments %d, %d, one of which is null", id1, id2);
    return std::nullopt;
  }

  /* SWITCH SO THAT S1 HAS LEAST AMOUNT OF BORDER KEYS */
  if (s1->border_keys.size() > s2->border_keys.size()) {
    Segment* tmp = s1;
    s1           = s2;
    s2           = tmp;
    int tmp_id   = id1;
    id1          = id2;
    id2          = tmp_id;
  }

  /* GET POINTS OF POSSIBLE PORTAL LOCATIONS */
  std::vector<octomap::point3d> test_points = {};
  int                           dirs[6][3]  = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  // FOR EACH BORDER KEY, LOOK TO ALL ADJACENT KEYS (hope that +leaf_voxel_length workswell) FOR DIFFERENT SEGMENT
  for (uint i = 0; i < s1->border_keys.size(); i++) {
    for (uint k = 0; k < 6; k++) {
      OcTreeKey test_key = s1->border_keys[i];
      test_key[0] += dirs[k][0] * leaf_voxel_length;
      test_key[1] += dirs[k][1] * leaf_voxel_length;
      test_key[2] += dirs[k][2] * leaf_voxel_length;

      SegmentNode* node = this->search(test_key);
      if (node == NULL) {
        continue;
      }
      int adjacent_seg_id = node->getSegmentID();
      /* NODE IS NOT NULL */
      if (adjacent_seg_id != s2->id) {
        continue;
      }
      /* NODE IS FROM DIFFERENT UNCONNECTED SEGMENT */
      // TODO CREATE NON-STATIC CALL TO ISPOINTSAFE CUZ THIS IS UGLY
      octomap::point3d border_key_pos       = this->keyToCoord(s1->border_keys[i]);
      octomap::point3d test_key_pos         = this->keyToCoord(test_key);
      octomap::point3d potential_portal_pos = (border_key_pos + test_key_pos) * 0.5;
      test_points.push_back(potential_portal_pos);
    }
  }

  if (test_points.size() == 0) {
    ROS_ERROR("cannot determine portal of segments %d, %d, there are no connected voxels of the segments", id1, id2);
    ROS_ERROR("bk sizes: %lu, %lu", s1->border_keys.size(), s2->border_keys.size());
    return std::nullopt;
  }

  /* float best_dist = min_safe_distance; */
  /* bool found_safe = false; */

  /* for(int m = 0; m < 4; m++){ */
  /*   best_dist = pow(2,m) * min_safe_distance; */
  /*   std::vector<octomap::point3d> new_points = {}; */
  /*   for(uint i = 0; i < test_points.size(); i++){ */
  /*     if(spheremap_server::isPointSafe(test_points[i], best_dist, occupancy_octree_)){ */
  /*       new_points.push_back(test_points[i]); */
  /*     } */
  /*   } */
  /*   if(new_points.size() == 0){ */
  /*     break; */
  /*   } */

  /*   found_safe = true; */
  /*   test_points = new_points; */
  /* } */
  /* ROS_INFO("best achieved dist %f. returning random point in last generation", best_dist); */
  /* return test_points[0]; */


  spheremap_server::BoundingBox bbx = spheremap_server::getPointsBoundingBox(test_points);
  bbx.expand(3);
  octomap::OcTreeKey bbx_start_key = coordToKey(bbx.x1, bbx.y1, bbx.z1);
  octomap::OcTreeKey bbx_end_key   = coordToKey(bbx.x2, bbx.y2, bbx.z2);


  ros::WallTime start_, end_;
  start_ = ros::WallTime::now();
  /* GET POINTS WITH MIN DISTANCES FROM OBSTACLES */
  octomap::point3d best_point;
  float            best_dist = -1;
  for (uint i = 0; i < test_points.size(); i++) {
    pcl::PointXYZ pcl_point;
    pcl_point.x                 = test_points[i].x();
    pcl_point.y                 = test_points[i].y();
    pcl_point.z                 = test_points[i].z();
    float nearest_obstacle_dist = pcl_map_ptr->getDistanceFromNearestPoint(pcl_point);
    /* ROS_INFO("kdtree dist: %f", nearest_obstacle_dist); */
    if (nearest_obstacle_dist > best_dist) {
      best_dist  = nearest_obstacle_dist;
      best_point = test_points[i];
    }
  }
  end_ = ros::WallTime::now();
  /* double execution_time = (end_ - start_).toNSec() * 1e-6; */
  /* ROS_INFO("[INFO]: getting points took %f ms", execution_time); */


  /* ROS_INFO("best portal safety distance: %f", best_dist); */
  /* CHECK IF BEST POINT IS SUFFICIENTLY SAFE */
  if (best_dist < min_safe_distance) {
    ROS_WARN("no portal was found with sufficiently safe distance");
    return std::nullopt;
  }

  return best_point;
}
//}

/* SegmentOcTree::expandSegment() //{ */
int SegmentOcTree::expandSegment(int seg_id, float max_radius, float compactness_delta, float convexity_threshold) {
  int seg_index = getSegmentIndex(seg_id);
  if (seg_index < 0) {
    ROS_WARN("[ExpandSegment]: segment not found");
    return -1;
  }
  bool expand_convex    = convexity_threshold >= 0;
  bool expand_compact   = compactness_delta >= 0;
  bool expand_maxradius = max_radius >= 0;

  /* if(segments[seg_index].bounding_sphere_radius */
  pcl::PointXYZ pcl_point;
  pcl_point.x             = segments[seg_index].nav_center.x();
  pcl_point.y             = segments[seg_index].nav_center.y();
  pcl_point.z             = segments[seg_index].nav_center.z();
  float navcenter_obsdist = pcl_map_ptr->getDistanceFromNearestPoint(pcl_point);
  float narrow_mindist    = 0.8;
  float narrow_maxdist    = 1.4;
  if (navcenter_obsdist < narrow_maxdist) {
    ROS_INFO_COND(verbose, "navcenter obsdist: %f, marking as narrow segment, reducing constraints", navcenter_obsdist);
    segments[seg_index].is_narrow_space   = true;
    segments[seg_index].narrowness_factor = (navcenter_obsdist - narrow_mindist) / (narrow_maxdist - narrow_mindist);
    if (segments[seg_index].narrowness_factor < 0) {
      segments[seg_index].narrowness_factor = 0;
    }

    expand_convex = false;
    compactness_delta *= 3;
    max_radius /= 1 + segments[seg_index].narrowness_factor;
  } else {
    segments[seg_index].is_narrow_space   = false;
    segments[seg_index].narrowness_factor = 1;
  }

  /* IF IS STAGING AREA, COMPUTE WITH THIS INFORMATION. STAGING AREA EXPANSION CANNOT BE STOPPED */
  bool is_staging_area = false;
  if (segments[seg_index].is_staging_area) {
    is_staging_area = true;
  } else if (segments[seg_index].expansion_disabled) {
    return seg_id;
  }
  int search_depth = this->getTreeDepth() - this->depth_offset;

  octomap::point3d segment_center = segments[seg_index].center;

  int dirs[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  // reanimate the border of the segment (border nodes + last frontiers)
  std::vector<OcTreeKey> frontier           = segments[seg_index].border_keys;
  std::vector<OcTreeKey> new_frontier       = {};
  std::vector<OcTreeKey> new_inside_keys    = {};
  std::vector<OcTreeKey> dead_frontier_keys = {};

  OcTreeKey test_key;
  bool      added_some_new_voxels = false;
  for (int i = 0; i < 420; i++) {
    // calculate minimal half axis
    std::vector<octomap::point3d> border_points          = {};
    std::vector<octomap::point3d> border_points_filtered = {};
    for (uint k = 0; k < frontier.size(); k++) {
      border_points.push_back(occupancy_octree->keyToCoord(frontier[k], 14));
    }
    for (uint k = 0; k < dead_frontier_keys.size(); k++) {
      border_points.push_back(occupancy_octree->keyToCoord(dead_frontier_keys[k], 14));
    }
    /* vectors of pca are 4: mean position, largest halfaxis, middle, smallest */
    spheremap_server::filterPoints(&border_points, &border_points_filtered, segments[seg_index].bounding_sphere_radius / 3);

    std::vector<octomap::point3d> pca_res               = spheremap_server::getPCA3D(border_points_filtered);
    octomap::point3d              smallest_halfaxis_dir = pca_res[3].normalized();

    /* PROJECT ALL POINTS ONTO THE LOWERMOST AXIS AND GET THE HIGHER LENGTH OF THIS AXIS*/
    std::vector<octomap::point3d> border_points_relative = {};
    for (uint k = 0; k < border_points_filtered.size(); k++) {
      border_points_relative.push_back(border_points_filtered[k] - pca_res[0]);
    }
    float smallest_halfaxis_projection = 0;
    for (uint k = 0; k < border_points_relative.size(); k++) {
      float proj = border_points_relative[k].dot(smallest_halfaxis_dir);
      if (abs(proj) > smallest_halfaxis_projection) {
        smallest_halfaxis_projection = abs(proj);
      }
    }
    float allowed_dist;
    if (expand_compact) {
      allowed_dist = smallest_halfaxis_projection + compactness_delta;
      allowed_dist = std::min(max_radius, allowed_dist);
    } else {
      allowed_dist = max_radius;
    }
    float allowed_dist2 = pow(allowed_dist, 2);

    /* IF THE SEGMENT IS BIGGER BECAUSE OF MERGING, ACCEPT ITS BIGGER SIZE */
    /* if (segments[seg_index].has_merged_atleast_once) { */
    /*   allowed_dist  = std::max(segments[seg_index].bounding_sphere_radius, allowed_dist); */
    /*   allowed_dist2 = allowed_dist * allowed_dist; */
    /* } */

    if (frontier.size() + dead_frontier_keys.size() > 0) {
      segment_center = pca_res[0];
    } else {
      ROS_WARN("[expandSegment]: frontier size and dead frontier size are 0!");
    }

    /* IF SEGMENT IS STAGING AREA, REMOVE LIMITATION OF SIZE AND COMPACTNESS */
    if (is_staging_area) {
      allowed_dist2 = 99999999;
    }

    // begin spread
    int frontiers_created = 0;
    for (uint k = 0; k < frontier.size(); k++) {
      OcTreeKey node_key       = frontier[k];
      bool      is_node_border = false;
      for (int d = 0; d < 6; d++) {
        test_key[0]                   = node_key[0] + dirs[d][0] * leaf_voxel_length;
        test_key[1]                   = node_key[1] + dirs[d][1] * leaf_voxel_length;
        test_key[2]                   = node_key[2] + dirs[d][2] * leaf_voxel_length;
        octomap::point3d test_point   = occupancy_octree->keyToCoord(test_key, search_depth);
        octomap::point3d delta_vec    = test_point - segment_center;
        float            center_dist2 = delta_vec.dot(delta_vec);

        // if is staging area, do not expand beyond infinite wall
        if (is_staging_area) {
          octomap::point3d delta_vec2 = test_point - staging_area_settings_ptr_->wall_pos;
          float            wall_proj  = delta_vec2.dot(staging_area_settings_ptr_->wall_dir_outward);
          if (wall_proj >= 0) {
            is_node_border = true;
            continue;
          }
        }

        if (center_dist2 > allowed_dist2) {
          is_node_border = true;
          continue;
        }

        // do not add the key if it exists in any of the groups
        // TODO maybe remove, redundant
        if (isKeyInVector(test_key, new_frontier)) {
          continue;
        }
        SegmentNode* node = this->search(test_key);
        // do not add already segmented keys
        if (node != NULL) {
          int adjacent_seg_id = node->getSegmentID();
          if (adjacent_seg_id >= 0) {
            if (adjacent_seg_id != seg_id) {
              is_node_border = true;
            }
            continue;
          }
        }
        OcTreeNode* oc_node = occupancy_octree->search(test_key, search_depth);
        // do not add frontier or occupied nodes
        if (oc_node == NULL || occupancy_octree->isNodeOccupied(oc_node) || spheremap_server::hasNodeUnknownChildren(oc_node, occupancy_octree, search_depth)) {
          is_node_border = true;
          continue;
        }
        /* check safety */
        pcl::PointXYZ pcl_point;
        pcl_point.x   = test_point.x();
        pcl_point.y   = test_point.y();
        pcl_point.z   = test_point.z();
        float obsdist = pcl_map_ptr->getDistanceFromNearestPoint(pcl_point);
        if (!is_staging_area && obsdist < voxel_min_safe_distance_) {
          is_node_border = true;
          continue;
        }
        // check convexity
        int max_failed = (int)(convexity_threshold * border_points_filtered.size());
        if (expand_convex) {
          octomap::point3d hit_point;
          octomap::point3d dirvec;
          int              num_rays_failed = 0;
          bool             voxel_nonconvex = false;
          for (uint j = 0; j < border_points_filtered.size(); j++) {
            dirvec       = test_point - border_points_filtered[j];
            bool hit_res = occupancy_octree->castRay(border_points_filtered[j], dirvec, hit_point, false, dirvec.norm());
            if (hit_res || occupancy_octree->search(hit_point) == NULL) {
              num_rays_failed++;
            }
            /* octomap::KeyRay ray; */
            /* occupancy_octree->computeRayKeys(test_point, border_points_filtered[j], ray); */
            /* for (octomap::KeyRay::iterator it = ray.begin(), end = ray.end(); it != end; ++it) { */
            /*   octomap::OcTreeNode* ocnode = occupancy_octree->search(*it); */
            /*   if (ocnode == NULL || occupancy_octree->isNodeOccupied(ocnode)) { */
            /*     num_rays_failed++; */
            /*     break; */
            /*   } */
            /*   /1* octomap::SegmentNode* seg_node = this->search(*it, search_depth); *1/ */
            /*   /1* if (seg_node != NULL && seg_node->getSegmentID() != seg_id) { *1/ */
            /*   /1*   num_rays_failed++; *1/ */
            /*   /1*   break; *1/ */
            /*   /1* } *1/ */
            /* } */
            if (num_rays_failed > max_failed) {
              voxel_nonconvex = true;
              break;
            }
          }
          if (voxel_nonconvex) {
            /* ROS_INFO("expansionn hit: %d rays our of %d", ); */
            is_node_border = true;
            continue;
          }
        }

        // create segment node at test position in the segoctomap
        node = this->touchNode(test_key, search_depth);
        node->setSegmentID(seg_id);
        // add the node to the new frontier
        frontiers_created++;
        added_some_new_voxels = true;
        new_frontier.push_back(test_key);
      }
      if (is_node_border) {
        dead_frontier_keys.push_back(node_key);
      } else {
        new_inside_keys.push_back(node_key);
      }
    }
    /* ROS_INFO("created frontiers: %d", frontiers_created); */
    frontier = new_frontier;
    if (new_frontier.empty()) {
      break;
    }
    new_frontier = {};
  }

  // add the last frontier to borders in segment
  for (uint i = 0; i < frontier.size(); i++) {
    dead_frontier_keys.push_back(frontier[i]);
  }
  // save the new dead frontier (border nodes + last frontier)
  segments[seg_index].border_keys = dead_frontier_keys;
  // add the newly found inside keys
  for (uint i = 0; i < new_inside_keys.size(); i++) {
    segments[seg_index].inside_keys.push_back(new_inside_keys[i]);
  }

  /* calculate new bounding radius */
  float max_border_dist2 = 1;
  for (uint i = 0; i < segments[seg_index].border_keys.size(); i++) {
    octomap::point3d deltavec = occupancy_octree->keyToCoord(segments[seg_index].border_keys[i], 16 - depth_offset) - segment_center;
    float            dist2    = deltavec.dot(deltavec);
    if (dist2 > max_border_dist2) {
      max_border_dist2 = dist2;
    }
  }
  segments[seg_index].bounding_sphere_radius = sqrt(max_border_dist2);  // + (0.2 * pow(2, depth_offset));

  // add the center and additional data
  segments[seg_index].center = segment_center;

  int total_num_nodes = segments[seg_index].border_keys.size() + segments[seg_index].inside_keys.size();
  /* ROS_INFO("total size of created segment: %d", total_num_nodes); */
  /* ROS_INFO("number of new inside nodes: %lu", new_inside_keys.size()); */
  segments[seg_index].num_nodes = total_num_nodes;


  updateConnectionsForSegment(seg_id);
  updateNavCenterForSegment(seg_id);
  if (added_some_new_voxels) {

    /* ROS_INFO("[ExpandSegment]:" */
    std::vector<octomap::point3d> deltapoints = {};
    for (uint k = 0; k < segments[seg_index].border_keys.size(); k++) {
      deltapoints.push_back(occupancy_octree->keyToCoord(segments[seg_index].border_keys[k], 14) - segments[seg_index].center);
    }
    /* SET NEW BLOCK COORDINATES */
    spheremap_server::calculateBlockParamsForSegment(&(segments[seg_index]), deltapoints);

    setPortalsOfSegmentMergeReadiness(seg_id, true);
  }

  return seg_id;
}
//}

/* SegmentOcTree::growSegment() //{ */
int SegmentOcTree::growSegment(octomap::point3d start_pt, float max_radius, bool growing_staging_area_segment, float compactness_delta,
                               float convexity_threshold) {
  /* ROS_INFO("growing segment"); */
  int                search_depth = this->getTreeDepth() - this->depth_offset;
  octomap::OcTreeKey start_key    = occupancy_octree->coordToKey(start_pt, search_depth);
  int                seg_id       = getSegmentID(start_key);
  if (seg_id > -1) {
    /* ROS_INFO("[INFO]: There is already a cluster at this position, expanding.."); */
    return expandSegment(seg_id, max_radius, compactness_delta, convexity_threshold);
  }
  OcTreeNode* oc_node_start = occupancy_octree->search(start_key, search_depth);
  if (oc_node_start == NULL || occupancy_octree->isNodeOccupied(oc_node_start)) {
    /* ROS_INFO("[growSegment]: Starting node is occupied"); */
    return -1;
  }
  // no segment found, creating new one
  seg_id = last_segment_id;
  last_segment_id++;
  // create new segment with 1 border node and 0 inside
  SegmentNode* start_node = this->touchNode(start_key, search_depth);
  start_node->setSegmentID(seg_id);
  segments.push_back(Segment(seg_id));
  int seg_index                   = segments.size() - 1;
  segments[seg_index].border_keys = {start_key};
  segments[seg_index].inside_keys = {};
  segments[seg_index].center      = occupancy_octree->keyToCoord(start_key);
  segments[seg_index].nav_center  = occupancy_octree->keyToCoord(start_key);
  if (growing_staging_area_segment) {
    segments[seg_index].is_staging_area = true;
  }

  expandSegment(seg_id, max_radius, compactness_delta, convexity_threshold);
  return seg_id;
}
//}

/* bool SegmentOcTree::deleteSegment() //{ */
bool SegmentOcTree::deleteSegment(int seg_id) {
  /* ROS_INFO("deleting segment"); */
  bool                           found_seg = false;
  std::vector<Segment>::iterator seg_iterator;
  for (std::vector<Segment>::iterator it = segments.begin(); it != segments.end(); it++) {
    if (it->id == seg_id) {
      seg_iterator = it;
      found_seg    = true;
      break;
    }
  }
  if (!found_seg) {
    ROS_ERROR("[SegmentOcTree]: cannot delete segment of id %d, segment not in segments", seg_id);
    return false;
  }

  // RESET SEG_ID AT KEYS OF THIS SEGMENT
  uint bk_size = seg_iterator->border_keys.size();
  uint ik_size = seg_iterator->inside_keys.size();
  for (uint i = 0; i < bk_size + ik_size; i++) {
    SegmentNode* node = NULL;
    if (i < bk_size) {
      node = search(seg_iterator->border_keys[i]);
    } else {
      node = search(seg_iterator->inside_keys[i - bk_size]);
    }
    if (node == NULL) {
      ROS_ERROR("[SegmentOcTree]: node in saved keys is null, should not be so");
      continue;
    }
    node->setSegmentID(-1);
  }

  bool connection_sever_failed = false;
  while (seg_iterator->connected_segments_ids.size() > 0) {
    bool res = deleteConnections(seg_id, seg_iterator->connected_segments_ids[0]);
    if (!res) {
      connection_sever_failed = true;
      break;
    }
  }
  if (connection_sever_failed) {
    ROS_ERROR("[SegmentOcTree]: an error occured during removing connections of segment %d", seg_id);
    return false;
  }

  segments.erase(seg_iterator);
  /* ROS_INFO("[SegmentOcTree]: successfully deleted semgnet %d", seg_id); */
  return true;
}
//}

/* bool SegmentOcTree::deleteConnections() //{ */
bool SegmentOcTree::deleteConnections(int _id1, int _id2) {
  int id1, id2;
  if (_id1 < _id2) {
    id1 = _id1;
    id2 = _id2;
  } else {
    id1 = _id2;
    id2 = _id1;
  }
  int index1 = -1;
  int index2 = -1;
  for (uint i = 0; i < segments.size(); i++) {
    if (segments[i].id == id1) {
      index1 = i;
    }
    if (segments[i].id == id2) {
      index2 = i;
    }
  }
  if (index1 < 0 || index2 < 0) {
    ROS_ERROR("[SegmentOcTree]: cannot remove connection of ids %d, %d because there is not connection!", id1, id2);
    return false;
  }
  bool found_portal = false;
  for (std::vector<SegmentPortal>::iterator it = portals.begin(); it != portals.end(); it++) {
    if (it->id1 == id1 && it->id2 == id2) {
      found_portal = true;
      portals.erase(it);
      break;
    }
  }
  if (!found_portal) {
    ROS_ERROR("[SegmentOcTree]: cannot remove connection of ids %d, %d because there is not a portal of this connection. This should not happen.", id1, id2);
    return false;
  }

  bool removed_from_connection_list1 = false;
  bool removed_from_connection_list2 = false;
  for (std::vector<int>::iterator it = segments[index1].connected_segments_ids.begin(); it != segments[index1].connected_segments_ids.end(); it++) {
    if (*it == id2) {
      removed_from_connection_list1 = true;
      segments[index1].connected_segments_ids.erase(it);
      break;
    }
  }
  for (std::vector<int>::iterator it = segments[index2].connected_segments_ids.begin(); it != segments[index2].connected_segments_ids.end(); it++) {
    if (*it == id1) {
      removed_from_connection_list2 = true;
      segments[index2].connected_segments_ids.erase(it);
      break;
    }
  }
  if (!removed_from_connection_list1 || !removed_from_connection_list2) {
    ROS_ERROR("[SegmentOcTree]: cannot remove connection of ids %d, %d because one did not have the other in connected id list. This should not happen.", id1,
              id2);
    return false;
  }

  return true;
}
//}

/* SegmentOcTree::updateConnectionsForSegment() //{ */
void SegmentOcTree::updateConnectionsForSegment(int seg_id) {
  /* ROS_INFO("updating segment conn for id %d", seg_id); */
  int               dirs[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  octomap::Segment* seg_ptr    = getSegmentPtr(seg_id);
  if (seg_ptr == NULL) {
    ROS_ERROR("cannot update connections for segment of seg_id %d", seg_id);
    return;
  }
  // FOR EACH BORDER KEY, LOOK TO ALL ADJACENT KEYS (hope that +leaf_voxel_length workswell) FOR DIFFERENT SEGMENT
  std::vector<int> newly_adjacent_segs_ids = {};
  for (uint i = 0; i < seg_ptr->border_keys.size(); i++) {
    for (uint k = 0; k < 6; k++) {
      OcTreeKey test_key = seg_ptr->border_keys[i];
      test_key[0] += dirs[k][0] * leaf_voxel_length;
      test_key[1] += dirs[k][1] * leaf_voxel_length;
      test_key[2] += dirs[k][2] * leaf_voxel_length;

      SegmentNode* node = this->search(test_key);
      if (node == NULL) {
        continue;
      }
      int adjacent_seg_id = node->getSegmentID();
      /* NODE IS NOT NULL */
      /* CHECK IF ADJACENT ID IS OWN ID OR INVALID OR ALREADY CONNECTED OR IN NEW POTENTIAL ADJACENT IDS */
      if (adjacent_seg_id == seg_id || adjacent_seg_id == -1 || areSegmentsConnected(seg_id, adjacent_seg_id)) {
        continue;
      }
      if (std::find(newly_adjacent_segs_ids.begin(), newly_adjacent_segs_ids.end(), adjacent_seg_id) != newly_adjacent_segs_ids.end()) {
        continue;
      }
      /* NODE IS FROM DIFFERENT UNCONNECTED SEGMENT */
      newly_adjacent_segs_ids.push_back(adjacent_seg_id);
      /* ROS_INFO("found voxel of unconnected segment with id %d", adjacent_seg_id); */
    }
  }

  for (uint i = 0; i < newly_adjacent_segs_ids.size(); i++) {
    /* int adjacent_seg_id = newly_adjacent_segs_ids[i]; */

    auto best_portal_pos = getBestPortalPositionBetweenSegments(seg_id, newly_adjacent_segs_ids[i], 0.2);
    // TODO FIND BETTER SOLUTION FOR HANDLING DANGEROUSLY NARROW PORTALS
    if (!best_portal_pos) {
      /* ROS_WARN("could not find safe portal position for segments"); */
      continue;
    }

    SegmentPortal portal;
    portal.position = best_portal_pos.value();
    portal.id1      = std::min(seg_id, newly_adjacent_segs_ids[i]);
    portal.id2      = std::max(seg_id, newly_adjacent_segs_ids[i]);
    addPortal(portal);
  }
}
//}

/* SegmentOcTree::addPortal() //{ */
bool SegmentOcTree::addPortal(SegmentPortal portal) {
  int index1 = getSegmentIndex(portal.id1);
  int index2 = getSegmentIndex(portal.id2);
  if (index1 < 0 || index2 < 0) {
    ROS_ERROR("[addPortal]: cannot add portal, one index is negative!");
    return false;
  }
  this->portals.push_back(portal);
  segments[index1].connected_segments_ids.push_back(portal.id2);
  segments[index2].connected_segments_ids.push_back(portal.id1);
  return true;
}
//}

/* SegmentOcTree::updateNavCenterForSegment() //{ */
void SegmentOcTree::updateNavCenterForSegment(int seg_id) {
  /* ROS_INFO("updating segment nav center"); */
  float safedist  = 0.8;
  int   seg_index = getSegmentIndex(seg_id);
  if (seg_index < 0) {
    ROS_ERROR("cannot update nav_center for segment of seg_id %d", seg_id);
    return;
  }
  /* IF CENTER IS SAFE, SET NAV_CENTER = CENTER */
  if (spheremap_server::isPointSafe(segments[seg_index].center, 0.8, occupancy_octree)) {
    segments[seg_index].nav_center         = segments[seg_index].center;
    segments[seg_index].nav_center_is_safe = true;
    /* ROS_INFO("nav_center set to center"); */
    return;
  }
  /* TRY TO FIND NAV CENTER IN SPHERES OF INCRESING RADIUS AROUND CENTER */
  /* TODO parametrize */
  float maxdist = 20;
  for (int i = 1; i < 10; i++) {
    float radius = maxdist * ((double)i) / 10;
    /* ROS_INFO("nav_center searching in radius %f around center", radius); */
    for (int k = 0; k < 15; k++) {
      octomap::point3d test_point = segments[seg_index].center + spheremap_server::getRandomPointInSphere(radius);
      if (this->search(test_point) == NULL || this->search(test_point)->getSegmentID() != seg_index) {
        continue;
      }
      if (!spheremap_server::isPointSafe(test_point, 0.8, occupancy_octree)) {
        continue;
      }
      segments[seg_index].nav_center         = test_point;
      segments[seg_index].nav_center_is_safe = true;
      return;
    }
  }
  /* IF CURRENT NAV CENTER IS OK, DONT CHANGE IT */
  if (spheremap_server::isPointSafe(segments[seg_index].nav_center, 0.8, occupancy_octree)) {
    return;
  }
  segments[seg_index].nav_center         = segments[seg_index].center;
  segments[seg_index].nav_center_is_safe = false;
}
//}

/* int SegmentOcTree::getNearestSegment() //{ */
int SegmentOcTree::getNearestSegment(octomap::point3d pos, float max_euclid_dist, bool plan_to_unreachable_point) {
  /* CHECK IF POINT IS IN SEGMENT */
  if (this->search(pos) != NULL) {
    int res = this->search(pos)->getSegmentID();
    if (res != -1) {
      /* ROS_INFO("point is in segment, returning that segment id %d", res); */
      return res;
    }
  }
  /* TODO CHECK IF ADJACENT KEYS ARE IN SEGMENT */
  /* GET NEAREST NAVIGABLE SEGMENT */
  std::vector<std::pair<float, int>> near_segments    = {};
  float                              max_euclid_dist2 = max_euclid_dist * max_euclid_dist;
  for (uint i = 0; i < segments.size(); i++) {
    if (!segments[i].nav_center_is_safe) {
      continue;
    }
    octomap::point3d deltavec = pos - segments[i].nav_center;
    float            dist2    = deltavec.dot(deltavec);
    if (dist2 > max_euclid_dist2) {
      continue;
    }
    /* TODO CHECK REACHABILITY */
    if (plan_to_unreachable_point) {
    }
    near_segments.push_back(std::make_pair(sqrt(dist2), segments[i].id));
  }
  if (near_segments.size() == 0) {
    ROS_WARN("no near segment found");
    return -1;
  }
  std::sort(near_segments.begin(), near_segments.end());
  /* ROS_INFO("nearest segment dist: %f, id: %d", near_segments[0].first, near_segments[0].second); */
  return near_segments[0].second;
}
//}

/* SegmentOcTree::areSegmentsConnected() //{ */
bool SegmentOcTree::areSegmentsConnected(int id1, int id2) {
  int index1 = getSegmentIndex(id1);
  for (uint i = 0; i < segments[index1].connected_segments_ids.size(); i++) {
    if (segments[index1].connected_segments_ids[i] == id2) {
      return true;
    }
  }
  return false;
}
//}


/* SegmentPortal * SegmentOcTree::getPortalPtr() //{ */
SegmentPortal* SegmentOcTree::getPortalPtr(int id1, int id2) {
  if (id1 > id2) {
    int tmp = id1;
    id1     = id2;
    id2     = tmp;
  }

  for (uint i = 0; i < portals.size(); i++) {
    if (portals[i].id1 == id1 && portals[i].id2 == id2) {
      return &(portals[i]);
    }
  }

  return NULL;
}
//}

/* float SegmentOcTree::getHeuristicCost() //{ */
float SegmentOcTree::getHeuristicCost(octomap::point3d current_plan_position, SegmentAstarHeuristic heuristic_type, void* args) {
  if (heuristic_type == SegmentAstarHeuristic::EULER_TO_POINT) {
    return (*((octomap::point3d*)args) - current_plan_position).norm();
  }
  return -1;
}
//}

/* bool SegmentOcTree::isSegmentGoal() //{ */
bool SegmentOcTree::isSegmentGoal(int seg_id, SegmentAstarGoalCondition goal_type, void* args) {
  if (goal_type == SegmentAstarGoalCondition::ONE_SEGMENT) {
    if ((*(int*)args) == seg_id) {
      return true;
    }
    return false;
  }
  return false;
}
//}

/* SegmentOcTree::getSegmentsInSphere() //{ */
std::vector<int> SegmentOcTree::getSegmentsInSphere(octomap::point3d center, float radius) {
  std::vector<int> res = {};
  for (Segment s : segments) {
    if ((center - s.center).norm() - radius - s.bounding_sphere_radius < 0) {
      res.push_back(s.id);
    }
  }
  return res;
}
//}

/* SegmentOcTree::getSegmentsJoinedKeys() //{ */
bool SegmentOcTree::getSegmentsJoinedKeys(int id1, int id2, std::vector<octomap::OcTreeKey>& new_border_keys,
                                          std::vector<octomap::OcTreeKey>& new_inside_keys) {
  Segment* s1 = getSegmentPtr(id1);
  Segment* s2 = getSegmentPtr(id2);
  if (s1->inside_keys.size() > s2->inside_keys.size()) {
    new_inside_keys = s1->inside_keys;
    new_inside_keys.insert(new_inside_keys.end(), s2->inside_keys.begin(), s2->inside_keys.end());
  } else {
    new_inside_keys = s2->inside_keys;
    new_inside_keys.insert(new_inside_keys.end(), s1->inside_keys.begin(), s1->inside_keys.end());
  }

  int       total_size = s1->border_keys.size() + s2->border_keys.size();
  int       s1bksize   = s1->border_keys.size();
  int       dirs[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  OcTreeKey test_key;
  OcTreeKey key;
  bool      some_borders_turned_to_inside_keys = false;
  for (int i = 0; i < total_size; i++) {
    key = (i >= s1bksize ? s2->border_keys[i - s1bksize] : s1->border_keys[i]);

    bool has_adjacent_voxel_with_different_id_than_1or2 = false;
    for (int k = 0; k < 6; k++) {
      test_key = key;
      test_key[0] += dirs[k][0] * leaf_voxel_length;
      test_key[1] += dirs[k][1] * leaf_voxel_length;
      test_key[2] += dirs[k][2] * leaf_voxel_length;

      SegmentNode* node = this->search(test_key);
      if (node == NULL) {
        has_adjacent_voxel_with_different_id_than_1or2 = true;
        break;
      }
      int adjacent_seg_id = node->getSegmentID();
      if (adjacent_seg_id != id1 && adjacent_seg_id != id2) {
        has_adjacent_voxel_with_different_id_than_1or2 = true;
        break;
      }
    }
    if (has_adjacent_voxel_with_different_id_than_1or2) {
      new_border_keys.push_back(key);
    } else {
      new_inside_keys.push_back(key);
      some_borders_turned_to_inside_keys = true;
    }
  }

  /* [ INFO] [1619285794.271580314, 19.600000000]: [SegmapUpdate]: bk sizes: 740, 17, 753 */
  /* [ INFO] [1619285794.271600987, 19.600000000]: [SegmapUpdate]: ik sizes: 643, 0, 647 */
  /* [ INFO] [1619285794.271612721, 19.600000000]: [SegmapUpdate]: total sizes: 1383, 660, 1400 */

  /* WTF */
  /* [ INFO] [1619289128.910882768, 85.652000000]: [SegmapUpdate]: bk sizes: 878, 2, 836 */
  /* [ INFO] [1619289128.910953307, 85.652000000]: [SegmapUpdate]: ik sizes: 1177, 0, 1221 */
  /* [ INFO] [1619289128.911192233, 85.652000000]: [SegmapUpdate]: total sizes: 2055, 2, 2057 */

  int totalsize1 = s1->inside_keys.size() + s1->border_keys.size();
  int totalsize2 = s2->inside_keys.size() + s2->border_keys.size();
  int totalsize3 = new_border_keys.size() + new_inside_keys.size();
  if (totalsize3 != totalsize1 + totalsize2) {
    ROS_ERROR("TOTAL SIZE MISMATCH");
  }

  return some_borders_turned_to_inside_keys;
}
//}

/* SegmentOcTree::tryMerge() //{ */
bool SegmentOcTree::tryMerge(int id1, int id2, float compactness_index, float max_joined_size, float convexity_threshold) {
  Segment* s1 = getSegmentPtr(id1);
  Segment* s2 = getSegmentPtr(id2);
  if (s1 == NULL || s2 == NULL) {
    ROS_ERROR("one of segments for merge is null");
  }
  if (s1->is_staging_area || s2->is_staging_area) {
    return false;
  }

  /* ROS_INFO("[SegmapUpdate]: can merge, merging segments %d, %d", id1, id2); */
  if (s1->border_keys.size() + s1->inside_keys.size() < s2->inside_keys.size() + s2->border_keys.size()) {
    /* IF SEGMENT TWO IS BIGGER, SET IT AS THE MASTER SEGMENT */
    int      tmp_id  = id1;
    Segment* tmp_ptr = s1;
    s1               = s2;
    s2               = tmp_ptr;
    id1              = id2;
    id2              = tmp_id;
  }


  /* CHECK IF BOTH SEGMENTS ARE NARROW */
  bool is_larger_narrow  = s1->is_narrow_space;
  bool is_smaller_narrow = s2->is_narrow_space;
  if (is_smaller_narrow && is_larger_narrow) {
    convexity_threshold = -1;
    compactness_index *= 2;
    max_joined_size /= 3;
  } else if (is_smaller_narrow || is_larger_narrow) {
    /* convexity_threshold *= 2; */
  }

  /* GET JOINED KEYS */
  std::vector<octomap::OcTreeKey> new_border_keys = {};
  std::vector<octomap::OcTreeKey> new_inside_keys = {};
  getSegmentsJoinedKeys(id1, id2, new_border_keys, new_inside_keys);

  /* INIT VECTORS */
  int                           search_depth   = 16 - depth_offset;
  std::vector<octomap::point3d> border_points1 = {};
  std::vector<octomap::point3d> border_points2 = {};
  std::vector<octomap::point3d> border_points_filtered1;
  std::vector<octomap::point3d> border_points_filtered2;
  std::vector<octomap::point3d> border_points_filtered;
  std::vector<octomap::point3d> border_points_relative = {};
  std::vector<octomap::point3d> pca_res                = {};
  /* TRANSFORM KEYS TO POINTS */
  for (uint i = 0; i < s1->border_keys.size(); i++) {
    border_points1.push_back(occupancy_octree->keyToCoord(s1->border_keys[i], search_depth));
  }
  for (uint i = 0; i < s2->border_keys.size(); i++) {
    border_points2.push_back(occupancy_octree->keyToCoord(s2->border_keys[i], search_depth));
  }

  /* FILTER BORDER POINTS */
  if (s1->bounding_sphere_radius < 5) {
    border_points_filtered = border_points1;
  } else {
    spheremap_server::filterPoints(&border_points1, &border_points_filtered, s1->bounding_sphere_radius / 4);
  }
  if (s2->bounding_sphere_radius < 5) {
    border_points_filtered2 = border_points2;
  } else {
    spheremap_server::filterPoints(&border_points2, &border_points_filtered2, s2->bounding_sphere_radius / 4);
  }

  border_points_filtered.insert(border_points_filtered.end(), border_points_filtered2.begin(), border_points_filtered2.end());


  /* GET PCA OF NEW BORDER KEYS */
  if (compactness_index > 0) {
    pca_res = spheremap_server::getPCA3D(border_points_filtered);
    /* GET PCA ANALYSIS AND CHECK IF THE RATIO OF FIRST TWO HALFAXES IS OK */
    float            halfaxis1_projection = 0;
    float            halfaxis2_projection = 0;
    octomap::point3d halfaxis1            = pca_res[1].normalized();
    octomap::point3d halfaxis2            = pca_res[3].normalized();
    for (uint k = 0; k < border_points_filtered.size(); k++) {
      octomap::point3d deltapoint = (border_points_filtered[k] - pca_res[0]);
      border_points_relative.push_back(deltapoint);
      float proj1 = deltapoint.dot(halfaxis1);
      float proj2 = deltapoint.dot(halfaxis2);
      if (abs(proj1) > halfaxis1_projection) {
        halfaxis1_projection = abs(proj1);
      }
      if (abs(proj2) > halfaxis2_projection) {
        halfaxis2_projection = abs(proj2);
      }
    }

    if (halfaxis2_projection == 0) {
      ROS_WARN_COND(verbose, "[SegmapUpdate]: halfaxis of merged segment is zero, this should not happen");
      return false;
    }
    if (halfaxis1_projection / halfaxis2_projection > compactness_index) {
      /* ROS_INFO("[SegmapUpdate]: cannot merge, ratio of PCA axis 1 and 2 is %f", halfaxis1_projection / halfaxis2_projection); */
      return false;
    }
  } else {
    /* CALCULATE CENTER AND DELTAPOINTS WITHOUT PCA */
    octomap::point3d center(0, 0, 0);
    for (uint k = 0; k < border_points_filtered.size(); k++) {
      center += border_points_filtered[k];
    }
    center = center * (1.0 / ((float)border_points_filtered.size()));
    pca_res.push_back(center);
    for (uint k = 0; k < border_points_filtered.size(); k++) {
      octomap::point3d deltapoint = (border_points_filtered[k] - pca_res[0]);
      border_points_relative.push_back(deltapoint);
    }
  }

  float new_bounding_sphere_radius = 0;
  float maxdist2                   = 0;
  for (octomap::point3d p : border_points_relative) {
    octomap::point3d deltavec = p;
    float            dist2    = deltavec.dot(deltavec);
    if (dist2 > maxdist2) {
      maxdist2 = dist2;
    }
  }

  /* CHECK IF SMALLER SEGMENT IS SMALL AND CANNOT GROW ANYMORE */
  if (s2->inside_keys.size() < 7 && (isSegmentCompletelyEnclosedByUnexpandableSpace(s2))) {
    if (isSegmentCompletelyEnclosedByUnexpandableSpace(s2)) {
      ROS_INFO_COND(true, "[SegmapUpdate]: segment is small and unexpandable, relaxing convexity constraints a lot");
      convexity_threshold = -1;
    } else if (isSegmentInsideBBXOfSegment(s1, s2)) {
      ROS_INFO_COND(true, "[SegmapUpdate]: segment is small and either unexpandable or inside bbx of larger segment, relaxing convexity constraints");
      convexity_threshold *= 2;
    }
  }

  /* CHECK MAX SIZE */
  new_bounding_sphere_radius = sqrt(maxdist2);
  float centerdist           = (s1->center - s2->center).norm();
  if (centerdist < s1->bounding_sphere_radius - s2->bounding_sphere_radius) {
    ROS_INFO_COND(verbose, "[SegmapUpdate]: smaller segment is completely engulfed by bounding sphere, ignoring max joined size");
  } else {
    if (max_joined_size > 0 && new_bounding_sphere_radius > max_joined_size) {
      ROS_INFO_COND(verbose, "[SegmapUpdate]: cannot merge, merged segment would have size radius: %f. Max bounding sphere radius: %f",
                    new_bounding_sphere_radius, max_joined_size);
      return false;
    }
  }

  /* CHECK CONVEXITY */
  int num_rays_failed = 0;
  int num_total_rays  = 0;

  if (convexity_threshold >= 0) {
    octomap::point3d hit_point, dirvec;
    for (uint i = 0; i < border_points_filtered.size(); i++) {
      for (uint j = i + 1; j < border_points_filtered.size(); j++) {
        num_total_rays++;

        dirvec       = border_points_filtered[i] - border_points_filtered[j];
        bool hit_res = occupancy_octree->castRay(border_points_filtered[j], dirvec, hit_point, false, dirvec.norm());
        if (hit_res || occupancy_octree->search(hit_point) == NULL) {
          num_rays_failed++;
        }
      }
    }
    float perc_hit = num_rays_failed / ((float)num_total_rays);
    if (perc_hit > convexity_threshold) {
      return false;
    }
    if (perc_hit > convexity_threshold) {
      /* ALSO CHECK IF THE SMALLER SEGMENT IS COMPLETELY IN BBX OF LARGER AND IF SO, MERGE ANYWAY */
    }
  }

  /* CAN MERGE, THEREFORE MERGE */
  /* GO THRU ALL PORTALS THAT WERE CONNECTED TO S2 */
  /* REMOVE THOSE PORTALS AND IF THE CONNECTED SEGMENTS WERE NOT CONNECTED TO S1, ADD NEW PORTALS FROM THEM TO S1 */
  std::vector<SegmentPortal>::iterator pit = portals.begin();
  while (pit != portals.end()) {
    if (pit->id1 != id2 && pit->id2 != id2) {
      ++pit;
      continue;
    }
    /* ROS_INFO("found portal connecting to s2 with ids: %d, %d", pit->id1, pit->id2); */
    /* ROS_INFO("position %f, %f, %f", pit->position.x(), pit->position.y(), pit->position.z()); */
    int other_segment_id;
    if (pit->id1 == id2) {
      other_segment_id = pit->id2;
    } else {
      other_segment_id = pit->id1;
    }
    Segment* connected_seg = getSegmentPtr(other_segment_id);
    /* FIND OUT IF THE CONNECTED SEGMENT IS S1 OR CONNECTED TO S1 */
    bool should_remove_portal = true;
    /* if (other_segment_id == id1) { */
    /*   should_remove_portal = true; */
    /* } else { */
    /*   for (std::vector<int>::iterator it = connected_seg->connected_segments_ids.begin(); it != connected_seg->connected_segments_ids.end(); it++) { */
    /*     if (*it == id1) { */
    /*       should_remove_portal = true; */
    /*       break; */
    /*     } */
    /*   } */
    /* } */
    std::vector<int>::iterator s2_id_iterator = std::find(connected_seg->connected_segments_ids.begin(), connected_seg->connected_segments_ids.end(), id2);
    if (s2_id_iterator == connected_seg->connected_segments_ids.end()) {
      ROS_ERROR("portal merging error");
    }
    if (should_remove_portal) {
      /* REMOVE PORTAL - REMOVE IT FROM LIST AND REMOVE THE CONNECTION TO S2 FROM CONNECTED SEGMENT */
      /* ROS_INFO("[SegmapUpdate]: removing portal, portals size: %lu, cs size: %lu", portals.size(), connected_seg->connected_segments_ids.size()); */
      connected_seg->connected_segments_ids.erase(s2_id_iterator);
      /* ROS_INFO("[SegmapUpdate]: removed from ids of connected segment"); */
      pit = portals.erase(pit);
      /* ROS_INFO("[SegmapUpdate]: removed portal, portals size: %lu, cs size: %lu", portals.size(), connected_seg->connected_segments_ids.size()); */
    } else {
      /* REMAP PORTAL TO S1 FROM S2 - JUST CHANGE THE IDS OF PORTAL AND IN CONNECTED_IDS OF CONNECTED SEGMENT */
      /* ROS_INFO("[SegmapUpdate]: remapping portal to %d, %d", other_segment_id, id1); */
      *s2_id_iterator = id1;
      if (other_segment_id < id1) {
        pit->id1 = other_segment_id;
        pit->id2 = id1;
      } else {
        pit->id1 = id1;
        pit->id2 = other_segment_id;
      }
      ++pit;
    }
  }

  /* SET KEYS OF ASSIMILATED SEGMENT TO ID OF BIGGER SEGMENT*/
  for (std::vector<OcTreeKey>::iterator it = new_border_keys.begin(); it != new_border_keys.end(); it++) {
    SegmentNode* node_ptr = search(*it);
    node_ptr->setSegmentID(s1->id);
  }
  for (std::vector<OcTreeKey>::iterator it = new_inside_keys.begin(); it != new_inside_keys.end(); it++) {
    SegmentNode* node_ptr = search(*it);
    node_ptr->setSegmentID(s1->id);
  }


  /* ERASE SEGMENT */
  // BIG TODO change segments to list because of the erasing
  /* ROS_INFO("[SegmapUpdate]: erasing merged segment %d, size of segments vector: %lu", id2, segments.size()); */
  for (std::vector<Segment>::iterator it = segments.begin(); it != segments.end(); it++) {
    if (it->id == id2) {
      segments.erase(it);
      /* ROS_INFO("[SegmapUpdate]: erased"); */
      break;
    }
  }
  s1 = getSegmentPtr(id1);
  /* SET NEW KEYS TO NEW MERGED SEGMENT */
  s1->border_keys             = new_border_keys;
  s1->inside_keys             = new_inside_keys;
  s1->center                  = pca_res[0];
  s1->bounding_sphere_radius  = new_bounding_sphere_radius;
  s1->has_merged_atleast_once = true;

  spheremap_server::calculateBlockParamsForSegment(s1, border_points_relative);

  updateConnectionsForSegment(id1);
  updateNavCenterForSegment(id1);

  setPortalsOfSegmentMergeReadiness(id1, true);
  /* ROS_INFO("[SegmapUpdate]: merge done"); */

  return true;
}
//}

/* SegmentOcTree::isSegmentInsideBBXOfSegment() //{ */
bool SegmentOcTree::isSegmentInsideBBXOfSegment(Segment* larger_segment, Segment* smaller_segment) {
  /* ALSO CHECK IF THE SMALLER SEGMENT IS COMPLETELY IN BBX OF LARGER AND IF SO, MERGE ANYWAY */
  /* float minproj                = 0.6; */
  float mindist_in_bbx = 0;
  float search_depth   = 16 - depth_offset;
  for (uint i = 0; i < smaller_segment->border_keys.size(); i++) {
    octomap::point3d deltavec = (keyToCoord(smaller_segment->border_keys[i], search_depth) - larger_segment->center);
    float            aproj    = abs(deltavec.dot(larger_segment->block_dirs[0]));
    float            bproj    = abs(deltavec.dot(larger_segment->block_dirs[1]));
    float            cproj    = abs(deltavec.dot(larger_segment->block_dirs[2]));
    if (larger_segment->block_a - aproj < mindist_in_bbx || larger_segment->block_b - bproj < mindist_in_bbx ||
        larger_segment->block_c - cproj < mindist_in_bbx) {
      return false;
    }
  }
  return true;
}
//}

/* SegmentOcTree::recalculateDistsFromHome() //{ */
void SegmentOcTree::recalculateDistsFromHome() {
  if (segments.empty()) {
    return;
  }
  for (std::vector<Segment>::iterator it = segments.begin(); it != segments.end(); it++) {
    it->is_connected_to_home = false;
  }

  int              start_seg_id = segments[0].id;
  SegmentAstarNode start_node(start_seg_id, NULL, 0, 0, segments[0].center, 0);
  /* PERFORM DIJKSTRA */
  std::list<SegmentAstarNode>   explored         = {};
  std::list<int>                explored_seg_ids = {start_seg_id};
  std::vector<SegmentAstarNode> frontier         = {start_node};

  int n_dist_set = 0;
  while (frontier.size() > 0) {
    /* SORT THE FRONTIER LIST AND SELECT LOWEST COST NODE */
    std::sort(frontier.begin(), frontier.end());
    SegmentAstarNode expanded = *frontier.begin();

    /* ERASE NODE FROM FRONTIER, ADD IT TO EXPANDED, KEEP POINTER TO IT IN EXPANDED */
    frontier.erase(frontier.begin());
    explored.push_back(expanded);
    /* SegmentAstarNode* parent_ptr = &(*(--explored.end())); */

    int expanded_seg_index = getSegmentIndex(expanded.seg_id);
    /* SET DIST FROM HOME */
    segments[expanded_seg_index].distance_from_home   = expanded.total_g_cost;
    segments[expanded_seg_index].is_connected_to_home = true;
    n_dist_set++;
    /* GET ADJACENT NODES */
    for (uint i = 0; i < segments[expanded_seg_index].connected_segments_ids.size(); i++) {
      int connected_seg_id = segments[expanded_seg_index].connected_segments_ids[i];
      /* IF ADJACENT SEGMENT ALREADY IN FRONTIER OR EXPLORED, IGNORE IT */
      if (std::find(explored_seg_ids.begin(), explored_seg_ids.end(), connected_seg_id) != explored_seg_ids.end()) {
        continue;
      }

      /* CREATE NEW ASTAR NODE AND STORE TO FRONTIER*/
      SegmentPortal* portal_ptr = getPortalPtr(expanded.seg_id, connected_seg_id);
      float          g_cost     = (expanded.pos - portal_ptr->position).norm();

      SegmentAstarNode new_node(connected_seg_id, NULL, expanded.total_g_cost + g_cost, 0, portal_ptr->position, expanded.total_g_cost + g_cost);
      frontier.push_back(new_node);
      /* ADD NEW FRONTIER SEGMENT ID TO EXPLORED SEGMENT ID LIST */
      explored_seg_ids.push_back(connected_seg_id);
    }
  }

  ROS_INFO("[SegmentHomeDistCalc]: %d/%lu segments are connected to home", n_dist_set, segments.size());
}
//}

/* void SegmentOcTree::spreadFrontierValue() //{ */
void SegmentOcTree::spreadFrontierValue(octomap::point3d frontier_pos, int start_seg_id, float max_spread_dist) {
  if (segments.empty()) {
    return;
  }

  int start_seg_index = getSegmentIndex(start_seg_id);
  if (start_seg_index < 0) {
    return;
  }

  SegmentAstarNode start_node(start_seg_id, NULL, 0, 0, frontier_pos, 0);
  /* PERFORM SPREAD */
  std::list<SegmentAstarNode>   explored         = {};
  std::list<int>                explored_seg_ids = {start_seg_id};
  std::vector<SegmentAstarNode> frontier         = {start_node};

  while (frontier.size() > 0) {
    /* SORT THE FRONTIER LIST AND SELECT LOWEST COST NODE */
    std::sort(frontier.begin(), frontier.end());
    SegmentAstarNode expanded = *frontier.begin();

    /* ERASE NODE FROM FRONTIER, ADD IT TO EXPANDED, KEEP POINTER TO IT IN EXPANDED */
    frontier.erase(frontier.begin());
    explored.push_back(expanded);
    int expanded_seg_index = getSegmentIndex(expanded.seg_id);

    /* SET FRONTIER VAL */
    float possible_frontier_value = 1 - pow(expanded.total_g_cost / max_spread_dist, 2);
    if (possible_frontier_value > segments[expanded_seg_index].frontier_value) {
      segments[expanded_seg_index].frontier_value = possible_frontier_value;
    }
    /* GET ADJACENT NODES */
    for (uint i = 0; i < segments[expanded_seg_index].connected_segments_ids.size(); i++) {
      int connected_seg_id = segments[expanded_seg_index].connected_segments_ids[i];
      /* IF ADJACENT SEGMENT ALREADY IN FRONTIER OR EXPLORED, IGNORE IT */
      if (std::find(explored_seg_ids.begin(), explored_seg_ids.end(), connected_seg_id) != explored_seg_ids.end()) {
        continue;
      }

      /* CREATE NEW ASTAR NODE AND STORE TO FRONTIER*/
      SegmentPortal* portal_ptr = getPortalPtr(expanded.seg_id, connected_seg_id);
      float          g_cost     = (expanded.pos - portal_ptr->position).norm();

      /* CUTOFF IF TOO FAR */
      if (expanded.total_g_cost + g_cost > max_spread_dist) {
        continue;
      }

      SegmentAstarNode new_node(connected_seg_id, NULL, expanded.total_g_cost + g_cost, 0, portal_ptr->position, expanded.total_g_cost + g_cost);
      frontier.push_back(new_node);
      /* ADD NEW FRONTIER SEGMENT ID TO EXPLORED SEGMENT ID LIST */
      explored_seg_ids.push_back(connected_seg_id);
    }
  }
}
//}

struct NarrowPassageCheckVoxel
{
  int   origin_seg_id;
  int   voxeldist_from_segment;
  float safedist;
  NarrowPassageCheckVoxel(int seg, int voxeldist, float safed) {
    origin_seg_id          = seg;
    voxeldist_from_segment = voxeldist;
    safedist               = safed;
  }
};

struct ByKey : public std::binary_function<octomap::OcTreeKey, octomap::OcTreeKey, bool>
{
  bool operator()(const octomap::OcTreeKey& lhs, const octomap::OcTreeKey& rhs) const {
    return lhs[0] < rhs[0] || lhs[1] < rhs[1] || lhs[2] < rhs[2];
  }
};


//}

}  // namespace octomap
