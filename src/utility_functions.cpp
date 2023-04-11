#include <search_planning/utility_functions.h>
/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

namespace search_planning
{

/* rotationMatrixToQuaternion() //{ */
tf2::Quaternion rotationMatrixToQuaternion(std::vector<octomap::point3d> v) {

  return tf2::Quaternion();  // FIXME

  /*   tf2::Matrix3x3 rot; */

  /*   mrs_lib::vec3_t v1(1, 0, 0); */
  /*   mrs_lib::vec3_t v2(v[0].x(), v[0].y(), v[0].z()); */
  /*   auto            qq = mrs_lib::quaternion_between(v1, v2); */
  /*   tf2::Quaternion q1; */
  /*   q1.setX(qq.x()); */
  /*   q1.setY(qq.y()); */
  /*   q1.setZ(qq.z()); */
  /*   q1.setW(qq.w()); */


  /*   mrs_lib::vec3_t v3(0, 1, 0); */
  /*   mrs_lib::vec3_t v4(v[1].x(), v[1].y(), v[1].z()); */
  /*   v3 = qq * v3; */
  /*   qq = mrs_lib::quaternion_between(v3, v4); */

  /*   tf2::Quaternion q2; */
  /*   q2.setX(qq.x()); */
  /*   q2.setY(qq.y()); */
  /*   q2.setZ(qq.z()); */
  /*   q2.setW(qq.w()); */
  /*   return q2 * q1; */


  /* for (int col = 0; col < 3; col++) { */
  /*   rot[col][0] = v[col].x(); */
  /*   rot[col][1] = v[col].y(); */
  /*   rot[col][2] = v[col].z(); */
  /* } */
  /* float d = rot.determinant(); */
  /* ROS_INFO("DETER %f", d); */
  /* if (d < 0) { */
  /*   rot[0][0] = -v[0].x(); */
  /*   rot[0][1] = -v[0].y(); */
  /*   rot[0][2] = -v[0].z(); */
  /*   d         = rot.determinant(); */
  /*   ROS_INFO("CHANGED DETER %f", d); */
  /* } */
  /* tf2::Quaternion q; */
  /* rot.getRotation(q); */
  /* return q; */
}
//}

/* castRayWithNullNodes() //{ */
octomap::point3d castRayWithNullNodes(std::shared_ptr<octomap::OcTree> occupancy_octree_, octomap::point3d point, octomap::point3d end_point, bool* hit) {
  octomap::KeyRay ray;
  occupancy_octree_->computeRayKeys(point, end_point, ray);
  bool               found_free_point = false;
  octomap::OcTreeKey last_free_key;

  for (octomap::KeyRay::iterator it = ray.begin(), end = ray.end(); it != end; ++it) {
    octomap::OcTreeNode* s_node = occupancy_octree_->search(*it);
    if (s_node != NULL && !occupancy_octree_->isNodeOccupied(s_node)) {
      found_free_point = true;
      last_free_key    = (*it);
    } else {
      break;
    }
  }
  if (found_free_point) {
    *hit = true;
    return (occupancy_octree_->keyToCoord(last_free_key));
  }
  *hit = false;
  return point;
}
//}

/* isPointSafe() //{ */
bool isPointSafe(octomap::point3d center_point, float safe_dist, std::shared_ptr<octomap::OcTree> occupancy_octree_) {
  // this manner of choosing the bounding box is not the most accurate, but it works
  int                cube_voxel_length = (safe_dist / occupancy_octree_->getResolution()) + 1;
  float              safe_dist2        = safe_dist * safe_dist;
  octomap::OcTreeKey iterator_key;

  octomap::OcTreeKey start_key = occupancy_octree_->coordToKey(center_point - octomap::point3d(1, 1, 1) * (safe_dist / 2));
  for (int xx = start_key[0]; xx < start_key[0] + cube_voxel_length; xx++) {
    for (int yy = start_key[1]; yy < start_key[1] + cube_voxel_length; yy++) {
      for (int zz = start_key[2]; zz < start_key[2] + cube_voxel_length; zz++) {
        iterator_key[0]              = xx;
        iterator_key[1]              = yy;
        iterator_key[2]              = zz;
        octomap::OcTreeNode* oc_node = occupancy_octree_->search(iterator_key);
        if (oc_node == NULL || occupancy_octree_->isNodeOccupied(oc_node)) {
          octomath::Vector3 delta_vector = (occupancy_octree_->keyToCoord(iterator_key) - center_point);
          float             dist2        = delta_vector.dot(delta_vector);
          if (dist2 < safe_dist2) {
            /* ROS_INFO("waypoint is not safe, found invalid at distance %f", sqrt(dist2)); */
            return false;
          }
        }
      }
    }
  }
  return true;
}
//}

/* makePointSafe() //{ */
bool makePointSafe(octomap::point3d* point_ptr, float safe_dist, int max_iters, std::shared_ptr<octomap::OcTree> occupancy_octree_,
                   std::shared_ptr<octomap::SurfaceOcTree> surface_octree_) {
  std::vector<octomap::point3d> occupied_points   = {};
  float                         safety_modifier   = 5;
  int                           cube_voxel_length = (safety_modifier * safe_dist / surface_octree_->getResolution()) + 1;
  float                         safe_dist2        = safe_dist * safe_dist;
  octomap::OcTreeKey            iterator_key;

  octomap::point3d   center_point = *point_ptr;
  octomap::OcTreeKey start_key    = surface_octree_->coordToKey(center_point - octomap::point3d(1, 1, 1) * (safety_modifier * safe_dist / 2));

  for (int xx = start_key[0]; xx < start_key[0] + cube_voxel_length; xx++) {
    for (int yy = start_key[1]; yy < start_key[1] + cube_voxel_length; yy++) {
      for (int zz = start_key[2]; zz < start_key[2] + cube_voxel_length; zz++) {
        iterator_key[0]              = xx;
        iterator_key[1]              = yy;
        iterator_key[2]              = zz;
        octomap::OcTreeNode* oc_node = occupancy_octree_->search(iterator_key);
        if (oc_node == NULL || occupancy_octree_->isNodeOccupied(oc_node)) {
          occupied_points.push_back(occupancy_octree_->keyToCoord(iterator_key));
        }
      }
    }
  }

  // how much will the point be moved at distance 1m and 1/10 at distance 0.1m
  float            move_strength          = 0.01;
  octomap::point3d center_point_start_pos = center_point;
  for (int i = 0; i < max_iters; i++) {
    octomap::point3d move_vector(0, 0, 0);
    int              num_near = 0;
    for (octomap::point3d point : occupied_points) {
      octomath::Vector3 delta_vector = (center_point - point);
      float             dist2        = delta_vector.dot(delta_vector);
      // capping the dist2 at 0.01m
      if (dist2 < 0.01) {
        dist2 = 0.01;
      }
      if (dist2 < safe_dist2) {
        num_near++;
        move_vector += delta_vector * (move_strength / dist2);
      }
    }
    if (num_near == 0) {
      /* ROS_INFO("moved point to safe spot, stopping iterations"); */
      *point_ptr = center_point;
      return true;
    }
    /* ROS_INFO("moving by %f m, near nodes: %d", move_vector.norm(), num_near); */
    /* ROS_INFO("total moved: %f", (center_point - center_point_start_pos).norm()); */
    center_point += move_vector;
  }

  *point_ptr = center_point;
  return false;
}
//}

/* getRandomFOVRayEndpoint() //{ */
tf2::Vector3 getRandomFOVRayEndpoint(octomap::point3d top_left_corner, bool is_optic_frame) {
  if (is_optic_frame) {
    float z = top_left_corner.z();
    float y = top_left_corner.y() * 2 * (double)rand() / (double)RAND_MAX - top_left_corner.y();
    float x = top_left_corner.x() * 2 * (double)rand() / (double)RAND_MAX - top_left_corner.x();
    return tf2::Vector3(x, y, z);
  } else {
    float x = top_left_corner.x();
    float y = top_left_corner.y() * 2 * (double)rand() / (double)RAND_MAX - top_left_corner.y();
    float z = top_left_corner.z() * 2 * (double)rand() / (double)RAND_MAX - top_left_corner.z();
    return tf2::Vector3(x, y, z);
  }
};
//}

/* float getExplorednessRelativeToCamera() //{ */
float getExplorednessRelativeToCamera(float dist) {
  /* return dist < camera_FOV_top_left_corner.x() ? ((camera_FOV_top_left_corner.x() - dist) / camera_FOV_top_left_corner.x()) : 0; */
  return 1;
  // FIXME
}

//}

/* getHeadingFromVector() //{ */
float getHeadingFromVector(octomap::point3d normal) {
  return std::atan2(normal.y(), normal.x());
}
//}

/* hasNodeUnknownChildren() //{ */
bool hasNodeUnknownChildren(octomap::OcTreeNode* node, std::shared_ptr<octomap::OcTree> octree, uint depth) {
  if (octree->nodeHasChildren(node)) {
    for (int i = 0; i < 8; i++) {
      if (octree->nodeChildExists(node, i)) {
        if (hasNodeUnknownChildren(octree->getNodeChild(node, i), octree, depth + 1)) {
          return true;
        }
      } else {
        return true;
      }
    }
    return false;
  } else if (depth >= octree->getTreeDepth()) {
    return false;
  }
  return false;
}
//}

/* heatMapColor() //{ */
std_msgs::ColorRGBA heatMapColor(double h, double a) {
  std_msgs::ColorRGBA color;
  /* if (h > 0.99) { */
  /*   color.r = 1; */
  /*   color.g = 1; */
  /*   color.b = 0; */
  /*   color.a = a; */
  /*   return color; */
  /* } */
  /* h = 1 - h; */


  color.a = a;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);  // h would be from zero to one here
  h *= 6;
  int    i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v;
      color.g = n;
      color.b = m;
      break;
    case 1:
      color.r = n;
      color.g = v;
      color.b = m;
      break;
    case 2:
      color.r = m;
      color.g = v;
      color.b = n;
      break;
    case 3:
      color.r = m;
      color.g = n;
      color.b = v;
      break;
    case 4:
      color.r = n;
      color.g = m;
      color.b = v;
      break;
    case 5:
      color.r = v;
      color.g = m;
      color.b = n;
      break;
    default:
      color.r = 1;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }

  return color;
}
//}

/* octomap::point3d getRandomPointInSphere() //{ */
octomap::point3d getRandomPointInSphere(float r) {
  float x, y, z;
  while (1) {
    x = 2 * (double)rand() / (double)RAND_MAX - 1;
    y = 2 * (double)rand() / (double)RAND_MAX - 1;
    z = 2 * (double)rand() / (double)RAND_MAX - 1;
    if (pow(x, 2) + pow(y, 2) + pow(z, 2) < 1) {
      break;
    }
  }
  return octomap::point3d(x * r, y * r, z * r);
}

octomap::point3d getRandomPointInSphere(float min_r, float delta_r) {
  float x, y, z;
  while (1) {
    x = 2 * (double)rand() / (double)RAND_MAX - 1;
    y = 2 * (double)rand() / (double)RAND_MAX - 1;
    z = 2 * (double)rand() / (double)RAND_MAX - 1;
    if (pow(x, 2) + pow(y, 2) + pow(z, 2) < 1) {
      break;
    }
  }

  float rad = min_r + delta_r * ((double)rand() / (double)RAND_MAX - 1);
  return (octomap::point3d(x, y, z).normalized()) * rad;
}
//}

/* getPointsBoundingBox() //{ */
BoundingBox getPointsBoundingBox(std::vector<octomap::point3d> points) {
  if (points.size() == 0) {
    return BoundingBox();
  }
  float x1, x2, y1, y2, z1, z2;
  x1 = points[0].x();
  x2 = x1;
  y1 = points[0].y();
  y2 = y1;
  z1 = points[0].z();
  z2 = z1;
  for (uint i = 1; i < points.size(); i++) {
    float x = points[i].x();
    float y = points[i].y();
    float z = points[i].z();
    if (x < x1) {
      x1 = x;
    } else if (x > x2) {
      x2 = x;
    }

    if (y < y1) {
      y1 = y;
    } else if (y > y2) {
      y2 = y;
    }

    if (z < z1) {
      z1 = z;
    } else if (z > z2) {
      z2 = z;
    }
  }

  return BoundingBox(x1, x2, y1, y2, z1, z2);
}
//}

/* getSegmentColor() //{ */
octomap::point3d getSegmentColor(int segment_id) {
  int num_rgbs = 10;
  /* int   num_darkness_levels = 2; */
  float rgbs[][10] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 0}, {0, 1, 1}, {1, 0, 1}, {1, 0.5, 0}, {1, 0, 0.5}, {0, 1, 0.5}, {0, 0.5, 1}};
  /* float lightness           = (1 + (segment_id / (num_rgbs * num_darkness_levels)) % num_darkness_levels) / ((float) num_darkness_levels); */
  float lightness = 1;
  float r         = rgbs[segment_id % num_rgbs][0];
  float g         = rgbs[segment_id % num_rgbs][1];
  float b         = rgbs[segment_id % num_rgbs][2];
  return octomap::point3d(r, g, b) * lightness;
}
//}

/* getSegmentPathEulerDistance() //{ */
float getSegmentPathEulerDistance(octomap::SegmentPath path) {
  if (path.positions.size() == 0) {
    return 0;
  }
  octomap::point3d current_position = path.positions[0];
  float            total_dist       = 0;
  for (uint i = 1; i < path.positions.size(); i++) {
    total_dist += (path.positions[i] - current_position).norm();
    current_position = path.positions[i];
  }
  return total_dist;
}
//}

/* octomapToPointcloud() //{ */
std::vector<pcl::PointXYZ> octomapToPointcloud(std::shared_ptr<octomap::OcTree> occupancy_octree_, BoundingBox bbx) {
  octomap::OcTreeKey         start_key = occupancy_octree_->coordToKey(bbx.x1, bbx.y1, bbx.z1, 16);
  octomap::OcTreeKey         end_key   = occupancy_octree_->coordToKey(bbx.x2, bbx.y2, bbx.z2, 16);
  std::vector<pcl::PointXYZ> output_pcl;
  for (octomap::OcTree::leaf_bbx_iterator it = occupancy_octree_->begin_leafs_bbx(start_key, end_key), end = occupancy_octree_->end_leafs_bbx(); it != end;
       ++it) {
    pcl::PointXYZ        point;
    octomap::OcTreeKey   tmp_key;
    octomap::OcTreeNode* node_ptr = &(*it);
    if (occupancy_octree_->isNodeOccupied(node_ptr)) {
      point.x = it.getX();
      point.y = it.getY();
      point.z = it.getZ();
      output_pcl.push_back(point);
    }
    /* //ROS_INFO("[%s]: x = %d, y = %d, z = %d", ros::this_node::getName().c_str(), x, y, z); */
  }  // namespace search_planning
  return output_pcl;
}
//}

/* octomapToPointcloud() //{ */
std::vector<pcl::PointXYZ> octomapToPointcloud(std::shared_ptr<octomap::OcTree> occupancy_octree_) {
  std::vector<pcl::PointXYZ> output_pcl;
  for (octomap::OcTree::iterator it = occupancy_octree_->begin(), end = occupancy_octree_->end(); it != end; ++it) {
    if (it.getDepth() != occupancy_octree_->getTreeDepth())
      continue;

    if (occupancy_octree_->search(it.getKey()) == NULL ||
        (occupancy_octree_->search(it.getKey()) != NULL && occupancy_octree_->isNodeOccupied(occupancy_octree_->search(it.getKey())))) {
      pcl::PointXYZ    point;
      octomap::point3d octomap_point = occupancy_octree_->keyToCoord(it.getKey());
      point.x                        = octomap_point.x();
      point.y                        = octomap_point.y();
      point.z                        = octomap_point.z();
      output_pcl.push_back(point);
    }
  }
  return output_pcl;
}
//}

/* calculatePointFrontierExplorationValue() //{ */
float calculatePointFrontierExplorationValue(octomap::point3d point, int num_raycasts, std::shared_ptr<octomap::OcTree> occupancy_octree_) {
  /* float                           normal_dot_product_threshold = 0.15; */
  /* float                           ceiling_node_z_threshold     = -0.2; */
  float max_dist           = 10;
  float max_vertical_angle = M_PI / 8;
  /* float                           total_infomation_value       = 0; */
  std::vector<octomap::OcTreeKey> unique_keys = {};

  std::vector<octomap::point3d> dirvecs                   = {};
  std::vector<octomap::point3d> hits                      = {};
  int                           num_unexplored_points_hit = 0;

  /* CAST RAYS IN ALL DIRECTIONS UNDER SOME MAX ANGLE */
  for (int i = 0; i < num_raycasts; i++) {
    float            phi       = (double(rand()) / RAND_MAX) * 2 * M_PI;
    float            theta     = 2 * (double(rand()) / RAND_MAX) * max_vertical_angle - max_vertical_angle;
    float            cs        = cos(phi);
    float            sn        = sin(phi);
    float            cos_theta = cos(theta);
    float            sin_theta = sin(theta);
    float            rotated_x = cs * cos_theta;
    float            rotated_y = sn * cos_theta;
    float            rotated_z = sin_theta;
    octomap::point3d dir_vec_rotated(rotated_x, rotated_y, rotated_z);

    octomap::point3d hit_point;
    bool             ray_res = occupancy_octree_->castRay(point, dir_vec_rotated, hit_point, false, max_dist);
    if (!ray_res) {
      /* ROS_INFO("ray dir: %f,%f,%f", dir_vec_rotated.x(), dir_vec_rotated.y(), dir_vec_rotated.z()); */
      // FIXME crude but it's quicker
      /* hit_point = vp.position + dir_vec_rotated; */
      /* hit_point = start_point + (end_point - start_point) * (max_dist / (end_point - start_point).norm()); */
      /* ROS_INFO("ray did not hit: %f, %f, %f", hit_point.x(), hit_point.y(), hit_point.z()); */
      if (occupancy_octree_->search(hit_point) == NULL) {
        num_unexplored_points_hit++;
      }
      continue;
    }
    /* ROS_INFO("ray hit: %f, %f, %f", hit_point.x(), hit_point.y(), hit_point.z()); */

    hits.push_back(hit_point);
    dirvecs.push_back(dir_vec_rotated.normalized());

    // Then check all surface nodes hit by the ray (approximately) and update their exploredness value
    /* octomap::KeyRay ray; */
    /* surface_octree_->computeRayKeys(vp.position, hit_point, ray); */
  }

  /* RETURN PERCENTAGE OF UNKNOWN NODES HIT  0 - 100*/
  return ((double)num_unexplored_points_hit / num_raycasts) * 100;
}
//}

/* publishPointsMarkers() //{ */
void publishPointsMarkers(std::vector<octomap::point3d> points, std::vector<int> colors, std::string markers_name, float size,
                          ros::Publisher* pub_octomap_markers_, std::string map_frame_) {
  int num_points = points.size();

  if (num_points < 1) {
    /* ROS_WARN("points list for publishing markers is empty"); */
    return;
  }
  // init markers:
  visualization_msgs::MarkerArray::Ptr occupiedNodesVis = boost::make_shared<visualization_msgs::MarkerArray>();
  // each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis->markers.resize(num_points);
  unsigned int idx              = 0;
  float        cube_side_length = size;

  // now, traverse all leafs in the tree:
  for (int i = 0; i < num_points; i++) {
    double x = points[i].x();
    double y = points[i].y();
    double z = points[i].z();

    // create marker:
    assert(idx < occupiedNodesVis->markers.size());

    geometry_msgs::Point cubeCenter;
    cubeCenter.x = x;
    cubeCenter.y = y;
    cubeCenter.z = z;

    occupiedNodesVis->markers[idx].pose.position = cubeCenter;
    occupiedNodesVis->markers[idx].color.a       = 1;
    if (colors[i] == 0) {
      occupiedNodesVis->markers[idx].color.r = 1;
      occupiedNodesVis->markers[idx].color.g = 0;
      occupiedNodesVis->markers[idx].color.b = 0;
    } else {
      occupiedNodesVis->markers[idx].color.r = 0;
      occupiedNodesVis->markers[idx].color.g = 1;
      occupiedNodesVis->markers[idx].color.b = 0;
    }

    occupiedNodesVis->markers[idx].header.frame_id = map_frame_;
    occupiedNodesVis->markers[idx].header.stamp    = ros::Time::now();
    occupiedNodesVis->markers[idx].ns              = markers_name;
    occupiedNodesVis->markers[idx].id              = idx;
    occupiedNodesVis->markers[idx].type            = visualization_msgs::Marker::SPHERE;
    occupiedNodesVis->markers[idx].scale.x         = cube_side_length;
    occupiedNodesVis->markers[idx].scale.y         = cube_side_length;
    occupiedNodesVis->markers[idx].scale.z         = cube_side_length;
    occupiedNodesVis->markers[idx].lifetime        = ros::Duration(2);
    occupiedNodesVis->markers[idx].action          = visualization_msgs::Marker::ADD;

    idx++;
  }
  pub_octomap_markers_->publish(occupiedNodesVis);
}
//}

/* publishArrowMarkers() //{ */
void publishArrowMarkers(std::vector<octomap::point3d> points, std::vector<octomap::point3d> points2, std::vector<int> colors, std::string markers_name,
                         ros::Publisher* pub_octomap_markers_, std::string map_frame_) {
  int num_points = points.size();

  if (num_points < 1) {
    /* ROS_WARN("points list for publishing markers is empty"); */
    return;
  }

  // init markers:
  visualization_msgs::MarkerArray::Ptr occupiedNodesVis = boost::make_shared<visualization_msgs::MarkerArray>();
  // each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis->markers.resize(num_points);
  unsigned int idx = 0;

  // now, traverse all leafs in the tree:
  for (int i = 0; i < num_points; i++) {
    // create marker:
    assert(idx < occupiedNodesVis->markers.size());

    geometry_msgs::Point start_point;
    start_point.x = points[i].x();
    start_point.y = points[i].y();
    start_point.z = points[i].z();

    geometry_msgs::Point end_point;
    end_point.x = points2[i].x();
    end_point.y = points2[i].y();
    end_point.z = points2[i].z();

    occupiedNodesVis->markers[idx].points.push_back(start_point);
    occupiedNodesVis->markers[idx].points.push_back(end_point);

    /* occupiedNodesVis->markers[idx].pose.position = cubeCenter; */
    occupiedNodesVis->markers[idx].color.a = 1;
    if (colors[i] == 0) {
      occupiedNodesVis->markers[idx].color.r = 1;
      occupiedNodesVis->markers[idx].color.g = 0;
      occupiedNodesVis->markers[idx].color.b = 0;
    } else {
      occupiedNodesVis->markers[idx].color.r = 0;
      occupiedNodesVis->markers[idx].color.g = 1;
      occupiedNodesVis->markers[idx].color.b = 0;
    }

    occupiedNodesVis->markers[idx].header.frame_id = map_frame_;
    occupiedNodesVis->markers[idx].header.stamp    = ros::Time::now();
    occupiedNodesVis->markers[idx].ns              = markers_name;
    occupiedNodesVis->markers[idx].id              = idx;
    occupiedNodesVis->markers[idx].type            = visualization_msgs::Marker::ARROW;
    occupiedNodesVis->markers[idx].scale.x         = 0.2;
    occupiedNodesVis->markers[idx].scale.y         = 0.3;
    occupiedNodesVis->markers[idx].scale.z         = 0.1;
    occupiedNodesVis->markers[idx].lifetime        = ros::Duration(2);
    occupiedNodesVis->markers[idx].action          = visualization_msgs::Marker::ADD;

    idx++;
  }
  pub_octomap_markers_->publish(occupiedNodesVis);
}

//}

/* publishMarkers() //{ */
void publishMarkers(std::vector<visualization_msgs::Marker> markers, ros::Publisher* publisher, std::string markers_name, std::string map_frame, int start_id,
                    float lifetime) {
  int num_points = markers.size();

  if (num_points < 1) {
    /* ROS_WARN("points list for publishing markers is empty"); */
    return;
  }
  // init markers:
  visualization_msgs::MarkerArray::Ptr m_array = boost::make_shared<visualization_msgs::MarkerArray>();
  // each array stores all cubes of a different size, one for each depth level:
  m_array->markers.resize(num_points);
  // now, traverse all leafs in the tree:
  ros::Time time_now = ros::Time::now();
  for (int i = 0; i < num_points; i++) {
    m_array->markers[i]                 = markers[i];
    m_array->markers[i].header.frame_id = map_frame;
    m_array->markers[i].header.stamp    = time_now;
    m_array->markers[i].ns              = markers_name;
    m_array->markers[i].id              = start_id + i;
    if (lifetime > 0) {
      m_array->markers[i].lifetime = ros::Duration(lifetime);
    }
    m_array->markers[i].action = visualization_msgs::Marker::ADD;
  }
  publisher->publish(m_array);
}
//}

/* publishMarkers() //{ */
void publishMarkers(std::shared_ptr<std::vector<visualization_msgs::Marker>> markers, ros::Publisher* publisher, std::string markers_name,
                    std::string map_frame, int start_id, float lifetime) {
  int num_points = markers->size();

  if (num_points < 1) {
    /* ROS_WARN("points list for publishing markers is empty"); */
    return;
  }
  // init markers:
  visualization_msgs::MarkerArray::Ptr m_array = boost::make_shared<visualization_msgs::MarkerArray>();
  // each array stores all cubes of a different size, one for each depth level:
  m_array->markers.resize(num_points);
  // now, traverse all leafs in the tree:
  ros::Time time_now = ros::Time::now();
  for (int i = 0; i < num_points; i++) {
    m_array->markers[i]                 = (*markers)[i];
    m_array->markers[i].header.frame_id = map_frame;
    m_array->markers[i].header.stamp    = time_now;
    m_array->markers[i].ns              = markers_name;
    m_array->markers[i].id              = start_id + i;
    if (lifetime > 0) {
      m_array->markers[i].lifetime = ros::Duration(lifetime);
    }
    m_array->markers[i].action = visualization_msgs::Marker::ADD;
  }
  publisher->publish(m_array);
}
//}

/* MARKER CREATING //{ */
visualization_msgs::Marker getMarkerArrow(octomap::point3d pos, octomap::point3d dir, octomap::point3d color, float shaft_width, float head_width,
                                          float head_length) {
  visualization_msgs::Marker res;
  geometry_msgs::Point       pos2;
  pos2.x = pos.x();
  pos2.y = pos.y();
  pos2.z = pos.z();

  geometry_msgs::Point pos3;
  pos3.x = pos.x() + dir.x();
  pos3.y = pos.y() + dir.y();
  pos3.z = pos.z() + dir.z();

  res.color.a = 1;
  res.color.r = color.x();
  res.color.g = color.y();
  res.color.b = color.z();

  res.points.push_back(pos2);
  res.points.push_back(pos3);
  res.type    = visualization_msgs::Marker::ARROW;
  res.scale.x = shaft_width;
  res.scale.y = head_width;
  res.scale.z = head_length;

  return res;
}
visualization_msgs::Marker getMarkerSphere(octomap::point3d pos, float d, octomap::point3d color, float alpha) {
  visualization_msgs::Marker res;
  geometry_msgs::Point       pos2;
  pos2.x = pos.x();
  pos2.y = pos.y();
  pos2.z = pos.z();

  res.pose.position = pos2;

  res.color.a = alpha;
  res.color.r = color.x();
  res.color.g = color.y();
  res.color.b = color.z();

  res.type    = visualization_msgs::Marker::SPHERE;
  res.scale.x = d;
  res.scale.y = d;
  res.scale.z = d;
  return res;
}
visualization_msgs::Marker getMarkerCube(octomap::point3d pos, float d, octomap::point3d color) {
  visualization_msgs::Marker res;
  geometry_msgs::Point       pos2;
  pos2.x = pos.x();
  pos2.y = pos.y();
  pos2.z = pos.z();

  res.pose.position = pos2;

  res.color.a = 1;
  res.color.r = color.x();
  res.color.g = color.y();
  res.color.b = color.z();

  res.type    = visualization_msgs::Marker::CUBE;
  res.scale.x = d;
  res.scale.y = d;
  res.scale.z = d;
  return res;
}
visualization_msgs::Marker getMarkerLine(octomap::point3d pos, octomap::point3d endpos, octomap::point3d color, float linewidth) {
  visualization_msgs::Marker res;
  geometry_msgs::Point       pos2;
  pos2.x = pos.x();
  pos2.y = pos.y();
  pos2.z = pos.z();

  geometry_msgs::Point pos3;
  pos3.x = endpos.x();
  pos3.y = endpos.y();
  pos3.z = endpos.z();

  res.color.a = 1;
  res.color.r = color.x();
  res.color.g = color.y();
  res.color.b = color.z();

  res.points.push_back(pos2);
  res.points.push_back(pos3);
  res.type    = visualization_msgs::Marker::LINE_LIST;
  res.scale.x = linewidth;
  res.scale.y = linewidth;
  res.scale.z = linewidth;

  return res;
}

visualization_msgs::Marker getMarkerBlock(octomap::point3d pos, float alpha, float beta, float a, float b, float c, octomap::point3d color, float clr_alpha) {
  visualization_msgs::Marker res;
  geometry_msgs::Point       pos2;
  pos2.x = pos.x();
  pos2.y = pos.y();
  pos2.z = pos.z();

  res.pose.position = pos2;

  res.color.a = clr_alpha;
  res.color.r = color.x();
  res.color.g = color.y();
  res.color.b = color.z();

  res.type    = visualization_msgs::Marker::CUBE;
  res.scale.x = a * 2;
  res.scale.y = b * 2;
  res.scale.z = c * 2;

  float           ca = cos(alpha);
  float           sa = sin(alpha);
  float           cb = cos(beta);
  float           sb = sin(beta);
  tf2::Quaternion quat;
  tf2::Matrix3x3  matrix(ca * cb, sa, -sb * ca, -cb * sa, ca, sa * sb, sb, 0, cb);
  matrix.getRotation(quat);
  res.pose.orientation.x = quat.x();
  res.pose.orientation.y = quat.y();
  res.pose.orientation.z = quat.z();
  res.pose.orientation.w = quat.w();
  return res;
}

/* visualization_msgs::Marker getMarkerText(octomap::point3d pos1, std::string text, octomap::point3d color = octomap::point3d(0, 1, 0), float size = 5) { */
/*   visualization_msgs::Marker res; */
/*   geometry_msgs::Point       pos2; */
/*   pos2.x = pos.x(); */
/*   pos2.y = pos.y(); */
/*   pos2.z = pos.z(); */

/*   res.pose.position = pos2; */

/*   res.color.a = 1; */
/*   res.color.r = color.x(); */
/*   res.color.g = color.y(); */
/*   res.color.b = color.z(); */

/*   res.type    = visualization_msgs::Marker::TEXT_VIEW_FACING; */
/*   res.scale.z = size; */

/*   res.text = text; */
/*   return res; */
/* } */
//}

/* SEGMAP MARKERS */

/* getSegmentOctreeMarkers()  //{ */
std::shared_ptr<std::vector<visualization_msgs::Marker>> getSegmentOctreeMarkers(std::shared_ptr<octomap::SegmentOcTree> segment_octree_) {
  std::vector<visualization_msgs::Marker>*                 ptr     = new std::vector<visualization_msgs::Marker>;
  std::shared_ptr<std::vector<visualization_msgs::Marker>> res_ptr = std::shared_ptr<std::vector<visualization_msgs::Marker>>(ptr);

  size_t octomapSize = segment_octree_->size();
  if (octomapSize <= 1) {
    return NULL;
  }

  float voxel_size = 0.2 * pow(2.0, segment_octree_->depth_offset);

  for (uint i = 0; i < segment_octree_->segments.size(); i++) {
    visualization_msgs::Marker marker;
    res_ptr->push_back(marker);

    octomap::point3d color = getSegmentColor(segment_octree_->segments[i].id);
    (*res_ptr)[i].color.r  = color.x();
    (*res_ptr)[i].color.g  = color.y();
    (*res_ptr)[i].color.b  = color.z();
    (*res_ptr)[i].color.a  = segment_octree_->segments[i].is_narrow_space ? (0.5 + segment_octree_->segments[i].narrowness_factor / 2) : 1;

    (*res_ptr)[i].type    = visualization_msgs::Marker::CUBE_LIST;
    (*res_ptr)[i].scale.x = voxel_size;
    (*res_ptr)[i].scale.y = voxel_size;
    (*res_ptr)[i].scale.z = voxel_size;

    if (segment_octree_->segments[i].border_keys.size() == 0) {
      continue;
    }
    /* m->markers[idx].points.resize(it->border_keys.size()); */
    for (uint k = 0; k < segment_octree_->segments[i].border_keys.size(); k++) {
      octomap::point3d     pos = segment_octree_->keyToCoord(segment_octree_->segments[i].border_keys[k], 16 - segment_octree_->depth_offset);
      geometry_msgs::Point pos2;
      pos2.x = pos.x();
      pos2.y = pos.y();
      pos2.z = pos.z();
      (*res_ptr)[i].points.push_back(pos2);
    }
  }

  int                        i = segment_octree_->segments.size();
  visualization_msgs::Marker marker;
  res_ptr->push_back(marker);

  (*res_ptr)[i].color.r = 0;
  (*res_ptr)[i].color.g = 0;
  (*res_ptr)[i].color.b = 0;
  (*res_ptr)[i].color.a = 1;

  (*res_ptr)[i].type    = visualization_msgs::Marker::POINTS;
  (*res_ptr)[i].scale.x = 0.2;
  (*res_ptr)[i].scale.y = 0.2;
  (*res_ptr)[i].scale.z = 0.2;

  /* m->markers[idx].points.resize(it->border_keys.size()); */
  for (uint k = 0; k < segment_octree_->debug_points_.size(); k++) {
    octomap::point3d     pos = segment_octree_->debug_points_[k];
    geometry_msgs::Point pos2;
    pos2.x = pos.x();
    pos2.y = pos.y();
    pos2.z = pos.z();
    (*res_ptr)[i].points.push_back(pos2);
  }

  return res_ptr;
}
//}

/* getSegmentOctreeMarkersInside()  //{ */
std::shared_ptr<std::vector<visualization_msgs::Marker>> getSegmentOctreeMarkersInside(std::shared_ptr<octomap::SegmentOcTree> segment_octree_) {
  std::vector<visualization_msgs::Marker>*                 ptr     = new std::vector<visualization_msgs::Marker>;
  std::shared_ptr<std::vector<visualization_msgs::Marker>> res_ptr = std::shared_ptr<std::vector<visualization_msgs::Marker>>(ptr);

  size_t octomapSize = segment_octree_->size();
  if (octomapSize <= 1) {
    return NULL;
  }

  for (uint i = 0; i < segment_octree_->segments.size(); i++) {
    visualization_msgs::Marker marker;
    res_ptr->push_back(marker);

    octomap::point3d color = getSegmentColor(segment_octree_->segments[i].id);
    (*res_ptr)[i].color.r  = color.x();
    (*res_ptr)[i].color.g  = color.y();
    (*res_ptr)[i].color.b  = color.z();
    (*res_ptr)[i].color.a  = 1;

    (*res_ptr)[i].type    = visualization_msgs::Marker::CUBE_LIST;
    (*res_ptr)[i].scale.x = 0.8;
    (*res_ptr)[i].scale.y = 0.8;
    (*res_ptr)[i].scale.z = 0.8;

    if (segment_octree_->segments[i].inside_keys.size() == 0) {
      continue;
    }
    /* m->markers[idx].points.resize(it->inside_keys.size()); */
    for (uint k = 0; k < segment_octree_->segments[i].inside_keys.size(); k++) {
      octomap::point3d     pos = segment_octree_->keyToCoord(segment_octree_->segments[i].inside_keys[k], 16 - segment_octree_->depth_offset);
      geometry_msgs::Point pos2;
      pos2.x = pos.x();
      pos2.y = pos.y();
      pos2.z = pos.z();
      (*res_ptr)[i].points.push_back(pos2);
    }
  }

  return res_ptr;
}
//}

/* getSegmapNavGraphMarkers() //{ */
std::vector<visualization_msgs::Marker> getSegmapNavGraphMarkers(std::shared_ptr<SegMap> segmap_) {
  std::vector<visualization_msgs::Marker> res       = {};
  float                                   spheres_d = 0.5;
  for (std::vector<octomap::Segment>::iterator it = segmap_->segment_octree_->segments.begin(); it != segmap_->segment_octree_->segments.end(); it++) {
    /* res.push_back(getMarkerSphere(it->nav_center, spheres_d, getSegmentColor(it->id))); */
    std::vector<octomap::point3d> positions;
    for (uint i = 0; i < it->connected_segments_ids.size(); i++) {
      /* for (uint j = i + 1; j < it->connected_segments_ids.size(); j++) { */
      octomap::SegmentPortal* p = segmap_->segment_octree_->getPortalPtr(it->id, it->connected_segments_ids[i]);
      positions.push_back(p->position);
      res.push_back(getMarkerSphere(p->position, 2.5, octomap::point3d(0, 0, 0)));
      /* octomap::Segment * segptr */
      /* res.push_back(getMarkerLine(p1, p2)); */
      /* } */
    }
    for (uint i = 0; i < positions.size(); i++) {
      for (uint j = i + 1; j < positions.size(); j++) {
        res.push_back(getMarkerLine(positions[i], positions[j], octomap::point3d(0, 0, 0), 1.2));
      }
    }
  }
  for (std::vector<octomap::SegmentPortal>::iterator it = segmap_->segment_octree_->portals.begin(); it != segmap_->segment_octree_->portals.end(); it++) {
    octomap::point3d p1, p2;
    p1 = segmap_->segment_octree_->getSegmentPtr(it->id1)->nav_center;
    p2 = segmap_->segment_octree_->getSegmentPtr(it->id2)->nav_center;
    res.push_back(getMarkerLine(p1, p2));
    /* res.push_back(getMarkerSphere(it->position, spheres_d, octomap::point3d(0, 0, 0))); */
  }

  /* if (is_received_segmap) { */
  /*   uint added_frontier_groups = 0; */
  /*   for (uint i = 0; i < segmap_->frontier_groups_.size(); i++) { */
  /*     added_frontier_groups++; */

  /*     int              connected_seg_id = segmap_->frontier_groups_[i].viable_fep.seg_id; */
  /*     octomap::point3d fep_pos          = segmap_->frontier_groups_[i].viable_fep.pos; */
  /*     float            score            = segmap_->frontier_groups_[i].score; */
  /*     float            prob_unexplored  = segmap_->frontier_groups_[i].probability_unexplored_by_any_robot; */
  /*     if (connected_seg_id < 0 || connected_seg_id > segmap_->segment_octree_->segments.size()) { */
  /*       ROS_WARN_THROTTLE(3, "[Markers]: connected seg id of visualized frontier group is bad: %d", connected_seg_id); */
  /*       continue; */
  /*     } */
  /*     octomap::point3d seg_pos = segmap_->segment_octree_->segments[connected_seg_id].center; */

  /*     res.push_back(getMarkerLine(fep_pos, seg_pos, octomap::point3d(1, 0, 1) * prob_unexplored)); */
  /*     res.push_back(getMarkerCube(fep_pos, 0.1 + score * 1, octomap::point3d(1, 0, 1) * prob_unexplored)); */
  /*   } */
  /* ROS_INFO("[Markers]: added %d frontier markers", added_frontier_groups); */
  /* } */
  return res;
}
//}

/* getSegmapDetailedMarkers() //{ */
std::vector<visualization_msgs::Marker> getSegmapDetailedMarkers(std::shared_ptr<SegMap> segmap_, bool is_received_segmap) {
  std::vector<visualization_msgs::Marker> res       = {};
  float                                   spheres_d = 0.5;
  for (std::vector<octomap::Segment>::iterator it = segmap_->segment_octree_->segments.begin(); it != segmap_->segment_octree_->segments.end(); it++) {
    res.push_back(getMarkerSphere(it->nav_center, spheres_d, getSegmentColor(it->id)));
  }
  for (std::vector<octomap::SegmentPortal>::iterator it = segmap_->segment_octree_->portals.begin(); it != segmap_->segment_octree_->portals.end(); it++) {
    octomap::point3d p1, p2;
    p1 = segmap_->segment_octree_->getSegmentPtr(it->id1)->nav_center;
    p2 = segmap_->segment_octree_->getSegmentPtr(it->id2)->nav_center;
    res.push_back(getMarkerLine(p1, p2));
  }

  if (is_received_segmap) {
    uint added_frontier_groups = 0;
    for (uint i = 0; i < segmap_->frontier_groups_.size(); i++) {
      added_frontier_groups++;

      int              connected_seg_id = segmap_->frontier_groups_[i].viable_fep.seg_id;
      octomap::point3d fep_pos          = segmap_->frontier_groups_[i].viable_fep.pos;
      float            score            = segmap_->frontier_groups_[i].score;
      float            prob_unexplored  = segmap_->frontier_groups_[i].probability_unexplored_by_any_robot;
      if (connected_seg_id < 0 || connected_seg_id > segmap_->segment_octree_->segments.size()) {
        ROS_WARN_THROTTLE(3, "[Markers]: connected seg id of visualized frontier group is bad: %d", connected_seg_id);
        continue;
      }
      octomap::point3d seg_pos = segmap_->segment_octree_->segments[connected_seg_id].center;

      res.push_back(getMarkerLine(fep_pos, seg_pos, octomap::point3d(1, 0, 1) * prob_unexplored));
      res.push_back(getMarkerCube(fep_pos, 8, octomap::point3d(1, 0, 1) * prob_unexplored));
    }
    /* ROS_INFO("[Markers]: added %d frontier markers", added_frontier_groups); */
  }
  return res;
}
//}

/* getSegmapBlockMarkers() //{ */
std::vector<visualization_msgs::Marker> getSegmapBlockMarkers(std::shared_ptr<SegMap> segmap_, bool is_received_segmap) {
  if (segmap_ == NULL) {
    ROS_ERROR("dont give me null segmaps you retard");
    return {};
  }
  std::vector<visualization_msgs::Marker> res = {};
  for (std::vector<octomap::Segment>::iterator it = segmap_->segment_octree_->segments.begin(); it != segmap_->segment_octree_->segments.end(); it++) {
    octomap::point3d           block_clr = octomap::point3d(0, 1, 0) * (1 - it->frontier_value);
    visualization_msgs::Marker mrk       = getMarkerBlock(it->center, it->block_alpha, it->block_beta, it->block_a, it->block_b, it->block_c, block_clr, 0.5);
    /* mrk.color                      = heatMapColor(0.8 * it->surface_coverage_local, 0.5); */
    /* float f                        = it->global_frontier_value; */
    /* if (!is_received_segmap) { */
    /*   f = it->frontier_value; */
    /* } */
    /* if (it->global_frontier_value > 0.1) { */
    /*   mrk.color.r = f; */
    /*   mrk.color.g = 0; */
    /*   mrk.color.b = f; */
    /*   mrk.color.a = 1; */
    /* } */
    res.push_back(mrk);
  }
  for (std::vector<octomap::SegmentPortal>::iterator it = segmap_->segment_octree_->portals.begin(); it != segmap_->segment_octree_->portals.end(); it++) {
    octomap::point3d p1, p2;
    if (segmap_->segment_octree_->getSegmentPtr(it->id1) == NULL || segmap_->segment_octree_->getSegmentPtr(it->id2) == NULL) {
      ROS_ERROR("segmap visualization error");
      return {};
    }
    p1 = segmap_->segment_octree_->getSegmentPtr(it->id1)->nav_center;
    p2 = segmap_->segment_octree_->getSegmentPtr(it->id2)->nav_center;
    res.push_back(getMarkerLine(p1, p2));
  }

  if (is_received_segmap) {
    uint added_frontier_groups = 0;
    for (uint i = 0; i < segmap_->frontier_groups_.size(); i++) {
      added_frontier_groups++;

      int                 connected_seg_id = segmap_->frontier_groups_[i].viable_fep.seg_id;
      octomap::point3d    fep_pos          = segmap_->frontier_groups_[i].viable_fep.pos;
      float               score            = segmap_->frontier_groups_[i].score;
      float               prob_unexplored  = segmap_->frontier_groups_[i].probability_unexplored_by_any_robot;
      std_msgs::ColorRGBA prob_color       = heatMapColor(0.8 * prob_unexplored, 1);
      if (connected_seg_id < 0 || connected_seg_id > segmap_->segment_octree_->segments.size()) {
        ROS_WARN_THROTTLE(3, "[Markers]: connected seg id of visualized frontier group is bad: %d", connected_seg_id);
        continue;
      }

      if (segmap_->segment_octree_->segments.size() <= connected_seg_id) {
        ROS_ERROR("segmap visualization error with frontiers");
        return {};
      }
      octomap::point3d seg_pos = segmap_->segment_octree_->segments[connected_seg_id].center;

      float sphere_size = 3 + score * 5;
      res.push_back(getMarkerLine(fep_pos, seg_pos, octomap::point3d(prob_color.r, prob_color.g, prob_color.b), 1));
      res.push_back(getMarkerSphere(fep_pos, sphere_size, octomap::point3d(prob_color.r, prob_color.g, prob_color.b)));
    }
    /* ROS_INFO("[Markers]: added %d frontier markers", added_frontier_groups); */
  }
  return res;
}
//}

/* getSegmapCoverageMarkers() //{ */
std::vector<visualization_msgs::Marker> getSegmapCoverageMarkers(std::shared_ptr<SegMap> segmap_) {
  std::vector<visualization_msgs::Marker> res       = {};
  float                                   spheres_d = 3;
  for (std::vector<octomap::Segment>::iterator it = segmap_->segment_octree_->segments.begin(); it != segmap_->segment_octree_->segments.end(); it++) {
    res.push_back(
        getMarkerSphere(it->center, it->bounding_sphere_radius * 2, octomap::point3d(it->surface_coverage_local, it->surface_coverage_local, 0), 0.5));
  }
  for (std::vector<octomap::SegmentPortal>::iterator it = segmap_->segment_octree_->portals.begin(); it != segmap_->segment_octree_->portals.end(); it++) {
    octomap::point3d p1, p2;
    p1 = segmap_->segment_octree_->getSegmentPtr(it->id1)->nav_center;
    p2 = segmap_->segment_octree_->getSegmentPtr(it->id2)->nav_center;
    res.push_back(getMarkerLine(p1, p2));
  }

  return res;
}
//}

/* getSegmapReceivedMarkers() //{ */
std::vector<visualization_msgs::Marker> getSegmapReceivedMarkers(std::shared_ptr<SegMap> segmap_) {
  std::vector<visualization_msgs::Marker> res = {};
  /* float                                   spheres_d = 0.5; */
  /* for (std::vector<octomap::Segment>::iterator it = segmap_->segment_octree_->segments.begin(); it != segmap_->segment_octree_->segments.end(); it++) { */
  /*   octomap::point3d color(0.1, 0.1, 0.1); */
  /*   if (it->associated_with_current_map) { */
  /*     color = octomap::point3d(1, 0.8, 0); */
  /*   } */
  /*   res.push_back(getMarkerSphere(it->nav_center, spheres_d, color)); */
  /* } */
  /* for (std::vector<octomap::SegmentPortal>::iterator it = segmap_->segment_octree_->portals.begin(); it != segmap_->segment_octree_->portals.end(); it++) {
   */
  /*   octomap::point3d p1, p2; */
  /*   p1 = segmap_->segment_octree_->getSegmentPtr(it->id1)->nav_center; */
  /*   p2 = segmap_->segment_octree_->getSegmentPtr(it->id2)->nav_center; */
  /*   res.push_back(getMarkerLine(p1, p2)); */
  /* } */
  return res;
}
//}

/* SPHEREMAP MARKERS */

/* getSpheremapMarkers() //{ */
std::vector<visualization_msgs::Marker> getSpheremapMarkers(octomap::point3d center, float box_halfsize, std::shared_ptr<SphereMap> spheremap_) {
  std::vector<visualization_msgs::Marker>           res = {};
  octomap::point3d                                  color(0, 1, 0);
  octomap::point3d                                  frontier_color(1, 0, 1);
  octomap::point3d                                  unsafe_color(1, 0, 0);
  float                                             alpha           = 0.5;
  float                                             min_unsafe_dist = spheremap_->planning_min_safe_dist;
  float                                             max_unsafe_dist = spheremap_->planning_base_safe_dist;
  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys        = spheremap_->getMaxSearchBBXBorderKeys(center, box_halfsize);
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = spheremap_->nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second, 16);
       it != spheremap_->nodes->end_leafs_bbx(); it++) {
    float mod = 1;
    if (it->valuePtr()->is_safe) {
      if (it->valuePtr()->radius < max_unsafe_dist) {
        mod = 1 - fmin(((max_unsafe_dist - it->valuePtr()->radius - min_unsafe_dist) / (max_unsafe_dist - min_unsafe_dist)), 1);
      }
    } else {
      mod = 0;
    }
    std_msgs::ColorRGBA clr = heatMapColor(0.8 * mod / 2, alpha);
    res.push_back(getMarkerSphere(it->valuePtr()->pos, it->valuePtr()->radius * 2, octomap::point3d(clr.r, clr.g, clr.b), alpha));
  }
  return res;
}
//}

/* getSpheremapDebugMarkers() //{ */
std::vector<visualization_msgs::Marker> getSpheremapDebugMarkers(std::shared_ptr<SphereMap> spheremap_) {
  octomap::point3d    octomap_clr;
  std_msgs::ColorRGBA marker_clr;
  marker_clr.a = 1;

  visualization_msgs::Marker marker;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1;

  marker.type = visualization_msgs::Marker::POINTS;

  for (uint i = 0; i < spheremap_->debug_points.size(); i++) {
    geometry_msgs::Point pos2;
    pos2.x = spheremap_->debug_points[i].x();
    pos2.y = spheremap_->debug_points[i].y();
    pos2.z = spheremap_->debug_points[i].z();
    marker.points.push_back(pos2);
  }

  std::vector<visualization_msgs::Marker> res = {marker};
  return res;
}
//}

/* getSpheremapPointMarkers() //{ */
std::vector<visualization_msgs::Marker> getSpheremapPointMarkers(octomap::point3d center, float box_halfsize, std::shared_ptr<SphereMap> spheremap_,
                                                                 bool is_visited_positions_map, float node_maxval, float val_decrease_dist,
                                                                 float blocking_dist) {
  std::vector<visualization_msgs::Marker> res = {};
  octomap::point3d                        line_color(0, 1, 0);
  float                                   alpha = 0.5;

  /* std_msgs::ColorRGBA        unsafe_color; */
  octomap::point3d    octomap_clr;
  std_msgs::ColorRGBA marker_clr;
  marker_clr.a = 1;

  visualization_msgs::Marker marker;

  if (is_visited_positions_map) {
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
  } else {
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
  }

  marker.color.r = 0.2;
  marker.color.g = 0.2;
  marker.color.b = 0.2;
  marker.color.a = 1;

  marker.type = visualization_msgs::Marker::POINTS;

  visualization_msgs::Marker line_marker;
  line_marker.type    = visualization_msgs::Marker::LINE_LIST;
  line_marker.scale.x = 0.05;
  line_marker.scale.y = 0.05;
  line_marker.scale.z = 0.05;

  line_marker.color.r = 0;
  line_marker.color.g = 0;
  line_marker.color.b = 0;
  line_marker.color.a = 1;

  std::pair<octomap::OcTreeKey, octomap::OcTreeKey> bbx_keys = spheremap_->getMaxSearchBBXBorderKeys(center, box_halfsize);
  for (octomap::SphereMapOcTree::leaf_bbx_iterator it = spheremap_->nodes->begin_leafs_bbx(bbx_keys.first, bbx_keys.second, 16);
       it != spheremap_->nodes->end_leafs_bbx(); it++) {
    geometry_msgs::Point pos2;
    pos2.x = it->valuePtr()->pos.x();
    pos2.y = it->valuePtr()->pos.y();
    pos2.z = it->valuePtr()->pos.z();
    marker.points.push_back(pos2);

    octomap_clr = getSegmentColor(it->valuePtr()->segment_id);
    if (is_visited_positions_map) {
      float mod = it->getTimeStayed() / node_maxval;
      if (mod > 1) {
        marker_clr.r = 1;
        marker_clr.g = 0;
        marker_clr.b = 0;
      } else {
        marker_clr.r = 0;
        marker_clr.g = (1 - mod);
        marker_clr.b = 0;
      }
    } else {
      marker_clr.r = octomap_clr.x();
      marker_clr.g = octomap_clr.y();
      marker_clr.b = octomap_clr.z();
    }
    marker.colors.push_back(marker_clr);

    /* ADD LINE MARKER */
    for (uint i = 0; i < it->valuePtr()->connected_keys.size(); i++) {
      auto pos = it.getCoordinate();
      pos2.x   = pos.x();
      pos2.y   = pos.y();
      pos2.z   = pos.z();
      line_marker.points.push_back(pos2);

      pos    = spheremap_->nodes->keyToCoord(it->valuePtr()->connected_keys[i]);
      pos2.x = pos.x();
      pos2.y = pos.y();
      pos2.z = pos.z();
      line_marker.points.push_back(pos2);
    }
  }

  res.push_back(marker);
  res.push_back(line_marker);
  return res;
}
//}

/* getSpheremapSegmentMarkers() //{ */
std::vector<visualization_msgs::Marker> getSpheremapSegmentMarkers(std::shared_ptr<SphereMap> spheremap_) {
  std::vector<visualization_msgs::Marker> res = {};
  octomap::point3d                        line_color(0, 1, 0);
  float                                   alpha = 0.5;

  /* std_msgs::ColorRGBA        unsafe_color; */
  octomap::point3d    octomap_clr;
  std_msgs::ColorRGBA marker_clr;
  marker_clr.a = 1;

  visualization_msgs::Marker marker;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;

  marker.color.r = 0.2;
  marker.color.g = 0.2;
  marker.color.b = 0.2;
  marker.color.a = 1;

  marker.type = visualization_msgs::Marker::POINTS;

  visualization_msgs::Marker line_marker;
  line_marker.type    = visualization_msgs::Marker::LINE_LIST;
  line_marker.scale.x = 0.1;
  line_marker.scale.y = 0.1;
  line_marker.scale.z = 0.1;

  line_marker.color.r = 0;
  line_marker.color.g = 0;
  line_marker.color.b = 0;
  line_marker.color.a = 1;

  /* visualization_msgs::Marker getMarkerLine(octomap::point3d pos, octomap::point3d endpos, octomap::point3d color, float linewidth) { */
  for (std::map<uint, SphereMapSegment>::iterator it = spheremap_->segments.begin(); it != spheremap_->segments.end(); it++) {
    /* VIS SEGMENT CENTER */
    geometry_msgs::Point pos2;
    pos2.x = it->second.center.x();
    pos2.y = it->second.center.y();
    pos2.z = it->second.center.z();
    marker.points.push_back(pos2);

    octomap_clr  = getSegmentColor(it->first);
    marker_clr.r = octomap_clr.x();
    marker_clr.g = octomap_clr.y();
    marker_clr.b = octomap_clr.z();
    marker.colors.push_back(marker_clr);

    /* res.push_back(getMarkerSphere(it->second.center, it->second.bounding_sphere_radius * 2, octomap_clr, 0.2)); */

    /* VIS SEGMENT CONNECTIONS */
    /* for (std::map<uint, SphereMapSegmentConnection>::iterator connection_iterator = it->second.connections.begin(); */
    /*      connection_iterator != it->second.connections.end(); connection_iterator++) { */
    /*   std::map<uint, SphereMapSegment>::iterator adj_seg_query = spheremap_->segments.find(connection_iterator->first); */
    /*   if (adj_seg_query == spheremap_->segments.end()) { */
    /*     ROS_ERROR("there is a portal in spheremap segments that leads to null segment"); */
    /*     continue; */
    /*   } */
    /*   octomap::point3d pos = it->second.center; */
    /*   pos2.x               = pos.x(); */
    /*   pos2.y               = pos.y(); */
    /*   pos2.z               = pos.z(); */
    /*   line_marker.points.push_back(pos2); */

    /*   pos    = adj_seg_query->second.center; */
    /*   pos2.x = pos.x(); */
    /*   pos2.y = pos.y(); */
    /*   pos2.z = pos.z(); */
    /*   line_marker.points.push_back(pos2); */
    /* } */
  }

  for (uint i = 0; i < spheremap_->portals.size(); i++) {
    geometry_msgs::Point                       pos2;
    std::map<uint, SphereMapSegment>::iterator s1_query = spheremap_->segments.find(spheremap_->portals[i].first);
    std::map<uint, SphereMapSegment>::iterator s2_query = spheremap_->segments.find(spheremap_->portals[i].second);
    octomap::point3d                           pos      = s1_query->second.center;
    pos2.x                                              = pos.x();
    pos2.y                                              = pos.y();
    pos2.z                                              = pos.z();
    line_marker.points.push_back(pos2);

    pos    = s2_query->second.center;
    pos2.x = pos.x();
    pos2.y = pos.y();
    pos2.z = pos.z();
    line_marker.points.push_back(pos2);
  }

  res.push_back(marker);
  res.push_back(line_marker);
  return res;
}
//}

/* getSpheremapNavigationMarkers() //{ */
std::vector<visualization_msgs::Marker> getSpheremapNavigationMarkers(std::shared_ptr<SphereMap> spheremap_) {
  std::vector<visualization_msgs::Marker> res = {};

  uint num_paths_visualized = 0;
  for (std::map<uint, SphereMapSegment>::iterator it = spheremap_->segments.begin(); it != spheremap_->segments.end(); it++) {
    for (std::map<std::pair<uint, uint>, SphereMapPath>::iterator path_it = it->second.interportal_paths.begin(); path_it != it->second.interportal_paths.end();
         path_it++) {
      float                      min_odist = spheremap_->planning_base_safe_dist;
      visualization_msgs::Marker line_marker;
      line_marker.type    = visualization_msgs::Marker::LINE_STRIP;
      line_marker.scale.x = 0.1;
      line_marker.scale.y = 0.1;
      line_marker.scale.z = 0.1;

      for (uint i = 0; i < path_it->second.positions.size(); i++) {
        geometry_msgs::Point pos2;
        pos2.x = path_it->second.positions[i].x();
        pos2.y = path_it->second.positions[i].y();
        pos2.z = path_it->second.positions[i].z();
        line_marker.points.push_back(pos2);

        float odist = path_it->second.obstacle_dists[i];
        if (odist < min_odist) {
          min_odist = odist;
        }
      }
      num_paths_visualized++;


      float safety_factor =
          fmax(fmin((min_odist - spheremap_->planning_min_safe_dist) / (spheremap_->planning_base_safe_dist - spheremap_->planning_min_safe_dist), 1), 0);
      if (min_odist < spheremap_->planning_min_safe_dist) {
        line_marker.color.r = 0.5;
        line_marker.color.g = 0;
        line_marker.color.b = 0;
        line_marker.color.a = 1;
      } else {
        line_marker.color = heatMapColor(safety_factor * 0.3, 1);
      }
      res.push_back(line_marker);
    }
  }

  for (uint i = 0; i < spheremap_->portals.size(); i++) {
    visualization_msgs::Marker line_marker;
    line_marker.type    = visualization_msgs::Marker::LINE_STRIP;
    line_marker.scale.x = 0.15;
    line_marker.scale.y = 0.15;
    line_marker.scale.z = 0.15;

    line_marker.color.r = 0;
    line_marker.color.g = 0;
    line_marker.color.b = 0;
    line_marker.color.a = 1;

    geometry_msgs::Point                       pos2;
    std::map<uint, SphereMapSegment>::iterator s1_query   = spheremap_->segments.find(spheremap_->portals[i].first);
    auto                                       connection = s1_query->second.connections.find(spheremap_->portals[i].second);
    /* std::map<uint, SphereMapSegment>::iterator s2_query = spheremap_->segments.find(spheremap_->portals[i].second); */
    octomap::point3d pos = spheremap_->nodes->search(connection->second.other_key, 16)->valuePtr()->pos;
    pos2.x               = pos.x();
    pos2.y               = pos.y();
    pos2.z               = pos.z();
    line_marker.points.push_back(pos2);

    pos    = spheremap_->nodes->search(connection->second.own_key, 16)->valuePtr()->pos;
    pos2.x = pos.x();
    pos2.y = pos.y();
    pos2.z = pos.z();
    line_marker.points.push_back(pos2);
    res.push_back(line_marker);
  }

  /* res.push_back(marker); */
  return res;
}
//}

/* getPointsMarkers() //{ */
std::vector<visualization_msgs::Marker> getPointsMarkers(std::vector<pcl::PointXYZ> points, octomap::point3d color, float size) {
  std_msgs::ColorRGBA clr_explored;
  clr_explored.r = color.x();
  clr_explored.g = color.y();
  clr_explored.b = color.z();
  clr_explored.a = 1;

  visualization_msgs::Marker marker;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;

  marker.type = visualization_msgs::Marker::POINTS;
  geometry_msgs::Point pos2;

  for (pcl::PointXYZ p : points) {
    pos2.x = p.x;
    pos2.y = p.y;
    pos2.z = p.z;
    marker.points.push_back(pos2);
    marker.colors.push_back(clr_explored);
  }

  return {marker};
}
//}

/* std::vector<visualization_msgs::Marker> getSphereMapPathsMarkers() //{ */
std::vector<visualization_msgs::Marker> getSphereMapPathsMarkers(std::vector<SphereMapPath> paths, octomap::point3d path_color, float path_line_width) {
  /* marker.type = visualization_msgs::Marker::POINTS; */
  /* geometry_msgs::Point                    pos2; */
  std::vector<visualization_msgs::Marker> res = {};


  for (SphereMapPath p : paths) {
    res.push_back(getMarkerCube(p.original_start, 1, octomap::point3d(0, 0, 0)));
    res.push_back(getMarkerCube(p.original_goal, 1, octomap::point3d(0, 0, 0)));

    if (!p.reaches_goal) {
      continue;
    }


    visualization_msgs::Marker line_marker;
    line_marker.type    = visualization_msgs::Marker::LINE_STRIP;
    line_marker.scale.x = path_line_width;
    line_marker.scale.y = path_line_width;
    line_marker.scale.z = path_line_width;

    line_marker.color.r = path_color.x();
    line_marker.color.g = path_color.y();
    line_marker.color.b = path_color.z();
    line_marker.color.a = 1;

    for (uint i = 0; i < p.positions.size(); i++) {
      geometry_msgs::Point pos2;
      pos2.x = p.positions[i].x();
      pos2.y = p.positions[i].y();
      pos2.z = p.positions[i].z();
      line_marker.points.push_back(pos2);
    }
    res.push_back(line_marker);
  }

  return res;
}
//}

/* FACETMAP MARKERS */

/* std::vector<visualization_msgs::Marker> getFacetMapMarkersFull() //{ */
std::vector<visualization_msgs::Marker> getFacetMapMarkersFull(octomap::point3d center, float box_halflength,
                                                               std::shared_ptr<octomap::SurfaceOcTree> surface_octree_) {
  std::vector<visualization_msgs::Marker> markers = {};

  for (octomap::SurfaceOcTree::leaf_bbx_iterator it =
           surface_octree_->begin_leafs_bbx(center - octomap::point3d(1, 1, 1) * box_halflength, center + octomap::point3d(1, 1, 1) * box_halflength);
       it != surface_octree_->end_leafs_bbx(); it++) {

    std::vector<SurfaceNode>* nodes_ptr = it->getExploredPtr();
    for (std::vector<SurfaceNode>::iterator it2 = nodes_ptr->begin(); it2 != nodes_ptr->end(); it2++) {
      markers.push_back(getMarkerArrow(it2->pos, it2->normal * 0.05, octomap::point3d(0, 0, 1), 0.4, 0.6, 0.1));
    }
    nodes_ptr = it->getUnexploredPtr();
    for (std::vector<SurfaceNode>::iterator it2 = nodes_ptr->begin(); it2 != nodes_ptr->end(); it2++) {
      markers.push_back(getMarkerArrow(it2->pos, it2->normal * 0.05, octomap::point3d(1, 0, 0), 0.4, 0.6, 0.1));
    }
  }

  return markers;
}
//}

/* std::vector<visualization_msgs::Marker> getFacetMapMarkersPoints() //{ */
std::vector<visualization_msgs::Marker> getFacetMapMarkersPoints(octomap::point3d center, float box_halflength,
                                                                 std::shared_ptr<octomap::SurfaceOcTree> surface_octree_) {
  std_msgs::ColorRGBA clr_explored;
  std_msgs::ColorRGBA clr_unexplored;
  clr_explored.r   = 0;
  clr_explored.g   = 0;
  clr_explored.b   = 1;
  clr_explored.a   = 1;
  clr_unexplored.r = 1;
  clr_unexplored.g = 0;
  clr_unexplored.b = 0;
  clr_unexplored.a = 1;

  visualization_msgs::Marker marker;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.type = visualization_msgs::Marker::POINTS;
  geometry_msgs::Point pos2;

  for (octomap::SurfaceOcTree::leaf_bbx_iterator it =
           surface_octree_->begin_leafs_bbx(center - octomap::point3d(1, 1, 1) * box_halflength, center + octomap::point3d(1, 1, 1) * box_halflength);
       it != surface_octree_->end_leafs_bbx(); it++) {

    std::vector<SurfaceNode>* nodes_ptr = it->getExploredPtr();
    for (std::vector<SurfaceNode>::iterator it2 = nodes_ptr->begin(); it2 != nodes_ptr->end(); it2++) {
      pos2.x = it2->pos.x();
      pos2.y = it2->pos.y();
      pos2.z = it2->pos.z();
      marker.points.push_back(pos2);
      marker.colors.push_back(clr_explored);
    }
    nodes_ptr = it->getUnexploredPtr();
    for (std::vector<SurfaceNode>::iterator it2 = nodes_ptr->begin(); it2 != nodes_ptr->end(); it2++) {
      pos2.x = it2->pos.x();
      pos2.y = it2->pos.y();
      pos2.z = it2->pos.z();
      marker.points.push_back(pos2);
      marker.colors.push_back(clr_unexplored);
    }
  }

  return {marker};
}
//}

/* UTILS */

/* getFrontierExplorationData() //{ */
std::optional<FrontierExplorationPoint> getFrontierExplorationData(octomap::point3d pos, int num_rays, float max_ray_dist,
                                                                   std::shared_ptr<octomap::OcTree> occupancy_octree_, bool only_look_down) {
  // TODO check if can compute here

  std::vector<octomap::OcTreeKey> unique_keys = {};

  std::vector<octomap::point3d> occupied_hits             = {};
  int                           num_unexplored_points_hit = 0;
  int                           num_occupied_points_hit   = 0;
  int                           unexplored_hits_up        = 0;
  int                           unexplored_hits_down      = 0;
  int                           unexplored_hits_horiz     = 0;
  float                         horiz_threshold           = 0.4;

  octomap::point3d     normal_sum(0, 0, 0);
  octomap::OcTreeNode* ocnode;
  float                lidar_vertical_fov_degrees = only_look_down ? M_PI / 8 : M_PI / 2;
  float                min_vertical_angle         = -lidar_vertical_fov_degrees;
  float                vertical_angle_delta       = lidar_vertical_fov_degrees * 2;

  /* CAST RAYS IN ALL DIRECTIONS UNDER SOME MAX ANGLE */
  for (int i = 0; i < num_rays; i++) {
    /* octomap::point3d dir_vec = getRandomPointInSphere(1).normalized(); */
    float            phi       = (double(rand()) / RAND_MAX) * 2 * M_PI;
    float            theta     = (double(rand()) / RAND_MAX) * vertical_angle_delta + min_vertical_angle;
    float            cs        = cos(phi);
    float            sn        = sin(phi);
    float            cos_theta = cos(theta);
    float            sin_theta = sin(theta);
    float            rotated_x = cs * cos_theta;
    float            rotated_y = sn * cos_theta;
    float            rotated_z = sin_theta;
    octomap::point3d dir_vec(rotated_x, rotated_y, rotated_z);

    octomap::point3d hit_point;
    bool             ray_res = occupancy_octree_->castRay(pos, dir_vec, hit_point, false, max_ray_dist);
    /* RAY RES IS FALSE IF UNKNOWN NODE WAS HIT OR IF RAY EXCEEDED MAX RANGE*/
    if (!ray_res) {

      ocnode = occupancy_octree_->search(hit_point, 14);
      if (ocnode == NULL || occupancy_octree_->nodeHasChildren(ocnode)) {
        num_unexplored_points_hit++;
        normal_sum += dir_vec;
        if (dir_vec.z() > horiz_threshold) {
          unexplored_hits_up++;
        } else if (dir_vec.z() < -horiz_threshold) {
          unexplored_hits_down++;
        } else {
          unexplored_hits_horiz++;
        }
      }
      continue;
    } else {
      num_occupied_points_hit++;
      occupied_hits.push_back(hit_point);
    }
  }
  if (num_unexplored_points_hit == 0) {
    /* ROS_WARN("[FG]: FrontierExplorationPoint hit 0 unknown nodes"); */
    return std::nullopt;
  }
  if (num_occupied_points_hit == 0) {
    /* ROS_WARN("[FG]: FrontierExplorationPoint hit 0 occupied nodes"); */
    return std::nullopt;
  }
  /* ROS_INFO("[FG]: FEP num_unknown hits: %d, occupied_hits: %d, normal_sum_mag: %f", num_unexplored_points_hit, num_occupied_points_hit, normal_sum.norm());
   */

  FrontierExplorationPoint res;
  res.pos                           = pos;
  res.frontier_direction            = normal_sum.normalized();
  res.perc_horizontal_frontier_hits = (float)(unexplored_hits_horiz) / num_unexplored_points_hit;
  res.perc_vertical_frontier_hits   = 1 - res.perc_horizontal_frontier_hits;
  res.perc_frontier_hits            = ((float)num_unexplored_points_hit) / num_rays;

  return res;
}
//}

/* getNearestSegmentRaycasting() //{ */
int getNearestSegmentRaycasting(octomap::point3d pos, int num_rays, float max_dist, std::shared_ptr<octomap::OcTree> occupancy_octree_,
                                std::shared_ptr<octomap::SegmentOcTree> seg_octree_) {
  /* int                   search_depth = 16 - seg_octree_->depth_offset; */
  octomap::SegmentNode* node = seg_octree_->search(pos);
  if (node != NULL && node->getSegmentID() > -1) {
    return node->getSegmentID();
  }
  for (int i = 0; i < num_rays; i++) {
    octomap::point3d dir_vec = getRandomPointInSphere(1).normalized();

    octomap::KeyRay ray;
    occupancy_octree_->computeRayKeys(pos, pos + dir_vec * max_dist, ray);

    for (octomap::KeyRay::iterator it = ray.begin(), end = ray.end(); it != end; ++it) {
      octomap::OcTreeNode* s_node = occupancy_octree_->search(*it);
      if (s_node != NULL && !occupancy_octree_->isNodeOccupied(s_node)) {
        node = seg_octree_->search(*it);
        if (node != NULL && node->getSegmentID() > -1) {
          return node->getSegmentID();
        }
      } else {
        break;
      }
    }
  }
  /* if no seg hit for any ray, return -1 */
  return -1;
}
//}

/* getNearestSegmentRaycasting2() //{ */
int getNearestSegmentRaycasting2(octomap::point3d pos, int num_rays, float max_dist, std::shared_ptr<octomap::OcTree> occupancy_octree_,
                                 std::shared_ptr<octomap::SegmentOcTree> seg_octree_) {
  /* int                   search_depth = 16 - seg_octree_->depth_offset; */
  float                 nearest_seg_dist = 10000;
  int                   nearest_seg_id   = -1;
  octomap::SegmentNode* node             = seg_octree_->search(pos);
  octomap::point3d      hit_pos;
  if (node != NULL && node->getSegmentID() > -1) {
    return node->getSegmentID();
  }
  for (int i = 0; i < num_rays; i++) {
    octomap::point3d dir_vec = getRandomPointInSphere(1).normalized();

    octomap::KeyRay ray;
    occupancy_octree_->computeRayKeys(pos, pos + dir_vec * max_dist, ray);

    for (octomap::KeyRay::iterator it = ray.begin(), end = ray.end(); it != end; ++it) {
      octomap::OcTreeNode* s_node = occupancy_octree_->search(*it);
      if (s_node != NULL && !occupancy_octree_->isNodeOccupied(s_node)) {
        node = seg_octree_->search(*it);
        if (node != NULL && node->getSegmentID() > -1) {
          /* SEGMENT HIT! */
          hit_pos    = occupancy_octree_->keyToCoord(*it);
          float dist = (pos - hit_pos).norm();
          if (dist < nearest_seg_dist) {
            nearest_seg_id   = node->getSegmentID();
            nearest_seg_dist = dist;
          }
          break;
        }
      } else {
        break;
      }
    }
  }
  /* if no seg hit for any ray, return -1 */
  return nearest_seg_id;
}
//}

/* filterPoints() //{ */
void filterPoints(std::vector<octomap::point3d>* in, std::vector<octomap::point3d>* out, float filter_dist) {
  float dist2 = filter_dist * filter_dist;
  for (uint i = 0; i < in->size(); i++) {
    bool filtered = false;
    for (uint j = 0; j < out->size(); j++) {
      octomap::point3d deltavec = in->at(i) - out->at(j);
      if (deltavec.dot(deltavec) < dist2) {
        filtered = true;
        break;
      }
    }
    if (!filtered) {
      out->push_back(in->at(i));
    }
  }
}
//}

/* getCylinderSamplingPoints() //{ */
std::vector<octomap::point3d> getCylinderSamplingPoints(int num_points_circle, float delta_r, float delta_z, int num_circles, int num_layers) {
  float                         delta_phi = 2 * M_PI / num_points_circle;
  std::vector<octomap::point3d> res       = {};

  std::vector<float> sins = {};
  std::vector<float> coss = {};

  for (int p = 0; p < num_points_circle; p++) {
    sins.push_back(sin(p * delta_phi));
    coss.push_back(cos(p * delta_phi));
  }

  /* TEST POINTS IN CYLINDERS OF INCREASING WIDTH */
  for (int h = -num_layers + 1; h < num_layers; h++) {
    float dz = h * delta_z;
    for (int r = 1; r < num_circles; r++) {
      float radius = delta_r * r;
      for (int p = 0; p < num_points_circle; p++) {
        float dx = coss[p] * radius;
        float dy = sins[p] * radius;

        res.push_back(octomap::point3d(dx, dy, dz));
      }
    }
  }
  return res;
}
//}

std::vector<octomap::point3d> blockAnglesToDirections(float alpha, float beta) {
  /* std::vector<octomap::point3d> res = {, octomap::point3d(0, 1, 0), octomap::point3d(0, 0, 1)}; */
  octomap::point3d x = octomap::point3d(cos(alpha) * cos(beta), -sin(alpha) * cos(beta), sin(beta));
  octomap::point3d y = octomap::point3d(sin(alpha), cos(alpha), 0);
  octomap::point3d z = octomap::point3d(-sin(beta) * cos(alpha), sin(alpha) * sin(beta), cos(beta));
  return {x, y, z};
}
std::pair<float, float> directionsToBlockAngles(std::vector<octomap::point3d> p) {
  float alpha = atan2(p[0].y(), p[0].x());
  float rho   = sqrt(pow(p[0].y(), 2) + pow(p[0].x(), 2));
  float beta  = atan2(p[0].z(), rho);
  return std::make_pair(alpha, beta);
}

std::pair<float, float> calculateBestFitAlphaBeta(std::vector<octomap::point3d>& pts, float sa, float sb) {
  int   num_tries_alpha = 5;
  float current_alpha   = M_PI / 4;
  float step            = M_PI / 8;
  int   ptsize          = pts.size();

  float apmax, bpmax, aproj, bproj, size;
  float leftval, rightval;
  for (int i = 0; i < num_tries_alpha; i++) {
    for (int j = 0; j < 2; j++) {
      float test_alpha  = current_alpha + (j - 0.5) * 2 * step;
      float test_adir_x = cos(test_alpha);
      float test_adir_y = sin(test_alpha);

      float test_bdir_x = cos(test_alpha + M_PI / 2);
      float test_bdir_y = sin(test_alpha + M_PI / 2);
      apmax             = 0;
      bpmax             = 0;
      for (int k = 0; k < ptsize; k++) {
        aproj = abs(pts[k].x() * test_adir_x + pts[k].y() * test_adir_y);
        bproj = abs(pts[k].x() * test_bdir_x + pts[k].y() * test_bdir_y);
        if (aproj > apmax) {
          apmax = aproj;
        }
        if (bproj > bpmax) {
          bpmax = bproj;
        }
      }
      size = apmax * bpmax;
      if (j == 0) {
        leftval = size;
      } else {
        rightval = size;
      }
    }

    /* ROS_INFO("leftval: %f, rightval: %f", leftval, rightval); */
    if (rightval < leftval) {
      current_alpha = current_alpha + step;
    } else {
      current_alpha = current_alpha - step;
    }
    step = step / 2;
  }
  /* ROS_INFO("best alpha: %f", current_alpha); */

  return std::make_pair(current_alpha, 0);
}

std::pair<float, float> calculateBestFitAlphaBeta(std::vector<octomap::point3d>& pts, std::vector<float>& radii, float sa, float sb) {
  int   num_tries_alpha = 5;
  float current_alpha   = M_PI / 4;
  float step            = M_PI / 8;
  int   ptsize          = pts.size();

  float apmax, bpmax, aproj, bproj, size;
  float leftval, rightval;
  for (int i = 0; i < num_tries_alpha; i++) {
    for (int j = 0; j < 2; j++) {
      float test_alpha  = current_alpha + (j - 0.5) * 2 * step;
      float test_adir_x = cos(test_alpha);
      float test_adir_y = sin(test_alpha);

      float test_bdir_x = cos(test_alpha + M_PI / 2);
      float test_bdir_y = sin(test_alpha + M_PI / 2);
      apmax             = 0;
      bpmax             = 0;
      for (int k = 0; k < ptsize; k++) {
        aproj = abs(pts[k].x() * test_adir_x + pts[k].y() * test_adir_y + radii[k]);
        bproj = abs(pts[k].x() * test_bdir_x + pts[k].y() * test_bdir_y + radii[k]);
        if (aproj > apmax) {
          apmax = aproj;
        }
        if (bproj > bpmax) {
          bpmax = bproj;
        }
      }
      size = apmax * bpmax;
      if (j == 0) {
        leftval = size;
      } else {
        rightval = size;
      }
    }

    /* ROS_INFO("leftval: %f, rightval: %f", leftval, rightval); */
    if (rightval < leftval) {
      current_alpha = current_alpha + step;
    } else {
      current_alpha = current_alpha - step;
    }
    step = step / 2;
  }
  /* ROS_INFO("best alpha: %f", current_alpha); */

  return std::make_pair(current_alpha, 0);
}

void calculateBlockParamsForSegment(octomap::Segment* seg_ptr, std::vector<octomap::point3d>& deltapoints) {
  std::pair<float, float>       block_angles = search_planning::calculateBestFitAlphaBeta(deltapoints);
  std::vector<octomap::point3d> block_dirs   = search_planning::blockAnglesToDirections(block_angles.first, block_angles.second);
  float                         aproj, bproj, cproj;
  float                         block_a = 0;
  float                         block_b = 0;
  float                         block_c = 0;
  for (uint k = 0; k < deltapoints.size(); k++) {
    aproj = abs(deltapoints[k].dot(block_dirs[0]));
    bproj = abs(deltapoints[k].dot(block_dirs[1]));
    cproj = abs(deltapoints[k].dot(block_dirs[2]));
    if (aproj > block_a) {
      block_a = aproj;
    }
    if (bproj > block_b) {
      block_b = bproj;
    }
    if (cproj > block_c) {
      block_c = cproj;
    }
  }
  seg_ptr->block_alpha = block_angles.first;
  seg_ptr->block_beta  = block_angles.second;
  seg_ptr->block_a     = block_a + 0.8;
  seg_ptr->block_b     = block_b + 0.8;
  seg_ptr->block_c     = block_c + 0.8;
  seg_ptr->block_dirs  = block_dirs;  // cache this for faster calculation
}

void calculateBlockParamsForSegment(std::map<uint, SphereMapSegment>::iterator seg_ptr, std::vector<octomap::point3d>& deltapoints, std::vector<float>& radii) {
  std::pair<float, float>       block_angles = search_planning::calculateBestFitAlphaBeta(deltapoints, radii);
  std::vector<octomap::point3d> block_dirs   = search_planning::blockAnglesToDirections(block_angles.first, block_angles.second);
  float                         aproj, bproj, cproj;
  float                         block_a = 0;
  float                         block_b = 0;
  float                         block_c = 0;
  for (uint k = 0; k < deltapoints.size(); k++) {
    aproj = abs(deltapoints[k].dot(block_dirs[0]) + radii[k]);
    bproj = abs(deltapoints[k].dot(block_dirs[1]) + radii[k]);
    cproj = abs(deltapoints[k].dot(block_dirs[2]) + radii[k]);
    if (aproj > block_a) {
      block_a = aproj;
    }
    if (bproj > block_b) {
      block_b = bproj;
    }
    if (cproj > block_c) {
      block_c = cproj;
    }
  }

  seg_ptr->second.block_alpha = block_angles.first;
  seg_ptr->second.block_beta  = block_angles.second;
  seg_ptr->second.block_a     = block_a + 0.5;
  seg_ptr->second.block_b     = block_b + 0.5;
  seg_ptr->second.block_c     = block_c + 0.5;
  seg_ptr->second.block_dirs  = block_dirs;  // cache this for faster calculation
}

float getObstacleDist(octomap::point3d& test_point, std::shared_ptr<search_planning::PCLMap>& pclmap) {
  pcl::PointXYZ pcl_point;
  pcl_point.x = test_point.x();
  pcl_point.y = test_point.y();
  pcl_point.z = test_point.z();
  return pclmap->getDistanceFromNearestPoint(pcl_point);
}

bool arePointsMutuallyVisible2(octomap::point3d p1, octomap::point3d p2, std::shared_ptr<octomap::OcTree> occupancy_octree) {
  octomap::KeyRay ray;
  occupancy_octree->computeRayKeys(p1, p2, ray);

  for (octomap::KeyRay::iterator it = ray.begin(), end = ray.end(); it != end; ++it) {
    octomap::OcTreeNode* oc_node = occupancy_octree->search(*it);
    if (oc_node == NULL || occupancy_octree->isNodeOccupied(oc_node)) {
      return false;
    }
  }
  return true;
}

bool isNear(std::vector<octomap::point3d> points, octomap::point3d test_point, float maxdist) {
  float maxdist2 = pow(maxdist, 2);
  for (octomap::point3d point : points) {
    if ((test_point - point).norm_sq() < maxdist2) {
      return true;
    }
  }
  return false;
}

std::optional<float> getNearestPointDist(std::vector<octomap::point3d> points, octomap::point3d test_point, float maxdist) {
  bool  found_some = false;
  float found_dist2;

  float max_group_dist2 = pow(maxdist, 2);
  for (octomap::point3d point : points) {
    float dist2 = (test_point - point).norm_sq();
    if (dist2 < max_group_dist2 && (!found_some || found_dist2 < dist2)) {
      found_dist2 = dist2;
      found_some  = true;
    }
  }
  if (found_some) {
    return sqrt(found_dist2);
  }
  return std::nullopt;
}

geometry_msgs::Point octopoint2geometry(octomap::point3d pos) {
  geometry_msgs::Point pos2;
  pos2.x = pos.x();
  pos2.y = pos.y();
  pos2.z = pos.z();
  return pos2;
}

}  // namespace search_planning
