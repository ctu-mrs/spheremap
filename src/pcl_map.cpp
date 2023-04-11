#include <ros/ros.h>
#include <spheremap_server/pcl_map.h>

using namespace spheremap_server;

#define RESOLUTION 0.1f

PCLMap::PCLMap(void) : octree(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(RESOLUTION)) {
  kdtree = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>);
}

PCLMap::~PCLMap(void) {
}

float PCLMap::radiusSearch(const double x, const double y, const double z, const double search_radius) {
  const pcl::PointXYZ point(x, y, z);
  /* ROS_WARN("[%s]: Calling radius search, point [%.2f, %.2f, %.2f], search_radius = %.2f", ros::this_node::getName().c_str(), x, y, z, search_radius); */
  std::vector<int>   k_indices;
  std::vector<float> k_sqr_distances;
  /* float result; */
  octree->radiusSearch(point, search_radius, k_indices, k_sqr_distances, 25);
  if (k_sqr_distances.size() > 0) {
    /* ROS_INFO("[%s]: Point found, size = %lu", ros::this_node::getName().c_str(), k_sqr_distances.size()); */
    /* result = *std::min_element(k_sqr_distances.begin(), k_sqr_distances.end()); */
    /* ROS_INFO("[%s]: Smallest dist = %.2f", ros::this_node::getName().c_str(), result); */
    return sqrt(*std::min_element(k_sqr_distances.begin(), k_sqr_distances.end()));
  }
  /* k_indices.resize(1); */
  /* k_sqr_distances.resize(1); */
  /* if (octree->nearestKSearch(point, 1, k_indices, k_sqr_distances) > -1) { */
  /*   /1* ROS_WARN("[%s]: result = %.2f", ros::this_node::getName().c_str(), sqrt(k_sqr_distances[0])); *1/ */
  /*   if (sqrt(k_sqr_distances[0]) < search_radius) { */
  /*     /1* ROS_WARN("[%s]: returning result = %.2f", ros::this_node::getName().c_str(), sqrt(k_sqr_distances[0])); *1/ */
  /*     return sqrt(k_sqr_distances[0]); */
  /*   } */
  /* } */
  /* ROS_WARN("[%s]: Returning -1", ros::this_node::getName().c_str()); */
  return -1;
}

void PCLMap::insertPoint(pcl::PointXYZ point) {
  pcl_cloud->push_back(point);
  octree->setInputCloud(pcl_cloud);
  octree->addPointsFromInputCloud();
}

float PCLMap::simulatedCyllinderSearch(const double x, const double y, const double z, const double search_radius, const double max_floor_height,
                                       const double allowed_angle_diff) {
  std::vector<int>   k_indices;
  std::vector<float> k_sqr_distances;
  double             z_space  = 2 * search_radius * sin(allowed_angle_diff);
  double             z_actual = z;
  while (z_actual > max_floor_height) {
    const pcl::PointXYZ point(x, y, z_actual);
    if (octree->nearestKSearch(point, 1, k_indices, k_sqr_distances) > 0) {
      for (unsigned int i = 0; i < k_indices.size(); i++) {
        if (pcl_cloud->points[k_indices[i]].z > max_floor_height && pcl_cloud->points[k_indices[i]].z < z) {
          return 1;
        }
      }
    }
    z_actual -= z_space;
  }
  return -1;
}

void PCLMap::initCloud() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_cloud = cloud;
}

void PCLMap::loadMap(const std::string &filepath, double resolution) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath.c_str(), *cloud) == -1)  // load the file
  {
    ROS_ERROR("Couldn't read file %s\n", filepath.c_str());
    return;
  }

  ROS_INFO_STREAM("Loaded " << cloud->width * cloud->height << " data points from " << filepath);
  octree->deleteTree();
  // Initialize octree-based point cloud change detection class
  octree->setInputCloud(cloud);
  // Add points from cloud to octree
  octree->addPointsFromInputCloud();
  /* octree->setResolution(0.1); */
  ROS_INFO("[%s]: Tree depth = %d for resolution %.2f", ros::this_node::getName().c_str(), octree->getTreeDepth(), resolution);
  pcl_cloud = cloud;
  ROS_INFO("[%s]: Testing distance from [3.9, 20.6, 8.70] = %.2f", ros::this_node::getName().c_str(), radiusSearch(4.8, 17.9, 8.7, 85.0));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCLMap::getPCLCloud() {
  return pcl_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCLMap::pclVectorToPointcloud(const std::vector<pcl::PointXYZ> &points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr data(new pcl::PointCloud<pcl::PointXYZ>());
  data->height = 1;
  data->width  = points.size();
  /* ROS_DEBUG("[POINTXYZ VECTOR TO POINTCLOUD] width, height: (%d, %d)", data->width, data->height); */
  data->is_dense = false;
  /* data->resize(data->width); \\ Uncommenting this leads to a bug -> pcl adds data->width number of points at the (0, 0, 0) */
  for (pcl::PointXYZ point : points) {
    data->push_back(point);
  }
  return data;
}

void PCLMap::initKDTreeSearch(pcl::PointCloud<pcl::PointXYZ>::Ptr points) {
  /* pcl::PointCloud<pcl::PointXYZ>::Ptr zero_points; */
  /* kdtree->setInputCloud(zero_points); */
  /* kdtree->reset(); */
  pcl_cloud = points; // FIXME: unnecessarry if not using check distance from nearest point

  if (points->size() > 0) {
    /* ROS_INFO("[%s]: initkdtree, point size = %lu", ros::this_node::getName().c_str(), points->size()); */
    kdtree->setInputCloud(points->makeShared());
    /* ROS_INFO("[%s]: kdtree function end", ros::this_node::getName().c_str()); */
    kd_tree_initialized = true;
  }
  ROS_INFO("[%s]: init kd tree search end", ros::this_node::getName().c_str());
}

double PCLMap::getDistanceFromNearestPoint(pcl::PointXYZ point) {
  std::vector<int>   indices(1);
  std::vector<float> sqr_distances(1);

  if (kd_tree_initialized && kdtree->nearestKSearch(point, 1, indices, sqr_distances) > 0) {
    /* ROS_INFO("[%s]: Nearest point search: returning %.2f", ros::this_node::getName().c_str(), sqrt(sqr_distances[0])); */
    return sqrt(sqr_distances[0]);
  } else {
    return FLT_MAX;
  }
}

bool PCLMap::checkDistanceFromNearestPoint(pcl::PointXYZ point, double safe_dist_xy, double safe_dist_z) {
  std::vector<int>   indices(100);
  std::vector<float> sqr_distances(100);
  bool res = false;
  if (kd_tree_initialized) {
    int n_found = kdtree->radiusSearch(point, safe_dist_xy, indices, sqr_distances, 100);
    if (n_found > 0 && sqrt(sqr_distances[0]) > safe_dist_z && sqrt(sqr_distances[0]) < safe_dist_xy) {
      for (int k = 0; k < n_found; k++) {
        if (abs(point.z - pcl_cloud->points[indices[k]].z) < safe_dist_z) {
          return false;
        }
      }
      return true;
    }
    /* ROS_INFO("[%s]: Nearest point search: returning %.2f", ros::this_node::getName().c_str(), sqrt(sqr_distances[0])); */
    return false;
  } else {
    return false;
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCLMap::octomapToPointcloud(std::shared_ptr<octomap::OcTree> input_octree, std::array<octomap::point3d, 2> map_limits) {
  std::vector<pcl::PointXYZ> output_pcl;

  if (map_limits[0].x() > map_limits[1].x() || map_limits[0].y() > map_limits[1].y() || map_limits[0].z() > map_limits[1].z()) { 
    ROS_ERROR("[PCL map]: Octomap cannot be converted. Provided map limits cannot be used for definition of bounding box.");
    return nullptr;
  }

  if (!input_octree) { 
    ROS_ERROR("[PCL map]: Octomap cannot be converted. Empty input octree received."); // FIXME add retunr
  }

  octomap::OcTreeKey min_key = input_octree->coordToKey(map_limits[0]);
  octomap::OcTreeKey max_key = input_octree->coordToKey(map_limits[1]);
  for (int x = min_key.k[0]; x <= max_key.k[0]; x++) {
    for (int y = min_key.k[1]; y <= max_key.k[1]; y++) {
      for (int z = min_key.k[2]; z <= max_key.k[2]; z++) {
        pcl::PointXYZ      point;
        octomap::OcTreeKey tmp_key;
        tmp_key.k[0] = x;
        tmp_key.k[1] = y;
        tmp_key.k[2] = z;
        if (input_octree->search(tmp_key) == NULL ||
            (input_octree->search(tmp_key) != NULL && input_octree->isNodeOccupied(input_octree->search(tmp_key)))) {
          octomap::point3d octomap_point = input_octree->keyToCoord(tmp_key);
          point.x                        = octomap_point.x();
          point.y                        = octomap_point.y();
          point.z                        = octomap_point.z();
          output_pcl.push_back(point);
        }
      }
    }
  }

  if (output_pcl.size() > 0) {
    return pclVectorToPointcloud(output_pcl);
  }

  ROS_ERROR("[PCL map]: Octomap cannot be converted empty pointcloud received.");
  return nullptr;
}
