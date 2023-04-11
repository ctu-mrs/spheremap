#ifndef __PCL_MAP_H__
#define __PCL_MAP_H__

#include <vector>
#include <string.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

namespace spheremap_server
{

/**
 * @brief Class PCLMap provides the loading of the map and obstacles using Point Cloud Library (PCL)
 */
class PCLMap {
public:
  /**
   * @brief constructor
   */
  PCLMap(void);

  /**
   * @brief destructor
   */
  ~PCLMap(void);

  /**
   * @brief loads the map from PCD file and sets its desired resolution
   */
  void loadMap(const std::string &name, double resolution);

  /**
   * @brief finds distance to the nearest point in radius
   *
   * @param x
   * @param y
   * @param z
   * @param search_radius
   * @return float squared distance to the point, if there is no point returns -1
   */
  float radiusSearch(const double x, const double y, const double z, const double search_radius);

  /**
   * @brief finds distance to the nearest point in radius
   *
   * @param x
   * @param y
   * @param z
   * @param search_radius
   * @param max_floor_height - maximum value of z which is supposed to be point in floor
   * @param allowed_angle_diff - allowed angle diff from horizontal plane to simulate cyllinder
   * @return float squared distance to the point, if there is no point returns -1
   */
  float simulatedCyllinderSearch(const double x, const double y, const double z, const double search_radius, const double max_floor_height,
                                 const double allowed_angle_diff);

  pcl::PointCloud<pcl::PointXYZ>::Ptr getPCLCloud();
  void                                initKDTreeSearch(pcl::PointCloud<pcl::PointXYZ>::Ptr points);
  double                              getDistanceFromNearestPoint(pcl::PointXYZ point);
  void                                insertPoint(pcl::PointXYZ point);
  void                                initCloud();

  bool checkDistanceFromNearestPoint(pcl::PointXYZ point, double safe_dist_xy, double safe_dist_z);

  static pcl::PointCloud<pcl::PointXYZ>::Ptr pclVectorToPointcloud(const std::vector<pcl::PointXYZ> &points);

  static pcl::PointCloud<pcl::PointXYZ>::Ptr octomapToPointcloud(std::shared_ptr<octomap::OcTree> input_octree, std::array<octomap::point3d, 2> map_limits);

private:
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr                     pcl_cloud;
  /* pcl::search::KdTree<pcl::PointXYZ>::Ptr                 kdtree; */
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree;
  bool                                 kd_tree_initialized = false;
  /* pcl::octree::OctreePointCloud */
};

}  // namespace darpa_planning

#endif
