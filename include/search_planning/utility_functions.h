#pragma once
#ifndef SP_UTILS_H
#define SP_UTILS_H

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

#include <search_planning/mapping_structures.h>
#include <search_planning/spheremap.h>

namespace search_planning
{
class StopWatch {
public:
  void tick() {
    measuring          = true;
    measure_start_time = ros::WallTime::now();
  }
  void tock() {
    last_val  = (ros::WallTime::now() - measure_start_time).toNSec() / 1000.0;
    measuring = false;
    sum_val += last_val;
    if (last_val > max_val) {
      max_val = last_val;
    }
  }
  void clear() {
    sum_val   = 0;
    last_val  = 0;
    measuring = false;
    max_val   = 0;
  }
  float valSec() {
    return sum_val / 1000.0;
  }
  float maxValSec() {
    return max_val / 1000.0;
  }

  float         max_val   = 0;
  float         sum_val   = 0;
  float         last_val  = 0;
  bool          measuring = false;
  ros::WallTime measure_start_time;
};

std::vector<octomap::point3d> getPCA3D(std::vector<octomap::point3d> points);
tf2::Quaternion               rotationMatrixToQuaternion(std::vector<octomap::point3d> vectors);
octomap::point3d    castRayWithNullNodes(std::shared_ptr<octomap::OcTree> occupancy_octree_, octomap::point3d point, octomap::point3d end_point, bool* hit);
bool                isPointSafe(octomap::point3d center_point, float safe_dist, std::shared_ptr<octomap::OcTree> occupancy_octree_);
bool                makePointSafe(octomap::point3d* point_ptr, float safe_dist, int max_iters, std::shared_ptr<octomap::OcTree> occupancy_octree_,
                                  std::shared_ptr<octomap::SurfaceOcTree> surface_octree_);
tf2::Vector3        getRandomFOVRayEndpoint(octomap::point3d top_left_corner, bool is_optic_frame = true);
float               getHeadingFromVector(octomap::point3d normal);
bool                hasNodeUnknownChildren(octomap::OcTreeNode* node, std::shared_ptr<octomap::OcTree> octree, uint depth);
float               getExplorednessRelativeToCamera(float dist);  // Because the distance might sometimes be calculated before calling this function
std_msgs::ColorRGBA heatMapColor(double h, double a);
octomap::point3d    getRandomPointInSphere(float r);
octomap::point3d    getRandomPointInSphere(float min_r, float delta_r);
BoundingBox         getPointsBoundingBox(std::vector<octomap::point3d> points);
octomap::point3d    getSegmentColor(int segment_id);
float               getSegmentPathEulerDistance(octomap::SegmentPath path);
std::vector<pcl::PointXYZ> octomapToPointcloud(std::shared_ptr<octomap::OcTree> occupancy_octree_, BoundingBox bbx);
std::vector<pcl::PointXYZ> octomapToPointcloud(std::shared_ptr<octomap::OcTree> occupancy_octree_);
float                      calculatePointFrontierExplorationValue(octomap::point3d point, int num_raycasts, std::shared_ptr<octomap::OcTree> occupancy_octree_);

void publishPointsMarkers(std::vector<octomap::point3d> points, std::vector<int> colors, std::string markers_name, float size,
                          ros::Publisher* pub_octomap_markers_, std::string map_frame_);
void publishMarkers(std::vector<visualization_msgs::Marker> markers, ros::Publisher* publisher, std::string markers_name, std::string map_frame,
                    int start_id = 0, float lifetime = 2);
void publishMarkers(std::shared_ptr<std::vector<visualization_msgs::Marker>> markers, ros::Publisher* publisher, std::string markers_name,
                    std::string map_frame, int start_id = 0, float lifetime = 2);

/* BASIC MARKERS */
visualization_msgs::Marker getMarkerArrow(octomap::point3d pos, octomap::point3d dir, octomap::point3d color = octomap::point3d(0, 1, 0),
                                          float shaft_width = 0.2, float head_width = 0.4, float head_length = 0.1);
visualization_msgs::Marker getMarkerSphere(octomap::point3d pos, float d, octomap::point3d color = octomap::point3d(0, 1, 0), float alpha = 1);
visualization_msgs::Marker getMarkerCube(octomap::point3d pos, float d, octomap::point3d color = octomap::point3d(0, 1, 0));
visualization_msgs::Marker getMarkerLine(octomap::point3d pos1, octomap::point3d pos2, octomap::point3d color = octomap::point3d(0, 1, 0),
                                         float linewidth = 0.1);
visualization_msgs::Marker getMarkerText(octomap::point3d pos1, std::string text, octomap::point3d color = octomap::point3d(0, 1, 0), float size = 5);
std::vector<visualization_msgs::Marker> getPointsMarkers(std::vector<pcl::PointXYZ> points, octomap::point3d color = octomap::point3d(1, 0, 0),
                                                         float size = 0.2);
std::vector<visualization_msgs::Marker> getSphereMapPathsMarkers(std::vector<SphereMapPath> paths, octomap::point3d path_color = octomap::point3d(1, 0, 0),
                                                         float path_line_width = 0.2);

/* SEGMAP MARKERS */
std::vector<visualization_msgs::Marker>                  getSegmapDetailedMarkers(std::shared_ptr<SegMap> segmap_, bool is_received_segmap = false);
std::vector<visualization_msgs::Marker>                  getSegmapCoverageMarkers(std::shared_ptr<SegMap> segmap_);
std::vector<visualization_msgs::Marker>                  getSegmapBlockMarkers(std::shared_ptr<SegMap> segmap_, bool is_received_segmap = false);
std::vector<visualization_msgs::Marker>                  getSegmapReceivedMarkers(std::shared_ptr<SegMap> segmap_);
std::shared_ptr<std::vector<visualization_msgs::Marker>> getSegmentOctreeMarkers(std::shared_ptr<octomap::SegmentOcTree> segment_octree_);
std::shared_ptr<std::vector<visualization_msgs::Marker>> getSegmentOctreeMarkersInside(std::shared_ptr<octomap::SegmentOcTree> segment_octree_);

/* SPHEREMAP MARKERS */
std::vector<visualization_msgs::Marker> getSpheremapDebugMarkers(std::shared_ptr<SphereMap> spheremap_);
std::vector<visualization_msgs::Marker> getSpheremapMarkers(octomap::point3d center, float box_halfsize, std::shared_ptr<SphereMap> spheremap_);
std::vector<visualization_msgs::Marker> getSpheremapPointMarkers(octomap::point3d center, float box_halfsize, std::shared_ptr<SphereMap> spheremap_,
                                                                 bool is_visited_positions_map = false, float node_maxval = 0, float val_decrease_dist = 0,
                                                                 float blocking_dist = 0);
std::vector<visualization_msgs::Marker> getSpheremapSegmentMarkers(std::shared_ptr<SphereMap> spheremap_);
std::vector<visualization_msgs::Marker> getSpheremapNavigationMarkers(std::shared_ptr<SphereMap> spheremap_);

/* FACETMAP MARKERS */
std::vector<visualization_msgs::Marker> getFacetMapMarkersFull(octomap::point3d center, float box_halflength,
                                                               std::shared_ptr<octomap::SurfaceOcTree> surface_octree_);
std::vector<visualization_msgs::Marker> getFacetMapMarkersPoints(octomap::point3d center, float box_halflength,
                                                                 std::shared_ptr<octomap::SurfaceOcTree> surface_octree_);

std::optional<FrontierExplorationPoint> getFrontierExplorationData(octomap::point3d pos, int num_rays, float max_ray_dist,
                                                                   std::shared_ptr<octomap::OcTree> occupancy_octree_, bool only_look_down = false);
int  getNearestSegmentRaycasting(octomap::point3d pos, int num_rays, float max_dist, std::shared_ptr<octomap::OcTree> occupancy_octree_,
                                 std::shared_ptr<octomap::SegmentOcTree> seg_octree_);
int  getNearestSegmentRaycasting2(octomap::point3d pos, int num_rays, float max_dist, std::shared_ptr<octomap::OcTree> occupancy_octree_,
                                  std::shared_ptr<octomap::SegmentOcTree> seg_octree_);
bool arePointsMutuallyVisible(octomap::point3d p1, octomap::point3d p2, std::shared_ptr<octomap::OcTree> occupancy_octree_);
void filterPoints(std::vector<octomap::point3d>* in, std::vector<octomap::point3d>* out, float filter_dist);
std::vector<octomap::point3d> getCylinderSamplingPoints(int num_points_circle, float delta_r, float delta_z, int num_circles, int num_layers);

std::vector<octomap::point3d> blockAnglesToDirections(float alpha, float beta);
std::pair<float, float>       directionsToBlockAngles(std::vector<octomap::point3d> p);

std::pair<float, float> calculateBestFitAlphaBeta(std::vector<octomap::point3d>& pts, float startalpha = 0, float startbeta = 0);
std::pair<float, float> calculateBestFitAlphaBeta(std::vector<octomap::point3d>& pts, std::vector<float>& radii, float startalpha = 0, float startbeta = 0);

void calculateBlockParamsForSegment(octomap::Segment* seg_ptr, std::vector<octomap::point3d>& deltapoints);
void calculateBlockParamsForSegment(std::map<uint, SphereMapSegment>::iterator seg_ptr, std::vector<octomap::point3d>& pts, std::vector<float>& radii);

std::vector<visualization_msgs::Marker> getSegmapNavGraphMarkers(std::shared_ptr<SegMap> segmap_);
float                                   getObstacleDist(octomap::point3d& test_point, std::shared_ptr<PCLMap>& pclmap);

bool                 arePointsMutuallyVisible2(octomap::point3d p1, octomap::point3d p2, std::shared_ptr<octomap::OcTree> occupancy_octree);
bool                 isNear(std::vector<octomap::point3d> points, octomap::point3d, float maxdist);
std::optional<float> getNearestPointDist(std::vector<octomap::point3d> points, octomap::point3d, float maxdist);
geometry_msgs::Point octopoint2geometry(octomap::point3d);
}  // namespace search_planning

#endif
