#pragma once
#ifndef MAPPING_NODELET_H
#define MAPPING_NODELET_H

/* each ros package must have these */
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

#include <spheremap_server/mapper.h>

namespace spheremap_server
{


class MappingNodelet : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  bool             is_initialized_        = false;

  ExplorationMapper * mapper_;
};

}  // namespace spheremap_server

#endif
