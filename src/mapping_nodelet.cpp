#include <search_planning/mapping_nodelet.h>
/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>


namespace search_planning
{

/* onInit() //{ */
void MappingNodelet::onInit() {
  ros::NodeHandle      nh("~");

  ros::Time::waitForValid();
  mapper_ = new ExplorationMapper();

  mapper_->initialize(&nh);

  ROS_INFO("mapping node init done");
  is_initialized_ = true;
}
//}

}  // namespace search_planning
/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(search_planning::MappingNodelet, nodelet::Nodelet);
