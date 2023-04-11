#include <spheremap_server/mapping_nodelet.h>
/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>


namespace spheremap_server
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

}  // namespace spheremap_server
/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(spheremap_server::MappingNodelet, nodelet::Nodelet);
