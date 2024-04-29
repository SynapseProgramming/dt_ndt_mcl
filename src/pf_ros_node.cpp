#include <ros/ros.h>

#include <dt_ndt_mcl/pf_ros.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pf_ros_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ParticleFilter2D pf(nh, pnh);

  ros::spin();

  return 0;
}