#include <dt_ndt_mcl/pf_ros.hpp>

ParticleFilter2D::ParticleFilter2D(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : m_nh(nh), m_prv_nh(pnh) {
  m_scan_matcher.initialize("ndt", nh, 100.0);
  m_map_sub =
      m_prv_nh.subscribe("/map", 1, &ParticleFilter2D::mapCallback, this);

  m_received_map = false;
}

void ParticleFilter2D::mapCallback(
    const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  m_scan_matcher.addMap(*msg);
  std::cout<<"Map received"<<std::endl;
  m_received_map = true;
}