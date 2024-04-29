#include <dt_ndt_mcl/pf_ros.hpp>

ParticleFilter2D::ParticleFilter2D(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : m_nh(nh), m_prv_nh(pnh) {
  m_scan_matcher.initialize("ndt", nh, 100.0);
}
