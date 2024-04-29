#ifndef PF_ROS_HPP_
#define PF_ROS_HPP_

#include <ros/ros.h>

#include <dt_ndt_mcl/scan_matcher_ndt.hpp>

class ParticleFilter2D {
 public:
  ParticleFilter2D(ros::NodeHandle &nh, ros::NodeHandle &pnh);

 private:
  ndt_2d::ScanMatcherNDT m_scan_matcher;
  ros::NodeHandle &m_nh;
  ros::NodeHandle &m_prv_nh;
};

#endif  // PF_ROS_HPP_