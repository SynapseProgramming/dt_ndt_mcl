#ifndef PF_ROS_HPP_
#define PF_ROS_HPP_

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <dt_ndt_mcl/motion_model.hpp>
#include <dt_ndt_mcl/particle_filter.hpp>
#include <dt_ndt_mcl/scan_matcher_ndt.hpp>
#include <iostream>

class ParticleFilter2D {
 public:
  ParticleFilter2D(ros::NodeHandle &nh, ros::NodeHandle &pnh);

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

 private:
  ndt_2d::ScanMatcherNDT m_scan_matcher;
  ndt_2d::MotionModelPtr m_motion_model;
  std::shared_ptr<ndt_2d::ParticleFilter> m_pf;

  ros::NodeHandle &m_nh;
  ros::NodeHandle &m_prv_nh;
  ros::Subscriber m_map_sub;

  bool m_received_map; 
  double m_kld_err;
  double m_kld_z;
};

#endif  // PF_ROS_HPP_