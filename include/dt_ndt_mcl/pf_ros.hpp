#ifndef PF_ROS_HPP_
#define PF_ROS_HPP_

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <dt_ndt_mcl/particle_filter.hpp>
#include <dt_ndt_mcl/scan_matcher_ndt.hpp>
#include <iostream>

class ParticleFilter2D {
 public:
  ParticleFilter2D(ros::NodeHandle &nh, ros::NodeHandle &pnh);

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

 private:
  ndt_2d::ScanMatcherNDT m_scan_matcher;

  ros::NodeHandle &m_nh;
  ros::NodeHandle &m_prv_nh;
  ros::Subscriber m_map_sub;

  bool m_received_map;
};

#endif  // PF_ROS_HPP_