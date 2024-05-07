#ifndef PF_ROS_HPP_
#define PF_ROS_HPP_

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <dt_ndt_mcl/conversions.hpp>
#include <dt_ndt_mcl/motion_model.hpp>
#include <dt_ndt_mcl/particle_filter.hpp>
#include <dt_ndt_mcl/scan_matcher_ndt.hpp>
#include <iostream>

class ParticleFilter2D {
 public:
  ParticleFilter2D(ros::NodeHandle &nh, ros::NodeHandle &pnh);

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void initPoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

 private:
  ndt_2d::ScanMatcherNDT m_scan_matcher;
  ndt_2d::MotionModelPtr m_motion_model;
  std::shared_ptr<ndt_2d::ParticleFilter> m_pf;

  tf2_ros::Buffer m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;

  ros::NodeHandle &m_nh;
  ros::NodeHandle &m_prv_nh;
  ros::Subscriber m_map_sub;
  ros::Subscriber m_init_pose_sub;
  ros::Subscriber m_scan_sub;
  ros::Publisher m_pose_particle_pub;

  bool m_received_map;
  bool m_received_init_pose;
  double m_kld_err;
  double m_kld_z;
};

#endif  // PF_ROS_HPP_