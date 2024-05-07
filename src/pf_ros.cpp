#include <dt_ndt_mcl/pf_ros.hpp>

ParticleFilter2D::ParticleFilter2D(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : m_nh(nh), m_prv_nh(pnh), m_tf_listener(m_tf_buffer) {
  m_scan_matcher.initialize("ndt", nh, 100.0);
  m_map_sub =
      m_prv_nh.subscribe("/map", 1, &ParticleFilter2D::mapCallback, this);

  m_pose_particle_pub =
      m_nh.advertise<geometry_msgs::PoseArray>("/particlecloudz", 1);

  m_init_pose_sub = m_prv_nh.subscribe(
      "/initialpose", 1, &ParticleFilter2D::initPoseCallback, this);

  m_scan_sub = m_prv_nh.subscribe("/scan_front", 1,
                                  &ParticleFilter2D::scanCallback, this);

  m_received_map = false;
  m_received_init_pose = false;
  // TODO: Add motion model alpha initialization parameters
  m_motion_model =
      std::make_shared<ndt_2d::MotionModel>(0.2, 0.2, 0.2, 0.2, 0.2);

  size_t min_particles = 100;
  size_t max_particles = 1000;
  m_kld_err = 0.01;
  m_kld_z = 0.99;

  m_pf = std::make_shared<ndt_2d::ParticleFilter>(min_particles, max_particles,
                                                  m_motion_model);

  m_min_travel_distance = 0.1;
  m_min_travel_rotation = 0.5;
}

void ParticleFilter2D::mapCallback(
    const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  m_scan_matcher.addMap(*msg);
  std::cout << "Map received" << std::endl;
  m_received_map = true;
}

void ParticleFilter2D::initPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  std::cout << "received initial pose!\n";
  m_pf->init(msg->pose.pose.position.x, msg->pose.pose.position.y,
             tf2::getYaw(msg->pose.pose.orientation),
             sqrt(msg->pose.covariance[0]), sqrt(msg->pose.covariance[7]),
             sqrt(msg->pose.covariance[35]));

  geometry_msgs::PoseArray pose_msg;
  pose_msg.header.frame_id = "map";
  m_pf->getMsg(pose_msg);
  m_pose_particle_pub.publish(pose_msg);

  geometry_msgs::TransformStamped tf_odom_pose;
  try {
    tf_odom_pose =
        m_tf_buffer.lookupTransform("odom", "base_footprint", ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  ndt_2d::Pose2d odom_pose = ndt_2d::fromMsg(tf_odom_pose);
  m_prev_robot_pose = ndt_2d::fromMsg(msg->pose.pose);
  m_prev_odom_pose = odom_pose;

  m_received_init_pose = true;
}

void ParticleFilter2D::scanCallback(
    const sensor_msgs::LaserScan::ConstPtr& msg) {
  if (m_received_map == false) {
    ROS_ERROR("No map received yet, ignoring scan");
    return;
  }
  if (m_received_init_pose == false) {
    ROS_ERROR("No initial pose received yet, ignoring scan");
    return;
  }
  // get the robots current pose in odom frame
  geometry_msgs::TransformStamped tf_odom_pose;
  try {
    tf_odom_pose =
        m_tf_buffer.lookupTransform("odom", "base_footprint", ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  ndt_2d::Pose2d odom_pose = ndt_2d::fromMsg(tf_odom_pose);
  ndt_2d::Pose2d robot_pose;

  // ensure that enough distance has been travelled
  // Calculate delta in odometry frame
  double dx = odom_pose.x - m_prev_odom_pose.x;
  double dy = odom_pose.y - m_prev_odom_pose.y;
  double dth = angles::shortest_angular_distance(m_prev_odom_pose.theta,
                                                 odom_pose.theta);
  double dist = (dx * dx) + (dy * dy);
  if (dist < m_min_travel_distance * m_min_travel_distance &&
      std::fabs(dth) < m_min_travel_rotation) {
    return;
  }

  // Odometry frame is usually not aligned with map frame
  double heading = angles::shortest_angular_distance(m_prev_odom_pose.theta,
                                                     m_prev_robot_pose.theta);

  // Now apply odometry delta, corrected by heading, to get initial corrected
  // pose
  robot_pose.x = prev_robot_pose_.x + (dx * cos(heading)) - (dy * sin(heading));
  robot_pose.y = prev_robot_pose_.y + (dx * sin(heading)) + (dy * cos(heading));
  robot_pose.theta = angles::normalize_angle(prev_robot_pose_.theta + dth);
}