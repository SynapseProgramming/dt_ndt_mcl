#include <dt_ndt_mcl/pf_ros.hpp>

ParticleFilter2D::ParticleFilter2D(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : m_nh(nh), m_prv_nh(pnh) {
  m_scan_matcher.initialize("ndt", nh, 100.0);
  m_map_sub =
      m_prv_nh.subscribe("/map", 1, &ParticleFilter2D::mapCallback, this);

  m_pose_particle_pub =
      m_nh.advertise<geometry_msgs::PoseArray>("/particlecloudz", 1);

  m_init_pose_sub = m_prv_nh.subscribe(
      "/initialpose", 1, &ParticleFilter2D::initPoseCallback, this);

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
  m_received_init_pose = true;
}