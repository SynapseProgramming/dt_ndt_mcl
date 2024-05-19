#include <dt_ndt_mcl/pf_ros.hpp>

ParticleFilter2D::ParticleFilter2D() : Node("dt_ndt_mcl_node")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
  auto durability_qos = rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile();

  m_pose_particle_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/particlecloudz", 1);
  m_best_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/best_pose", 1);
  m_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", qos, std::bind(&ParticleFilter2D::mapCallback, this, std::placeholders::_1));
  m_init_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", durability_qos, std::bind(&ParticleFilter2D::initPoseCallback, this, std::placeholders::_1));
  m_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", durability_qos, std::bind(&ParticleFilter2D::scanCallback, this, std::placeholders::_1));

  m_tf_buffer =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

  // TODO: Add reconfigurable parameters

  m_scan_matcher_ptr = std::make_shared<ndt_2d::ScanMatcherNDT>();
  m_scan_matcher_ptr->initialize("ndt", this, 100.0);

  m_motion_model =
      std::make_shared<ndt_2d::MotionModel>(0.1, 0.1, 0.1, 0.1, 0.1);

  m_received_map = false;
  m_received_init_pose = false;

  m_kld_err = 0.05;
  m_kld_z = 0.99;

  size_t min_particles = 500;
  size_t max_particles = 2000;
  m_min_travel_distance = 0.1;
  m_min_travel_rotation = 0.5;

  m_scan_id = 0;
  m_pf = std::make_shared<ndt_2d::ParticleFilter>(min_particles, max_particles,
                                                  m_motion_model);
}

void ParticleFilter2D::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{

  m_scan_matcher_ptr->addMap(*msg);
  RCLCPP_INFO(this->get_logger(), "Map received");
  m_received_map = true;
}

void ParticleFilter2D::initPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{

  std::cout << "received initial pose!\n";
  m_pf->init(msg->pose.pose.position.x, msg->pose.pose.position.y,
             tf2::getYaw(msg->pose.pose.orientation),
             sqrt(msg->pose.covariance[0]), sqrt(msg->pose.covariance[7]),
             sqrt(msg->pose.covariance[35]));

  geometry_msgs::msg::PoseArray pose_msg;
  pose_msg.header.frame_id = "map";
  m_pf->getMsg(pose_msg);
  m_pose_particle_pub->publish(pose_msg);

  geometry_msgs::msg::TransformStamped tf_odom_pose;
  std::string odomFrame = "odom";
  std::string baseFrame = "base_footprint";

  try
  {
    tf_odom_pose =
        m_tf_buffer->lookupTransform(odomFrame, baseFrame, tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        baseFrame.c_str(), odomFrame.c_str(), ex.what());
    return;
  }

  ndt_2d::Pose2d odom_pose = ndt_2d::fromMsg(tf_odom_pose);
  m_prev_robot_pose = ndt_2d::fromMsg(msg->pose.pose);
  m_prev_odom_pose = odom_pose;

  m_received_init_pose = true;
}

void ParticleFilter2D::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (m_received_map == false)
  {
    RCLCPP_ERROR(this->get_logger(), "No map received yet, ignoring scan");
    return;
  }
  if (m_received_init_pose == false)
  {
    RCLCPP_ERROR(this->get_logger(), "No initial pose received yet, ignoring scan");
    return;
  }
  // get the robots current pose in odom frame
  geometry_msgs::msg::TransformStamped tf_odom_pose;
  try
  {
    tf_odom_pose =
        m_tf_buffer->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_INFO(
        this->get_logger(), "Could not transform: %s", ex.what());
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
      std::fabs(dth) < m_min_travel_rotation)
  {
    return;
  }

  // Odometry frame is usually not aligned with map frame
  double heading = angles::shortest_angular_distance(m_prev_odom_pose.theta,
                                                     m_prev_robot_pose.theta);

  // Now apply odometry delta, corrected by heading, to get initial corrected
  // pose
  robot_pose.x =
      m_prev_robot_pose.x + (dx * cos(heading)) - (dy * sin(heading));
  robot_pose.y =
      m_prev_robot_pose.y + (dx * sin(heading)) + (dy * cos(heading));
  robot_pose.theta = angles::normalize_angle(m_prev_robot_pose.theta + dth);

  // Add in laserscan points
  ndt_2d::ScanPtr scan = std::make_shared<ndt_2d::Scan>(m_scan_id);
  m_scan_id++;
  scan->setPose(robot_pose);
  std::vector<ndt_2d::Point> points;
  points.reserve(msg->ranges.size());

  for (int i = 0; i < (int)msg->ranges.size(); i++)
  {
    if (std::isnan(msg->ranges[i]) || msg->ranges[i] <= msg->range_min ||
        msg->ranges[i] >= msg->range_max)
      continue;
    double d = (double)i * msg->angle_increment + msg->angle_min;
    double xx = msg->ranges[i] * cos(d);
    double yy = msg->ranges[i] * sin(d);
    // translate laser points to be wrt to base_footprint frame
    geometry_msgs::msg::PointStamped point_in;
    geometry_msgs::msg::PointStamped point_out;
    // TODO: change this to be a reconfigurable parameter
    point_in.header.frame_id = "base_laser";
    point_in.point.x = xx;
    point_in.point.y = yy;
    try
    {
      // TODO: also change this to be a reconfigurable parameter
      point_out = m_tf_buffer->transform(point_in, "base_footprint");
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_INFO(
          this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }
    ndt_2d::Point p(point_out.point.x, point_out.point.y);
    points.push_back(p);
  }
  scan->setPoints(points);

  // Extract change in position in map frame
  Eigen::Vector3d map_delta(scan->getPose().x - m_prev_robot_pose.x,
                            scan->getPose().y - m_prev_robot_pose.y, 1.0);

  // Transform change in pose into robot-centric frame
  Eigen::Isometry3d transform(
      Eigen::Translation3d(0.0, 0.0, 0.0) *
      Eigen::AngleAxisd(m_prev_robot_pose.theta, Eigen::Vector3d::UnitZ()));

  Eigen::Vector3d robot_delta = transform.inverse() * map_delta;

  // Compute change in heading
  robot_delta(2) = angles::shortest_angular_distance(m_prev_robot_pose.theta,
                                                     scan->getPose().theta);

  RCLCPP_INFO(this->get_logger(), "Updating filter with control %f %f %f", robot_delta(0),
              robot_delta(1), robot_delta(2));
  // previous pose theta print
  RCLCPP_INFO(this->get_logger(), "Previous pose theta: %f", m_prev_odom_pose.theta);

  m_pf->update(dx, dy, dth, m_prev_robot_pose);
  m_pf->measure(m_scan_matcher_ptr, scan);
  m_pf->resample(m_kld_err, m_kld_z);
  auto mean = m_pf->getMean();
  ndt_2d::Pose2d mean_pose(mean(0), mean(1), mean(2));
  scan->setPose(mean_pose);
  geometry_msgs::msg::PoseArray pose_msg;
  pose_msg.header.frame_id = "map";
  m_pf->getMsg(pose_msg);
  m_pose_particle_pub->publish(pose_msg);

  m_prev_robot_pose = scan->getPose();
  m_prev_odom_pose = odom_pose;

  // publish best pose
  geometry_msgs::msg::PoseStamped best_pose_msg;
  best_pose_msg.header.frame_id = "map";
  best_pose_msg.pose.position.x = mean(0);
  best_pose_msg.pose.position.y = mean(1);
  best_pose_msg.pose.orientation.z = std::sin(mean(2) / 2.0);
  best_pose_msg.pose.orientation.w = std::cos(mean(2) / 2.0);
  m_best_pose_pub->publish(best_pose_msg);

  double current_trace = computeTrace();
  if (current_trace < 0.02)
  {
    RCLCPP_INFO(this->get_logger(), "updating local map!");
    m_scan_matcher_ptr->updateLocalMap(scan);
  }
}

double ParticleFilter2D::computeTrace()
{
  Eigen::Matrix3d cov_matrix = m_pf->getCovariance();
  double trace = 0.0;
  for (int i = 0; i < 3; i++)
  {
    trace += cov_matrix(i, i);
  }
  return trace;
}