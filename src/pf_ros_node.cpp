#include <rclcpp/rclcpp.hpp>
#include <dt_ndt_mcl/pf_ros.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParticleFilter2D>());
  rclcpp::shutdown();
  return 0;

  return 0;
}