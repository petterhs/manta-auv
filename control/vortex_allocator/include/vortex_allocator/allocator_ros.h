#ifndef VORTEX_ALLOCATOR_ALLOCATOR_ROS_H
#define VORTEX_ALLOCATOR_ALLOCATOR_ROS_H

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"

#include "vortex_allocator/pseudoinverse_allocator.h"

#include <map>
#include <string>
#include <vector>

class Allocator
{
public:
  explicit Allocator(ros::NodeHandle nh);
  void callback(const geometry_msgs::Wrench &msg) const;
private:
  ros::NodeHandle m_nh;
  ros::Subscriber m_sub;
  ros::Publisher  m_pub;

  ros::Publisher  m_pub0;
  ros::Publisher  m_pub1;
  ros::Publisher  m_pub2;
  ros::Publisher  m_pub3;
  ros::Publisher  m_pub4;
  ros::Publisher  m_pub5;
  ros::Publisher  m_pub6;
  ros::Publisher  m_pub7;



  int m_num_degrees_of_freedom;
  int m_num_thrusters;
  std::vector<int> m_direction;
  std::map<std::string, bool> m_active_degrees_of_freedom;
  double m_min_thrust;
  double m_max_thrust;
  const double c_force_range_limit = 100;

  std::unique_ptr<PseudoinverseAllocator> m_pseudoinverse_allocator;

  Eigen::VectorXd rovForcesMsgToEigen(const geometry_msgs::Wrench &msg) const;
  bool healthyWrench(const Eigen::VectorXd &v) const;
};

#endif  // VORTEX_ALLOCATOR_ALLOCATOR_ROS_H
