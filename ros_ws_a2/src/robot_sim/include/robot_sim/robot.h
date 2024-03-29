#pragma once

// #include <boost/thread.hpp>
#include <thread>
#include <mutex>
#include <vector>
#include <map>

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

namespace robot_sim {

class Robot : public rclcpp::Node 
{
 public:
  Robot(const std::map<std::string, size_t> &name_map);
  bool init();
  void stop();

  size_t getNumJoints() const {return num_joints_;}

  std::vector<std::string> getJointNames() const;

  std::vector<double> getJointValues() const;
  void setJointValues(const std::vector<double> &vals);
  void setJointValues(const std::vector<std::string> &names, const std::vector<double> &vals);

  void setTargetValues(const std::vector<double> &vals);
  void setTargetValues(const std::vector<std::string> &names, const std::vector<double> &vals);

  void setVelocities(const std::vector<double> &vels);
  void setVelocities(const std::vector<std::string> &names, const std::vector<double> &vels);

 private:
  size_t num_joints_;
  std::map<std::string, size_t> name_map_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_values_;
  std::vector<double> goal_values_;
  std::vector<double> joint_velocities_;
  bool velocity_mode_;
  double max_jump_;
  std::thread thread_;
  mutable std::mutex mutex_;
  bool running_;
  rclcpp::Time last_tick_;
  rclcpp::Time last_velocity_command_;
  rclcpp::Clock clock;
  double velocity_timeout_;

  void update();
};

} //namespace robot_sim
