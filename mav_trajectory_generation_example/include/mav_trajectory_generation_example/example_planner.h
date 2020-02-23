#ifndef MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
#define MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>


class PTG {
 public:
  PTG(ros::NodeHandle& nh);
  void uavPathCallback(const nav_msgs::Path::ConstPtr& pose);

  void setMaxSpeed(double max_v);

  // Plans a trajectory to take off from the current position and
  // fly to the given altitude (while maintaining x,y, and yaw).
  bool planTrajectory(mav_trajectory_generation::Trajectory* trajectory);
                      
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      const Eigen::VectorXd& start_pos,
                      const Eigen::VectorXd& start_vel,
                      double v_max, double a_max,
                      mav_trajectory_generation::Trajectory* trajectory);
                      
  bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

  void gateCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

  void stateCallback(const nav_msgs::Odometry::ConstPtr& msg);

  void pubTraj(mav_msgs::EigenTrajectoryPoint::Vector* states);

  void generate_path();

 private:
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;
  ros::Publisher pub_to_gtp;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_to_airSim_gate;
  ros::Subscriber sub_to_airSim_state;

  mav_trajectory_generation::Trajectory full_traj;

  ros::NodeHandle& nh_;
  Eigen::Affine3d current_pose_;
  Eigen::Vector3d current_position_;  
  Eigen::Vector3d current_velocity_;
  Eigen::Vector3d current_acceleration_;
  Eigen::Vector3d current_angular_velocity_;
  //Eigen::Affine3d* p_ = new Eigen::Affine3d[3];
  Eigen::Affine3d p_;




  geometry_msgs::PoseArray gates;

  double max_v_; // m/s
  double max_a_; // m/s^2
  double max_ang_v_;
  double max_ang_a_;
  int ith_gate;
  int n_of_gates;
  int section_length;
  double t_start;
  bool processed_gates;

};

#endif // MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
