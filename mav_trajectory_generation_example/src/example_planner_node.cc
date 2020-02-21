/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */

#include  "ros/ros.h"
#include <mav_trajectory_generation_example/example_planner.h>

#include <iostream>

int main(int argc, char** argv) {

    ros::init(argc, argv, "simple_planner");

    ros::NodeHandle n;
    PTG planner(n);
    //ros::Duration(1.0).sleep();

    ros::spin();

    ROS_WARN_STREAM("DONE. GOODBYE.");

    return 0;
}
