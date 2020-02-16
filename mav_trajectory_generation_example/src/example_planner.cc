#include <mav_trajectory_generation_example/example_planner.h>

ExamplePlanner::ExamplePlanner(ros::NodeHandle& nh) :
    nh_(nh),
    max_v_(15.0),
    max_a_(7.0),
    current_velocity_(Eigen::Vector3d::Zero()) {
    /*
    // Load params
    if (!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_)) {
        ROS_WARN("[example_planner] param max_v not found");
    }
    if (!nh_.getParam(ros::this_node::getName() + "/max_a", max_a_)) {
        ROS_WARN("[example_planner] param max_a not found");
    }
    */

    // create publisher for RVIZ markers
    pub_markers_ =
        nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);

    pub_trajectory_ =
        nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory",
            0);

    // subscriber for Odometry
    sub_odom_ =
        nh.subscribe("uav_pose", 1, &ExamplePlanner::uavPathCallback, this);
}

// Callback to get current Pose of UAV
void ExamplePlanner::uavPathCallback(const nav_msgs::Path::ConstPtr& path) {
    ROS_WARN("Message recieved! called Callback function.");

    //Eigen::Affine3d* p_ = new Eigen::Affine3d[3];

    // store current position in our planner
    for (int i = 0; i < 3; i++) {
        tf::poseMsgToEigen(path->poses[i].pose, p_[i]);
    }


    geometry_msgs::Vector3 v_;
    v_.x = path->poses[3].pose.orientation.x;
    v_.y = path->poses[3].pose.orientation.y;
    v_.z = path->poses[3].pose.orientation.z;

    // store current vleocity
    tf::vectorMsgToEigen(v_, current_velocity_);

    mav_trajectory_generation::Trajectory trajectory;
    planTrajectory(&trajectory);
    publishTrajectory(trajectory);

    ROS_WARN("Trajectory published");

}

// Method to set maximum speed.
void ExamplePlanner::setMaxSpeed(const double max_v) {
    max_v_ = max_v;
}

bool ExamplePlanner::planTrajectory(mav_trajectory_generation::Trajectory* trajectory) {

    // 3 Dimensional trajectory => through carteisan space, no orientation
    const int dimension = 3;

    // Array for all waypoints and their constrains
    mav_trajectory_generation::Vertex::Vector vertices;
    vertices.clear();

    // Optimze up to 4th order derivative (SNAP)
    const int derivative_to_optimize =
        mav_trajectory_generation::derivative_order::JERK;

    int n = 3; // number of gates in the current trajectory

    // Vertex::Vector createSquareVertices
    mav_trajectory_generation::Vertex gate(dimension);
    // set vertex point constraints to current position and set all derivatives to zero
    gate.makeStartOrEnd(p_[0].translation(),
        derivative_to_optimize);
    // Only add velocity constraints to the first position
        // set vertex point's velocity to be constrained
    gate.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
            current_velocity_);
    vertices.push_back(gate);


    for (int i = 1; i < n - 1; i++) { // The -1 is for the start and end
        // Vertex::Vector createSquareVertices
        mav_trajectory_generation::Vertex gate(dimension);

        // set vertex point's velocity to be constrained
        gate.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
            p_[i].translation());

        // add waypoint to list
        vertices.push_back(gate);
    }

    gate.makeStartOrEnd(p_[n-1].translation(),
        derivative_to_optimize);
    vertices.push_back(gate);

    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;

    // set up optimization problem
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));

    return true;
}

bool ExamplePlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory) {
    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance =
        0.1; // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";

    mav_trajectory_generation::drawMavTrajectory(trajectory,
        distance,
        frame_id,
        &markers);
    pub_markers_.publish(markers);

    // send trajectory to be executed on UAV
    mav_planning_msgs::PolynomialTrajectory msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
        &msg);
    msg.header.frame_id = "world";
    pub_trajectory_.publish(msg);

    return true;
}

