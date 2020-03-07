#include <mav_trajectory_generation_example/example_planner.h>
#include <chrono>

#define N 20

#define f 10.0

PTG::PTG(ros::NodeHandle& nh) :
    nh_(nh),
    max_v_(30.0),
    max_a_(15.0),
    current_velocity_(Eigen::Vector3d::Zero()) {

    ith_gate = 0;
    section_length = 4; // Changed later in the gateCallback method

    // Controller
    sub_to_controller_state = nh.subscribe("/drone_1/state", 1, &PTG::state_callback, this);
    sub_to_controller_gate = nh.subscribe("/controller/gates", 1, &PTG::gate_callback, this);

    // Temporary, for testing
    pubControl = nh.advertise<geometry_msgs::Quaternion>("control", 2);

    // Game Theory Planner
    pub_to_gtp = nh.advertise<geometry_msgs::PoseArray>("ptg", 1); // Path generated by the PTG, as well as the initial state for the ego and opponent

    // create publisher for RVIZ markers
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
    pub_trajectory_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);

    current_velocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    current_acceleration_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    t_start = 2 * 1/f; // Two timesteps
    processed_gates = 0;

    ROS_INFO_STREAM("Initialized Planner node");
}

void PTG::gate_callback(const geometry_msgs::PoseArray::ConstPtr& msg){
    if(~processed_gates){ // To only process the gates once
    ROS_INFO_STREAM("Received gate poses!");

    int n_of_poses = int(msg->poses[0].position.x);
    n_of_gates = n_of_poses - 1;
    //section_length = n_of_gates - 1;
    section_length = 4;
    /* Use this to reserve segment sizes
    Segment::Vector segments;
    segments.reserve(segments_.size());
    */
    geometry_msgs::Pose pos;
    // Store the initial position and the gates' poses locally
    for(int i = 1; i < n_of_poses; i++){
        pos.position.x = msg->poses[i].position.x;
        pos.position.y = msg->poses[i].position.y;
        pos.position.z = msg->poses[i].position.z;

        pos.orientation.x = msg->poses[i].orientation.x;
        pos.orientation.y = msg->poses[i].orientation.y;
        pos.orientation.z = msg->poses[i].orientation.z;
        pos.orientation.w = msg->poses[i].orientation.w;

        gates.poses.push_back(pos);
    }

    // For use in PlanTrajectory
    Eigen::Affine3d p_;
    tf::poseMsgToEigen(gates.poses[0], p_);
    Eigen::Vector3d current_position_ = p_.translation();
    
    ROS_INFO_STREAM("Processed gate poses!\n");
    processed_gates = 1;
    generate_path();
    }

}

void PTG::state_callback(const nav_msgs::Odometry::ConstPtr& msg){

    if(processed_gates){ // Only call this function when the gates' positions have been received and processed
    std::cout << ith_gate << std::endl;
    ROS_INFO_STREAM("Received state (Odometry message)!");
    double x, y, z;
    double dx, dy, dz;

    // Get the position from the message
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;

    /* Find the closest point to it in the trajectory */

    // By first sampling a short time, finding the closest point to the current position, and then using that to sample the trajectory
    //double sampling_time = 2.0;
    //int derivative_order = mav_trajectory_generation::derivative_order::POSITION;


    t_start -= 2 * 1/f;
    if(t_start < 0){
        t_start = 0;
    }
    //double t_end = 10.0;
    double duration = 1;
    double dt = 1/(2*f); // Finer sampling
    //dt = 1/f;
    
    //mav_msgs::EigenTrajectoryPoint state;
    mav_msgs::EigenTrajectoryPoint::Vector local_states; // Local meaning around the current position
    //states.clear();
    //states.reserve(states.size());

    bool success;


    success = mav_trajectory_generation::sampleTrajectoryInRange(full_traj, t_start, t_start + duration, dt, &local_states);
    //std::vector<Eigen::VectorXd> result;
    //std::vector<double> sampling_times; // Optional.
    //full_traj.evaluateRange(t_start, t_end, dt, derivative_order, &result);

    int n_of_samples = int(floor(duration / dt));
    int j = 0;
    double min = 10000; // Any large number
    
    for (int i = 0; i < n_of_samples; i++){
        dx = local_states[i].position_W(0) - x;
        dy = local_states[i].position_W(1) - y;
        dz = local_states[i].position_W(2) - z;
        if (dx*dx + dy*dy + dz*dz < min){
            j = i;
            min = dx*dx + dy*dy + dz*dz;
        }
    }
    
    j += 3; // So the drone always moves forward, even if the closes point is slightly behind it


    /* Get a section of the trajectory starting from that point, + N time steps ahead */
    t_start = t_start + j * dt;
    dt = 1/double(f);
    duration = double(N) * dt;
    mav_msgs::EigenTrajectoryPoint::Vector states;

    std::cout << "t_start: " << t_start << " t_end: " << t_start + duration << std::endl;
    success = mav_trajectory_generation::sampleTrajectoryInRange(full_traj, t_start, t_start + duration, dt, &states);


    // Whole trajectory:
    //double sampling_interval = 0.01;
    //success = mav_trajectory_generation::sampleWholeTrajectory(full_traj, sampling_interval, &states);

    /* Publish the trajectory to the GTP */
    pubTraj(&states, msg);

    // If possible, get the parameters needed for the GTP, tangent, normal, and curvature
    // Can maybe get this from the polynomial coefficients of the segments

    // Generate a section of the trajectory
    generate_path();
    }

}

void PTG::generate_path(){
    if (ith_gate < n_of_gates){
        std::chrono::time_point<std::chrono::system_clock> m_StartTime = std::chrono::system_clock::now();

        mav_trajectory_generation::Trajectory trajectory;

        planTrajectory(&trajectory);

        //publishTrajectory(trajectory);
        //ROS_WARN("Trajectory published");

        std::chrono::time_point<std::chrono::system_clock> m_EndTime = std::chrono::system_clock::now();
        std::cout << "Generating the path took: ";
        std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(m_EndTime - m_StartTime).count()/1000.0 << std::endl;
    }
    else{
        publishTrajectory(full_traj);
    }

}

bool PTG::planTrajectory(mav_trajectory_generation::Trajectory* trajectory) {

    ROS_INFO_STREAM("Starting path generation loop!");
    // 3 Dimensional trajectory => through carteisan space, no orientation
    const int dimension = 3;

    // Start section planning
    //for(ith_gate; ith_gate < n_of_gates - section_length; ith_gate++){
    // Array for all waypoints and their constrains
    mav_trajectory_generation::Vertex::Vector vertices;
    vertices.clear();

    // Optimze up to 4th order derivative (SNAP)
    const int derivative_to_optimize =
        mav_trajectory_generation::derivative_order::JERK;

    // Vertex::Vector createSquareVertices
    mav_trajectory_generation::Vertex gate(dimension);
    // set vertex point constraints to current position and set all derivatives to zero
    gate.makeStartOrEnd(current_position_,
        derivative_to_optimize);
    // Only add velocity constraints to the first position
    // set vertex point's velocity and acceleration to be constrained
    gate.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
            current_velocity_);
    gate.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
            current_acceleration_);
    vertices.push_back(gate);

    for (int j = 1; j < section_length && ith_gate + j < n_of_gates; j++) {
        // Vertex::Vector createSquareVertices
        mav_trajectory_generation::Vertex gate(dimension);

        // set vertex point's velocity to be constrained
        tf::poseMsgToEigen(gates.poses[ith_gate + j], p_);
        //std::cout << p_.translation() << std::endl;
        gate.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
            p_.translation());

        // add waypoint to list
        vertices.push_back(gate);
    }

    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

    for (std::vector<double>::const_iterator i = segment_times.begin(); i != segment_times.end(); ++i)
        std::cout << *i << ' ';
    std::cout << std::endl;
    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;

    // set up optimization problem
    /*    
    parameters.max_iterations = 1000;
    parameters.f_rel = 0.05;
    parameters.x_rel = 0.1;
    parameters.time_penalty = 500.0;
    parameters.initial_stepsize_rel = 0.1;
    parameters.inequality_constraint_tolerance = 0.1;
    */
    const int K = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<K> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

    // solve trajectory
    opt.optimize();
    //opt.solveLinear();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));
    //trajectory->scaleSegmentTimesToMeetConstraints(max_v_, max_a_);

    // Append segments.
    if (ith_gate == 0){
        full_traj = *trajectory;
    }

    else{
        mav_trajectory_generation::Segment::Vector segments;

        trajectory->getSegments(&segments);

        full_traj.addSegments(segments);        
    }

    // Update initial position for the next iteration
    mav_msgs::EigenTrajectoryPoint state;
    //mav_msgs::EigenTrajectoryPoint::Vector states;
    // Choose a better sample time

    // Using the very last segment's state as the start
    double sample_time = 0;
    std::vector<double> sTimes = full_traj.getSegmentTimes();
    for(std::vector<double>::iterator it = sTimes.begin(); it != sTimes.end(); ++it)
        sample_time += *it;
    //std::cout << "Sample time: " << sample_time << std::endl;
    //sample_time = sTimes.back(); // Does this give the time at the start of the segment or the end of it?
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(full_traj, sample_time, &state);

    // Eigen::Vector3d
    current_position_ = state.position_W; // Use the position of the last gate in this iteration as the start of the next iteration
    current_velocity_ = state.velocity_W;
    current_acceleration_ = state.acceleration_W;

    ROS_INFO_STREAM("Finished " << ith_gate << " iterations of the loop");

    // Store the trajectory in memory

    //ith_gate += 1;
    ith_gate += section_length;
    if (ith_gate >= n_of_gates){
        ith_gate = n_of_gates;
        ROS_INFO_STREAM("Finished generating the whole trajectory");
    }

    return true;
}

void PTG::pubTraj(mav_msgs::EigenTrajectoryPoint::Vector* states, const nav_msgs::Odometry::ConstPtr& msg){
    
    geometry_msgs::PoseArray traj;
    geometry_msgs::Pose pos;

    traj = geometry_msgs::PoseArray(); // Making sure it's cleared
    pos = geometry_msgs::Pose(); // Just making sure that the message is cleared

    traj.header.frame_id = "world";
    // Adding the initial state to the first two poses x0 = [x, y, z, x., y., z. theta, phi, psi, ...]
    pos.position.x = msg->pose.pose.position.x;
    pos.position.y = msg->pose.pose.position.y;
    pos.position.z = msg->pose.pose.position.z;

    pos.orientation.x = msg->pose.pose.orientation.x;
    pos.orientation.y = msg->pose.pose.orientation.y;
    pos.orientation.z = msg->pose.pose.orientation.z;
    pos.orientation.w = 0;

    traj.poses.push_back(pos);

    pos = geometry_msgs::Pose(); // Just making sure that the message is cleared
    pos.position.x = msg->twist.twist.linear.x;
    pos.position.y = msg->twist.twist.linear.y;
    pos.position.z = msg->twist.twist.linear.z;

    pos.orientation.x = msg->twist.twist.angular.x;
    pos.orientation.y = msg->twist.twist.angular.y;
    pos.orientation.z = msg->twist.twist.angular.z;
    pos.orientation.w = 0;

    traj.poses.push_back(pos);

    // Sending the generated trajectory (x, y, z) to the GTP
    for(int i = 0; i < N; i++){ 
        pos.position.x = (*states)[i].position_W(0);
        pos.position.y = (*states)[i].position_W(1);
        pos.position.z = (*states)[i].position_W(2);
        traj.poses.push_back(pos);
    }

    pub_to_gtp.publish(traj);

    //This is temporary
    // Publish a 4d vector (Overloaded as a quaternion message for convenience) as the 4 control inputs to the quadcopter
    geometry_msgs::Quaternion cont;

    std::cout << "The size is: " << (*states).size() << std::endl;
    // Getting the control input from the solution of the optimization problem
    cont.x = (*states)[1].velocity_W(0) + (*states)[1].acceleration_W(0) * 1/double(f);
    cont.y = (*states)[1].velocity_W(1) + (*states)[1].acceleration_W(1) * 1/double(f);
    cont.z = (*states)[1].velocity_W(2) + (*states)[1].acceleration_W(2) * 1/double(f);
    cont.w = 0;
    std::cout << (*states)[1].velocity_W << std::endl;
    //pubControl.publish(cont);

    ROS_INFO_STREAM("Published trajectory to GTP");
}

bool PTG::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory) {
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

// Method to set maximum speed.
void PTG::setMaxSpeed(const double max_v) {
    max_v_ = max_v;
}