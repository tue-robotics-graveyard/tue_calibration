#include "tue_calibration/calibration.h"

/// Messages and services
#include "tue_calibration/getPose.h"

Calibration::Calibration() {

    init();

}

Calibration::~Calibration() {
    delete laser_chain_;
    delete kinect_chain_;
    laser_chain_ = NULL;
    kinect_chain_ = NULL;

}

void Calibration::init() {

    /// Topics and services
    ros::NodeHandle nh;
    laser_client_  = nh.serviceClient<tue_calibration::getPose>("/laser_line_detector/toggle_line_detector");
    kinect_client_ = nh.serviceClient<tue_calibration::getPose>("/kinect_checkerboard_detector/toggle_checkerboard_detector");

    joint_state_sub_ = nh.subscribe("/amigo/joint_states", 5, &Calibration::jointStateCallback, this);
    joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/amigo/joint_states", 1);
    toggle_sub_ = nh.subscribe("toggle_calibration", 1, &Calibration::toggleCallback, this);

    /// Kinematic chain objects with solvers
    laser_chain_ = new KinematicChain("base_link", "base_laser");
    kinect_chain_ = new KinematicChain("base_link", "top_kinect/openni_rgb_optical_frame");

    /// Publish zero joint states for optimization joints
    sensor_msgs::JointState jointstate_msg;
    std::vector<std::string> joint_names = kinect_chain_->getJointNames();
    for (unsigned int i = 0; i < joint_names.size(); i++) {
        if (kinect_chain_->getJointType(i) == optimize) {
            jointstate_msg.name.push_back(joint_names[i]);
            jointstate_msg.position.push_back(0.0);
        }
    }
    joint_state_pub_.publish(jointstate_msg);

    ///// Test FK
    //KDL::JntArray qtest; qtest.resize(0);
    //KDL::Frame testfk = laser_chain_->getFK(qtest);
    //ROS_INFO("Testfk laser = [%f, %f, %f]", testfk.p.x(), testfk.p.y(), testfk.p.z());

    //KDL::JntArray qtest; qtest.resize(3);
    //qtest(0) = 0.400005;
    //qtest(1) = 0.0;
    //qtest(1) = -0.005118;
    //qtest(2) = 0.2456768;
    //qtest(4) = 0.0;
    //qtest(5) = 0.0;
    //std::vector<std::string> joint_names = kinect_chain_->getJointNames();
    //for (unsigned int i = 0; i < joint_names.size(); ++i) std::cout << joint_names[i] << " = " << qtest(i) << std::endl;
    //KDL::Frame testfk = kinect_chain_->getFK(qtest);
    //ROS_INFO("Testfk kinect = [%f, %f, %f]", testfk.p.x(), testfk.p.y(), testfk.p.z());

    /// Offsets
    kinect_offset_ = KDL::Frame();
    laser_offset_  = KDL::Frame(KDL::Vector(0.0, 0.0, 0.1015-0.24));

    /// Initialize optimizer
    optimizer_.init(laser_chain_, kinect_chain_);

}

void Calibration::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {

    // ToDo: do smarter. Store all joints?
    /// Loop over message
    for (unsigned int i = 0; i < msg->name.size(); i++) {
        //ROS_INFO("Storing %s: %f", msg->name[i].c_str(), msg->position[i]);
        joint_states_[msg->name[i]] = msg->position[i];
    }
}

void Calibration::toggleCallback(const std_msgs::String::ConstPtr& msg) {

    std::string add_meas_str("add");
    std::string optimize_str("opt");
    if (!msg->data.compare(add_meas_str)) {
        if (addMeasurement()) {
            ROS_INFO("Measurement successfully added to optimization data");
        }
    } else if (!msg->data.compare(optimize_str)) {
        if (calibrate()) {
            ROS_INFO("Calibration succeeded");
        }
    } else {
        ROS_WARN("Don't know what to do");
    }

}

bool Calibration::addMeasurement() {

    ROS_INFO("Adding measurement");

    /// Initialize opt data
    OptimizationData meas_data;

    /// Get laser measurement
    tue_calibration::getPose laser_srv;
    if (laser_client_.call(laser_srv)) {
        stampedPoseToKDLframe(laser_srv.response.pose, meas_data.laser_meas_in_laser);

        /// Compensate offset
        meas_data.laser_meas_in_laser = meas_data.laser_meas_in_laser * laser_offset_;

        /// Publish result
        marker_pub_.publishSphere(meas_data.laser_meas_in_laser.p.x(), meas_data.laser_meas_in_laser.p.y(), meas_data.laser_meas_in_laser.p.z(), laser_srv.response.pose.header.frame_id,
                                  1.0, 1.0, 0.0, 0.05);
        ROS_INFO("Laser in laser = [%f, %f, %f]", meas_data.laser_meas_in_laser.p.x(), meas_data.laser_meas_in_laser.p.y(), meas_data.laser_meas_in_laser.p.z());
    } else {
        ROS_ERROR("Line detection failed, cancelling measurement");
        return false;
    }

    /// Get kinect measurement
    tue_calibration::getPose kinect_srv;
    if (kinect_client_.call(kinect_srv)) {

        stampedPoseToKDLframe(kinect_srv.response.pose, meas_data.kinect_meas_in_kinect);

        /// Compensate offset
        meas_data.kinect_meas_in_kinect = meas_data.kinect_meas_in_kinect * kinect_offset_;
        std::cout << "x_ros = " << kinect_srv.response.pose.pose.position.x << ", x_kdl = " << meas_data.kinect_meas_in_kinect.p.x() << std::endl;
        std::cout << "y_ros = " << kinect_srv.response.pose.pose.position.y << ", y_kdl = " << meas_data.kinect_meas_in_kinect.p.y() << std::endl;
        std::cout << "z_ros = " << kinect_srv.response.pose.pose.position.z << ", z_kdl = " << meas_data.kinect_meas_in_kinect.p.z() << std::endl;

        /// Publish result
        marker_pub_.publishSphere(meas_data.kinect_meas_in_kinect.p.x(), meas_data.kinect_meas_in_kinect.p.y(), meas_data.kinect_meas_in_kinect.p.z(), kinect_srv.response.pose.header.frame_id,
                                  0.0, 0.0, 1.0, 0.05);
        ROS_INFO("Kinect in kinect = [%f, %f, %f]", meas_data.kinect_meas_in_kinect.p.x(), meas_data.kinect_meas_in_kinect.p.y(), meas_data.kinect_meas_in_kinect.p.z());
    } else {
        ROS_ERROR("Checkerboard detection failed, cancelling measurement");
        return false;
    }

    /// Get joint measurement
    // Assume only Kinect is relevant
    fillJointArray(kinect_chain_, meas_data.kinect_joint_data);

    /// Append data
    optimization_data_.push_back(meas_data);

    return true;

}

bool Calibration::calibrate() {

    /// Optimize
    if (optimizer_.optimize(optimization_data_)) {
        ROS_INFO("Optimization finished");
    } else {
        ROS_ERROR("Optimization failed");
        return false;
    }

    /// Publish result on jointstate topic
    std::map<std::string, double> opt_joints = optimizer_.getOptimizedJoints();
    sensor_msgs::JointState jointstate_msg;
    for (std::map<std::string, double>::iterator iter = opt_joints.begin(); iter != opt_joints.end(); ++iter) {
        jointstate_msg.name.push_back(iter->first);
        jointstate_msg.position.push_back(iter->second);
    }
    joint_state_pub_.publish(jointstate_msg);

    return true;
}

void Calibration::stampedPoseToKDLframe(const geometry_msgs::PoseStamped& pose, KDL::Frame& frame) {

    /// Position
    frame.p.x(pose.pose.position.x);
    frame.p.y(pose.pose.position.y);
    frame.p.z(pose.pose.position.z);

    /// Orientation
    frame.M = KDL::Rotation::Quaternion(pose.pose.orientation.x,
                                        pose.pose.orientation.y,
                                        pose.pose.orientation.z,
                                        pose.pose.orientation.w);

}

void Calibration::fillJointArray(const KinematicChain* chain, KDL::JntArray& joint_array) {

    /// Get joints
    std::vector<std::string> chain_joints = chain->getJointNames();

    /// Resize joint array
    joint_array.resize(chain_joints.size());

    /// Initialize all joints to zero
    joint_array.data.setZero();
    ROS_WARN("Filling joint array");
    /// Loop over joint states
    for (std::map<std::string, double>::iterator iter = joint_states_.begin(); iter != joint_states_.end(); iter++) {
        std::string current_joint = iter->first;
        /// Loop over relevant joints
        for (unsigned int j = 0; j < chain_joints.size(); j++) {
            //ROS_INFO("Current joint = %s, desired joint = %s", current_joint.c_str(), chain_joints[j].c_str());
            /// Check
            if (current_joint == chain_joints[j]) {
                joint_array(j) = iter->second;
                ROS_INFO("Joint %s = %f", current_joint.c_str(), iter->second);
            }
        }
    }
    ROS_WARN("Filled joint array");
}
