#include "tue_calibration/calibration.h"

/// Messages and services
#include "tue_calibration/getPose.h"

Calibration::Calibration() {

    init();

}

Calibration::~Calibration() {

}

void Calibration::init() {

    /// Topics and services
    ros::NodeHandle nh;
    laser_client_  = nh.serviceClient<tue_calibration::getPose>("/laser_line_detector/toggle_line_detector");
    kinect_client_ = nh.serviceClient<tue_calibration::getPose>("/kinect_checkerboard_detector/toggle_checkerboard_detector");

    toggle_sub_ = nh.subscribe("toggle_calibration", 1, &Calibration::toggleCallback, this);

}

void Calibration::toggleCallback(const std_msgs::String::ConstPtr& msg) {

    std::string add_meas_str("add");
    if (!msg->data.compare(add_meas_str)) {
        ROS_INFO("Adding measurement");

        /// Initialize opt data
        optimization_data meas_data;

        /// Get laser measurement
        tue_calibration::getPose laser_srv;
        if (laser_client_.call(laser_srv)) {
            stampedPoseToKDLframe(laser_srv.response.pose, meas_data.laser_meas_in_laser);
            ROS_INFO("Laser in laser = [%f, %f, %f]", meas_data.laser_meas_in_laser.p.x(), meas_data.laser_meas_in_laser.p.y(), meas_data.laser_meas_in_laser.p.z());
        } else {
            ROS_ERROR("Line detection failed, cancelling measurement");
            return;
        }

        /// Get kinect measurement
        tue_calibration::getPose kinect_srv;
        if (kinect_client_.call(kinect_srv)) {
            stampedPoseToKDLframe(kinect_srv.response.pose, meas_data.kinect_meas_in_kinect);
            ROS_INFO("Kinect in kinect = [%f, %f, %f]", meas_data.kinect_meas_in_kinect.p.x(), meas_data.kinect_meas_in_kinect.p.y(), meas_data.kinect_meas_in_kinect.p.z());
        } else {
            ROS_ERROR("Checkerboard detection failed, cancelling measurement");
            return;
        }

        /// Get joint measurement
        // Push desired values in joint array

        /// Append data
        optimization_data_.push_back(meas_data);

    } else {
        ROS_WARN("Don't know what to do");
    }

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
