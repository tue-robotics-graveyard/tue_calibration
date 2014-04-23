/*!
 * \author Janno Lunenburg
 * \date April 2014
 * \version 0.1
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

/// ROS
#include "ros/ros.h"

/// Messages and services
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

/// KDL datatypes
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>


class Calibration {

public:

    /** Constructor */
    Calibration();

    /** Deconstructor */
    ~Calibration();

    /** Initialization function */
    void init();

private:

    /** @brief Toggle callback function: either adds measurements or computes result */
    void toggleCallback(const std_msgs::String::ConstPtr& msg);

    /**
     * @brief Converts Converts geometry_msgs::PoseStamped to KDL::Frame
     * @param pose: input pose
     * @param frame: output frame */
    void stampedPoseToKDLframe(const geometry_msgs::PoseStamped& pose, KDL::Frame& frame);

    /** Service clients to toggle laser detector and kinect detector */
    ros::ServiceClient laser_client_, kinect_client_;
    // ToDo: vector of clients to make it generic?

    /** Topic to toggle measurements */
    ros::Subscriber toggle_sub_;

    /** Struct containing the measurement information that is required for the optimization */
    struct optimization_data {
        KDL::Frame laser_meas_in_laser;
        KDL::Frame kinect_meas_in_kinect;
        KDL::Frame laser_in_baselink;
        KDL::Frame kinect_in_baselink;
        KDL::JntArray joint_data;
        KDL::Jacobian laser_jacobian;
        KDL::Jacobian kinect_jacobian;
    };

    /** Vector containing optimization data for each measurement */
    std::vector<optimization_data> optimization_data_;

};

#endif
