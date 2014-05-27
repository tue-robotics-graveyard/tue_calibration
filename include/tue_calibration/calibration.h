/*!
 * \author Janno Lunenburg
 * \date April 2014
 * \version 0.1
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include <map>

/// ROS
#include "ros/ros.h"

/// Messages and services
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

/// KDL datatypes
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <tue_calibration/chain.h>
#include "tue_calibration/optimizer.h"


class Calibration {

public:

    /** Constructor */
    Calibration();

    /** Deconstructor */
    ~Calibration();

    /** Initialization function */
    void init();

private:

    /** @brief Callback function simply stores all received measurements */
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    /** @brief Toggle callback function: either adds measurements or computes result */
    void toggleCallback(const std_msgs::String::ConstPtr& msg);

    /** @brief adds measurement to optimization data vector */
    bool addMeasurement();

    /** @brief calls optimizer */
    bool calibrate();

    /**
     * @brief Converts Converts geometry_msgs::PoseStamped to KDL::Frame
     * @param pose: input pose
     * @param frame: output frame */
    void stampedPoseToKDLframe(const geometry_msgs::PoseStamped& pose, KDL::Frame& frame);

    /** @brief fills the joint array with the joint positions of the chain joints
     * @param chain: pointer to the relevant kinematic chain
     * @param joint_array: joint array to be filled */
    void fillJointArray(const KinematicChain* chain, KDL::JntArray& joint_array);

    /** Service clients to toggle laser detector and kinect detector */
    ros::ServiceClient laser_client_, kinect_client_;
    // ToDo: vector of clients to make it generic?

    /** Subscriber to joint states */
    ros::Subscriber joint_state_sub_;

    /** Joint state pub: publishes the 'optimization' joint states */
    ros::Publisher joint_state_pub_;

    /** Topic to toggle measurements */
    ros::Subscriber toggle_sub_;

    /** Vector containing optimization data for each measurement */
    std::vector<OptimizationData> optimization_data_;

    /** Kinematic chain objects including forward kinematics and jacobian solvers */
    KinematicChain* laser_chain_;
    KinematicChain* kinect_chain_;

    /** Offsets to compensate for physical offset between measurement points (e.g., laser measures at fixed height, Kinect at lower right corner) */
    KDL::Frame laser_offset_;
    KDL::Frame kinect_offset_;

    /** Stores all relevant joint states */
    std::map<std::string, double> joint_states_;

    /** Optimizer object */
    Optimizer optimizer_;

    /** Marker publisher */
    MarkerPublisher marker_pub_;
};

#endif
