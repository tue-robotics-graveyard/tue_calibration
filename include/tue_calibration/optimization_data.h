/*!
 * \author Janno Lunenburg
 * \date April 2014
 * \version 0.1
 */

#ifndef OPTIMIZATION_DATA_H_
#define OPTIMIZATION_DATA_H_

/** Struct containing the measurement information that is required for the optimization */
struct OptimizationData {
	KDL::Frame laser_meas_in_laser;
	KDL::Frame kinect_meas_in_kinect;
	KDL::Frame laser_in_root;
	KDL::Frame kinect_in_root;
	KDL::JntArray kinect_joint_data;
	KDL::Jacobian laser_jacobian;
	KDL::Jacobian kinect_jacobian;
    KDL::Frame offset; /// Physical offset between what the Kinect and the laser detect
};

#endif
