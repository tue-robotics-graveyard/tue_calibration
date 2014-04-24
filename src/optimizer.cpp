#include "tue_calibration/optimizer.h"

Optimizer::Optimizer() {

}

Optimizer::Optimizer(KinematicChain *laser_chain, KinematicChain *kinect_chain) {
    init(laser_chain, kinect_chain);
}

Optimizer::~Optimizer() {

}

void Optimizer::init(KinematicChain *laser_chain, KinematicChain *kinect_chain) {
    laser_chain_ = laser_chain;
    kinect_chain_ = kinect_chain;
    // ToDo: don't hardcode
    num_dofs = 3;
}

bool Optimizer::optimize(std::vector<OptimizationData>& data) {

    // Resize error vector
    // Resize Jacobian
    // Fill weighting matrices
    // Fill joint array (Eigen?) with measured data

    /// Number of measurements
    unsigned int n_meas = data.size();
    /// Number of measured joints in the kinect chain:
    unsigned int n_kinect_joints = kinect_chain_->getJointNames().size();
    /// Number of optimization variables in kinect chain:
    // ToDo: don't hardcode
    unsigned int n_kinect_opts = 0;

    /// Resize variables
    error_vector_.resize(n_meas*num_dofs);
    error_vector_.setZero();

    total_jacobian_.resize(n_meas*num_dofs, n_kinect_joints+n_kinect_opts);
    total_jacobian_.setZero();

    // ToDo: weighting matrices???

    /// Vectors with joint measurements and optimization variables as joints
    KDL::JntArray q_kinect, q_laser;
    q_kinect.resize(n_kinect_joints);
    //ToDo: fill properly with optimization data as well

    for (unsigned int i = 0; i < data[0].kinect_joint_data.data.rows(); i++) {
        q_kinect(i) = data[0].kinect_joint_data.data(i);
    }
    q_laser.resize(0);

    double norm = 1.0;
    unsigned int iter = 0;
    unsigned int max_iter = 10;
    /// While norm too large && iter < max_iter:
    while (norm > 0.01 && iter < max_iter) {

        ///  For every measurement
        for (unsigned int i = 0; i < data.size(); i++) {

            /// Compute kinect_meas in root
            KDL::Frame kinect_pose = kinect_chain_->getFK(q_kinect);
            KDL::Frame kinect_meas_in_root = kinect_pose * data[i].kinect_meas_in_kinect;
            std::cout << "Kinect measurement in root = " << kinect_meas_in_root.p.x() << ", "
                                                         << kinect_meas_in_root.p.y() << ", "
                                                         << kinect_meas_in_root.p.z() << std::endl;

            /// Compute laser meas in root
            KDL::Frame laser_pose = laser_chain_->getFK(q_laser);
            KDL::Frame laser_meas_in_root = laser_pose * data[i].laser_meas_in_laser;
            std::cout << "Laser measurement in root = " << laser_meas_in_root.p.x() << ", "
                                                        << laser_meas_in_root.p.y() << ", "
                                                        << laser_meas_in_root.p.z() << std::endl;

            /// Compute error
            KDL::Twist error = diff(kinect_meas_in_root, laser_meas_in_root);
            std::cout << "Error = " << error.vel.x() << ", "
                                    << error.vel.y() << ", "
                                    << error.vel.z() << std::endl;
        //      Compute laser_meas in root
        //      Compute error
        //      Put error in total vector
        //      Compute Kinect Jacobian
        //      Fill Kinect Jacobian in total Jacobian
        }
        //  Fill joint array with current values of joints to optimize
        //  Compute Jacobians
        //  Compute generalized pseudoinverse of complete Jacobian
        //  Premultiply erro vector with Jdagger to optain Delta q
        //  Add Delta q to q current
        ++iter;

    }

    return true;

}
