#include "tue_calibration/optimizer.h"

/// Eigen
#include <Eigen/SVD>

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
    // ToDo: six dofs does not work!
    num_dofs = 3;
}

bool Optimizer::optimize(std::vector<OptimizationData>& data) {

    // Resize error vector
    // Resize Jacobian
    // Fill weighting matrices
    // Fill joint array (Eigen?) with measured data
    /// Print joint names
    std::vector<std::string> joint_names = kinect_chain_->getJointNames();
    for (unsigned int i = 0; i < joint_names.size(); i++) {
        std::cout << "Joint " << i << " = " << joint_names[i] << std::endl;
    }

    /// Number of measurements
    unsigned int n_meas = data.size();
    /// Number of measured joints in the kinect chain:
    unsigned int n_kinect_joints = kinect_chain_->getJointNames().size();

    /// Gain vector
    // ToDo: don't hardcode gain
    Eigen::VectorXd gain; gain.resize(n_kinect_joints); for (unsigned int i = 0; i < gain.rows(); i++) gain(i) = 1.0;

    /// Resize variables
    error_vector_.resize(n_meas*num_dofs);
    error_vector_.setZero();

    total_jacobian_.resize(n_meas*num_dofs, n_kinect_joints);
    total_jacobian_.setZero();

    // ToDo: modify???
    weighting_matrix_.resize(n_kinect_joints);
    for (unsigned int i = 0; i < weighting_matrix_.rows(); i++) {
        weighting_matrix_(i) = double(kinect_chain_->getJointType(i));
    }
    std::cout << "Weighting matrix = \n" << weighting_matrix_ << std::endl;

    /// Vectors with joint measurements and optimization variables as joints
    KDL::JntArray q_kinect, q_laser;
    q_kinect.resize(n_kinect_joints);
    q_kinect.data.setZero(); // This assumes that all optimization joints are initialized as zero!!!

    q_laser.resize(0);

    /// Data for summary
    std::vector<KDL::Vector> errors;
    errors.resize(data.size());
    std::vector<double> t_norm;

    double norm = 1.0;
    unsigned int iter = 0;
    unsigned int max_iter = 100;
    /// While norm too large && iter < max_iter:
    while (norm > 0.001 && iter < max_iter) {

        /// For every measurement
        for (unsigned int i = 0; i < data.size(); i++) {

            /// Update measurement data (weight = 0), keep optimization data (weight = 1)
            for (unsigned int j = 0; j < data[i].kinect_joint_data.data.rows(); j++) {
                if (weighting_matrix_(j) == 0) {
                    q_kinect(j) = data[i].kinect_joint_data.data(j);
                }
            }

            /// Compute kinect_meas in root
            KDL::Frame kinect_pose = kinect_chain_->getFK(q_kinect);

            KDL::Frame kinect_meas_in_root = kinect_pose * data[i].kinect_meas_in_kinect;

            /// Compute laser meas in root
            KDL::Frame laser_pose = laser_chain_->getFK(q_laser);
            KDL::Frame laser_meas_in_root = laser_pose * data[i].laser_meas_in_laser;

            KDL::Twist error = KDL::diff(kinect_meas_in_root, laser_meas_in_root);

            /// If errors too large: don't take this measurement into account
            double threshold = 0.2;
            if (fabs(error.vel.x()) > threshold || fabs(error.vel.y()) > threshold || fabs(error.vel.z()) > threshold) {
                std::cout << "Warning: error of measurement " << i << " = " << error.vel.x() << ", "
                          << error.vel.y() << ", "
                          << error.vel.z() << ", norm^2 = " << error.vel.x()*error.vel.x() + error.vel.y()*error.vel.y() + error.vel.z()*error.vel.z() <<
                             ", discarding measurement!!!" << std::endl;
                continue;
            }
            errors[i] = KDL::Vector(error.vel.x(), error.vel.y(), error.vel.z());

            /// Print initial measurement to screen
            if (iter == 0) {
                std::cout << "Kinect pose in root = " << kinect_pose.p.x() << ", "
                          << kinect_pose.p.y() << ", "
                          << kinect_pose.p.z() << std::endl;

                std::cout << "Kinect measurement in root = " << kinect_meas_in_root.p.x() << ", "
                          << kinect_meas_in_root.p.y() << ", "
                          << kinect_meas_in_root.p.z() << std::endl;

                std::cout << "Laser measurement in root = " << laser_meas_in_root.p.x() << ", "
                          << laser_meas_in_root.p.y() << ", "
                          << laser_meas_in_root.p.z() << std::endl;

                std::cout << "Error = " << error.vel.x() << ", "
                          << error.vel.y() << ", "
                          << error.vel.z() << ", norm^2 = " << error.vel.x()*error.vel.x() + error.vel.y()*error.vel.y() + error.vel.z()*error.vel.z() <<  std::endl;
            }

            /// Put error in vector
            if (num_dofs == 3) {
                error_vector_(3*i+0) = error.vel.x();
                error_vector_(3*i+1) = error.vel.y();
                error_vector_(3*i+2) = error.vel.z();
            } else if (num_dofs == 6) {
                error_vector_(6*i+0) = error.vel.x();
                error_vector_(6*i+1) = error.vel.y();
                error_vector_(6*i+2) = error.vel.z();
                error_vector_(6*i+3) = error.rot.x();
                error_vector_(6*i+4) = error.rot.y();
                error_vector_(6*i+5) = error.rot.z();
            } else {
                std::cout << "Please define num_dofs as 3 or 6, cancelling optimization" << std::endl;
                return false;
            }

            /// Compute Kinect Jacobian
            KDL::Jacobian kinect_jacobian;
            kinect_chain_->getJacobian(q_kinect, kinect_jacobian);
            /// Change reference point: error is w.r.t. measurements
            KDL::Twist ref_point_offset = KDL::diff(kinect_pose, kinect_meas_in_root);
            kinect_jacobian.changeRefPoint(ref_point_offset.vel);
            //std::cout << "Kinect Jacobian = \n " << kinect_jacobian.data << std::endl;

            /// Fill in Jacobian
            // Weigh entire matrix or per Jacobian?
            total_jacobian_.block(num_dofs*i, 0, num_dofs, kinect_jacobian.data.cols()) = kinect_jacobian.data.block(0, 0, num_dofs, kinect_jacobian.data.cols());

        }

        ///  Compute generalized pseudoinverse of complete Jacobian
        // From: WBC/ComputeNullspace...
       // std::cout << "Jacobian = \n " << total_jacobian_ << std::endl;
        Eigen::MatrixXd WJ = total_jacobian_ * weighting_matrix_.asDiagonal() * total_jacobian_.transpose(); // Weighted Jacobian
        Eigen::JacobiSVD<Eigen::MatrixXd> WJsvd(WJ, Eigen::ComputeThinU | Eigen::ComputeFullV);

        // ToDo: Choose eps wisely
        double eps=0.00001;
        Eigen::VectorXd Svec = WJsvd.singularValues();
        Eigen::VectorXd Sinv;
        Sinv.resize(Svec.rows());

        for (int i = 0; i < WJsvd.singularValues().rows(); i++) {
            if (Svec(i) > eps) Sinv(i) = 1.0/Svec(i);
            else Sinv(i) = 0;
        }
        /// Jpinv = A*J^T * (J*A*J^T)^(-1)
        //std::cout << "Weighting matrix = \n" << weighting_matrix_ << "\n Jacobian transpose = \n" << total_jacobian_.transpose() << "\n Vmat = \n" <<
        //             WJsvd.matrixV() << "\n Sinv = \n " << Sinv << "\n Umat = \n" << WJsvd.matrixU() << std::endl;
        Eigen::MatrixXd Jpinv = weighting_matrix_.asDiagonal() * total_jacobian_.transpose() * WJsvd.matrixV() * Sinv.asDiagonal() * WJsvd.matrixU().transpose();
        //std::cout << "Jpinv = \n" << Jpinv << std::endl;

        ///  Premultiply erro vector with Jdagger to optain Delta q
        Eigen::VectorXd delta_q = gain.asDiagonal() * Jpinv * error_vector_;

        ///  Add Delta q to q current
        for (unsigned int i = 0; i < delta_q.rows(); i++) {
            //q_kinect(i) += delta_q(i);
            q_kinect(i) += delta_q(i)/n_meas;
        }

        norm = error_vector_.norm();
        t_norm.push_back(norm);
        //std::cout << "Iteration: " << iter+1 << ", norm: " << norm << "\n" << std::endl;

        /// Print initial measurement to screen
        if (iter == 0) {
            for (unsigned int i = 0; i < delta_q.rows(); i++) {
                if (weighting_matrix_(i) == 1.0) {
                    std::cout << joint_names[i] << ": old = " << q_kinect(i) << ", delta = " << delta_q(i) << ", new = " << q_kinect(i)+delta_q(i) << std::endl;
                }
            }
        }

        ++iter;

    }

    /// Summarize results
    std::cout << "\nSUMMARY\n" << std::endl;
    std::cout << "Optimization converged after " << iter << " of " << max_iter << " iterations, norm = " << norm << "\n" << std::endl;
    for (unsigned int i = 0; i < q_kinect.data.rows(); i++) {
        if (weighting_matrix_(i) == 1) {
            std::cout << "Joint " << joint_names[i] << std::endl;//", optimized: " << q_kinect(i) << std::endl;
            optimized_joints_[joint_names[i]] = q_kinect(i);
        }
    }
    for (unsigned int i = 0; i < q_kinect.data.rows(); i++) {
        if (weighting_matrix_(i) == 1) {
            std::cout << q_kinect(i) << std::endl;
        }
    }
    std::cout << "\n" << std::endl;
    for (unsigned int i = 0; i < data.size(); i++) {
        std::cout << "Error of measurement " << i+1 << ": x = " << errors[i].x() << ", y = " << errors[i].y() << ", z = " << errors[i].z() << std::endl;
    }
    std::cout << "\n" << std::endl;
    for (unsigned int i = 0; i < t_norm.size(); i++) {
        std::cout << "Norm " << i << " = " << t_norm[i] << std::endl;
    }

    // Debug
    //for (unsigned int i = 0; i < error_vector_.rows(); ++i) {
    //    std::cout << "Error (" << i << ") = " << error_vector_(i) << std::endl;
    //}

    return true;

}

std::map<std::string, double> Optimizer::getOptimizedJoints() const {
    return optimized_joints_;
}
