/*!
 * \author Janno Lunenburg
 * \date April 2014
 * \version 0.1
 */

#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include "tue_calibration/chain.h"
#include "tue_calibration/optimization_data.h"
#include "tue_calibration/marker_publisher.h"

class Optimizer {

public:

    /** Default constructor */
    Optimizer();

    /** Constructor */
    Optimizer(KinematicChain* laser_chain, KinematicChain* kinect_chain);

    /** Deconstructor */
    ~Optimizer();

    /** Initialization */
    void init(KinematicChain* laser_chain, KinematicChain* kinect_chain);

    /** @brief minimizes the error between laser measurements and kinect measurements by varying the 'optimization joints'
     *  @param optimization_data: data from the measurements */
    bool optimize(std::vector<OptimizationData> &data);

    /** Returns optimized joints
     *  @return Map with joint name and corresponding value */
    std::map<std::string, double> getOptimizedJoints() const;

private:

    /** Kinematic chain objects to compute forward kinematics and Jacobians */
    KinematicChain* laser_chain_;
    KinematicChain* kinect_chain_;

    /** Number of DoFs that is considered. Since orientation seems less important, this can be 3
     * or all 6 */
    unsigned int num_dofs;

    // ToDo: no member variables below?
    /** Vector containing the stacked errors */
    Eigen::VectorXd error_vector_;

    /** Total Jacobian matrix */
    Eigen::MatrixXd total_jacobian_;

    /** Diagonal of the weighting matrix */
    Eigen::VectorXd weighting_matrix_;

    /** Marker publisher */
    MarkerPublisher marker_pub_;

    /** Optimized joints */
    std::map<std::string, double> optimized_joints_;

};

#endif
