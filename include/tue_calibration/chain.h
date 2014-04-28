/*!
 * \author Janno Lunenburg
 * \date April 2014
 * \version 0.1
 */

#ifndef KINEMATIC_CHAIN_H_
#define KINEMATIC_CHAIN_H_

#include <string>

/// KDL
//#include <kdl/chain.hpp>
//#include <kdl/jntarray.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <urdf/model.h>

/** Enum to distinguish measured joints from optimization joints */
enum JointType{measured, optimize};

class KinematicChain {

public:

    /** Default Constructor */
    KinematicChain();

    /** Constructor
     * @param root: root of the kinematic chain
     * @param tip: tip of the kinematic chain
     */
    KinematicChain(const std::string root_frame, const std::string tip_frame);

    /** Deconstructor */
    ~KinematicChain();

    /** Initialization function
     * @param root: root of the kinematic chain
     * @param tip: tip of the kinematic chain
     */
    bool init(const std::string root_frame, const std::string tip_frame);

    /** @brief Returns vector with joint names */
    std::vector<std::string> getJointNames() const;

    /** @brief computes pose of frame
     *  @param joint_array KDL::JntArray with joint values */
    KDL::Frame getFK(const KDL::JntArray& joint_array);

    /** @brief computes the Jacobian for the given joint array
     *  @param joint_array KDL::JntArray with joint values
     *  @param jacobian Jacobian matrix to be computed
     *  @return returns the output of the KDL JntToJac function */
    int getJacobian(const KDL::JntArray& joint_array, KDL::Jacobian& jacobian);

    /** @brief Returns the joint type of joint i
     *  @param joint_index: joint index
     *  @return Joint Type */
    JointType getJointType(const unsigned int joint_index) const;

private:

    /** @brief Loads model from xml description */
    bool loadModel(const std::string xml);

    /** @brief reads the joints of the urdf model */
    bool readJoints(urdf::Model &robot_model);

    /** @brief reeds the links of the urdf model */
    // Do we need this?
    bool readLinks(urdf::Model &robot_model);

    /** Root frame and tip frame */
    std::string root_frame_, tip_frame_;

    /** KDL Chain */
    KDL::Chain chain_;

    /** Number of joints */
    unsigned int num_joints_;

    /** Current joint state */
    KDL::JntArray q_;

    /** Joint limits */
    KDL::JntArray joint_min_, joint_max_;

    /** Vector with joint names */
    std::vector<std::string> joint_names_;

    /** Forward kinematics solver */
    KDL::ChainFkSolverPos_recursive* fk_solver_;

    /** Jacobian solver */
    KDL::ChainJntToJacSolver* jac_solver_;

    /** Vector containing joint types */
    std::vector<JointType> joint_types_;

};

#endif
