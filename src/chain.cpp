#include "tue_calibration/chain.h"
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>

KinematicChain::KinematicChain(): num_joints_(0) {

}

KinematicChain::KinematicChain(const std::string root_frame, const std::string tip_frame): num_joints_(0) {

    if (!init(root_frame, tip_frame)) {
        ROS_ERROR("Cannot initialize chain with root %s and tip %s", root_frame.c_str(), tip_frame.c_str());
    }
}

KinematicChain::~KinematicChain() {

}

bool KinematicChain::init(const std::string root_frame, const std::string tip_frame) {

    root_frame_ = root_frame;
    tip_frame_  = tip_frame;
    num_joints_ = 0;

    /// Get URDF XML
    ros::NodeHandle nh;
    std::string urdf_xml, full_urdf_xml;
    // ToDo: don't hardcode
    nh.param("urdf_xml",urdf_xml,std::string("/amigo/robot_opt_description"));
    nh.searchParam(urdf_xml,full_urdf_xml);
    ROS_DEBUG("Reading xml file from parameter server");
    std::string result;
    if (!nh.getParam(full_urdf_xml, result)) {
        ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return false;
    }

    /// Load and Read Models
    if (!loadModel(result)) {
        ROS_FATAL("Could not load models!");
        return false;
    }

    /// Initialize solvers
    jac_solver_ = new KDL::ChainJntToJacSolver(chain_);
    fk_solver_  = new KDL::ChainFkSolverPos_recursive(chain_);

    ROS_INFO("Kinematic chain successfully initialized");

    return true;
}

std::vector<std::string> KinematicChain::getJointNames() const {
    return joint_names_;
}

KDL::Frame KinematicChain::getFK(const KDL::JntArray& joint_array) {

    /// Check length of joint array
    if (joint_array.data.rows() != chain_.getNrOfJoints()) {
        ROS_ERROR("Nr input joints = %i while the chain has %i", joint_array.rows(), chain_.getNrOfJoints());
        KDL::Frame frame_out;
        return frame_out;
    }

    q_ = joint_array;
    KDL::Frame frame_out;
    /// Assume we're talking about the last segment
    int segment_nr = chain_.getNrOfSegments();
    fk_solver_->JntToCart(joint_array, frame_out, segment_nr);
    return frame_out;

}

int KinematicChain::getJacobian(const KDL::JntArray& joint_array, KDL::Jacobian& jacobian) {

    jacobian.resize(chain_.getNrOfJoints());
    return jac_solver_->JntToJac(joint_array, jacobian);

}

JointType KinematicChain::getJointType(const unsigned int joint_index) const {
    return joint_types_[joint_index];
}

bool KinematicChain::loadModel(const std::string xml) {
    urdf::Model robot_model;
    KDL::Tree tree;

    if (!robot_model.initString(xml)) {
        ROS_FATAL("Could not initialize robot model");
        return -1;
    }
    ROS_INFO("Robot model initialized");

    if (!kdl_parser::treeFromString(xml, tree)) {
        ROS_ERROR("Could not initialize tree object");
        return false;
    }
    ROS_INFO("Tree object initialized");

    //KDL::SegmentMap::iterator tree_end = tree.getSegments().end();
    if (tree.getSegment(root_frame_) == tree.getSegments().end()) {
        ROS_ERROR("Cannot find root frame %s", root_frame_.c_str());
    }
    if (tree.getSegment(tip_frame_) == tree.getSegments().end()) {
        ROS_ERROR("Cannot find tip frame %s", tip_frame_.c_str());
    }

    if (!tree.getChain(root_frame_, tip_frame_, chain_)) {
        ROS_ERROR("Could not initialize chain object");
        return false;
    }
    ROS_INFO("Chain object initialized");

    if (!readJoints(robot_model)) {
        ROS_FATAL("Could not read information about the joints");
        return false;
    }
    ROS_INFO("Robot joints read");

    //if (!readLinks(robot_model)) {
    //   ROS_FATAL("Could not read information about the links");
    //    return false;
    //}
    //ROS_INFO("Robot links read");

    return true;
}

bool KinematicChain::readJoints(urdf::Model &robot_model) {
    num_joints_ = 0;

    // get joint maxs and mins
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_frame_);
    boost::shared_ptr<const urdf::Joint> joint;

    //root to tip
    while (link && link->name != root_frame_) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (!joint) {
            ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
            return false;
        }
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            num_joints_++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }

    joint_min_.resize(num_joints_);
    joint_max_.resize(num_joints_);
    joint_names_.resize(num_joints_);
    joint_types_.resize(num_joints_);
    q_.resize(num_joints_);

    link = robot_model.getLink(tip_frame_);
    unsigned int i = 0;
    while (link && i < num_joints_) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {


            float lower, upper;
            if ( joint->type != urdf::Joint::CONTINUOUS ) {
                lower = joint->limits->lower;
                upper = joint->limits->upper;
            } else {
                lower = -M_PI;
                upper = M_PI;
            }
            int index = num_joints_ - i -1;

            joint_min_.data[index] = lower;
            joint_max_.data[index] = upper;
            joint_names_[index] = joint->name;

            /// Distinguish measured joints from optimization joints
            if (strncmp(joint->name.c_str(), "opt", 3) == 0) {
                joint_types_[index] = optimize;
            } else {
                joint_types_[index] = measured;
            }

            q_(index) =  0;
            i++;
        }
        link = robot_model.getLink(link->getParent()->name);

    }

    return true;
}

bool KinematicChain::readLinks(urdf::Model &robot_model) {

    int num_links = 1;
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_frame_);

    ROS_INFO("Tip link retreived");

    while (link && link->name != root_frame_) {
        if (!link) {
            ROS_ERROR("Could not find link: %s",link->name.c_str());
            return false;
        }

        num_links++;
        link = robot_model.getLink(link->getParent()->name);
    }

    link = robot_model.getLink(tip_frame_);
    int i = 0;
    while (link && i < num_links){
        //ROS_INFO("link nr %d = %s", i, link->name.c_str());
        link = robot_model.getLink(link->getParent()->name);
        i++;
    }

    return true;

}
