#include "tue_calibration/laser_detector.h"
#include "tue_calibration/getPose.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Empty.h"
#include <tf/tf.h>

const double loop_rate_ = 10;

LineDetector line_detector_;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    line_detector_.setData(msg);

}

bool toggleCallback(tue_calibration::getPose::Request   &req,
                    tue_calibration::getPose::Response  &resp) {

    double x, y, phi;
    if (!line_detector_.determineLine(x, y, phi)) {
        ROS_ERROR("Cannot determine line");
        return false;
    }

    resp.pose.header.frame_id = line_detector_.getFrame();
    resp.pose.header.stamp    = ros::Time::now();
    resp.pose.pose.position.x = x;
    resp.pose.pose.position.y = y;
    ROS_ERROR("Determine correct z-position");
    resp.pose.pose.position.z = 0.0;

    resp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(phi);

    return true;

}

int main(int argc, char **argv) {

    /// Initialize node
    ros::init(argc, argv, "laser_detector");
    ros::NodeHandle nh_private("~");
    ros::Rate rate(loop_rate_);

    ros::Subscriber    laser_sub  = nh_private.subscribe("/scan", 1, laserCallback);;
    ros::ServiceServer server_    = nh_private.advertiseService("toggle_line_detector", toggleCallback);

    line_detector_.init();

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}
