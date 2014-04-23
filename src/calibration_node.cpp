#include "tue_calibration/calibration.h"

const double loop_rate_ = 10;

int main(int argc, char **argv) {

    /// Initialize node
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle nh_private("~");
    ros::Rate rate(loop_rate_);

    Calibration calibration;

    while (ros::ok()) {
        ros::spinOnce();

        rate.sleep();
    }
}
