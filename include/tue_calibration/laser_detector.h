/*!
 * \author Janno Lunenburg
 * \date April 2014
 * \version 0.1
 */

#ifndef LASER_DETECTOR_H_
#define LASER_DETECTOR_H_

/// ROS
#include "ros/ros.h"

/// Messages
#include <sensor_msgs/LaserScan.h>

class LineDetector {

public:

    /** Constructor */
    LineDetector();

    /** Deconstructor */
    ~LineDetector();

    /** @brief Initialization for marker publisher */
    void init();

    /**
     * @brief Set data
     * @param laser_data Map from angle (rad) to distance (m) */
    void setData(sensor_msgs::LaserScan::ConstPtr msg);

    /** Determines the position and orientation w.r.t. the laser frame
     * of the line perpendicular through the middle of the
     * edge of the box in sight
     * @param x: x-coordinate (with respect to laser frame)
     * @param y: y-coordinate (with respect to laser frame)
     * @param phi: rotation (with respect to the x-axis of the laser frame)
     */
     bool determineLine(double &x, double &y, double &phi);

     /** Returns frame id of latest message */
     std::string getFrame() const;

private:

    /** Contains the most recent measurement */
    sensor_msgs::LaserScan::ConstPtr msg_;

    /** Marker publisher */
    ros::Publisher marker_pub_;

    /** Publishes marker
     * @param x: x-coordinate
     * @param y: y-coordinate
     * @param z: z-coordinate
     * @param yaw: yaw angle (pitch and yaw are assumed zero)
     * @param frame id
     */
     void publishMarker(const double x, const double y, const double z, const double yaw, const std::string frame_id);

};

#endif
