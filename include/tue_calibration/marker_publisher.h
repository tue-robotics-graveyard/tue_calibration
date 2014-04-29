/*!
 * \author Janno Lunenburg
 * \date April 2014
 * \version 0.1
 */

#ifndef MARKER_PUBLISHER_H_
#define MARKER_PUBLISHER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class MarkerPublisher {

public:

    /** Default constructor */
    MarkerPublisher();

    /** Deconstructor */
    ~MarkerPublisher();

    /** Publish functions */
    void publishSphere(const double x, const double y, const double z, const std::string& frame_id, const double red, const double green, const double blue, const double size);

private:

    /** Publisher */
    ros::Publisher marker_pub_;

    /** ID */
    unsigned int id_;

    /** Default duration */
    ros::Duration duration_;


};

#endif


