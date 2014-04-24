#include "tue_calibration/laser_detector.h"
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

LineDetector::LineDetector() {

}

LineDetector::~LineDetector() {

}

void LineDetector::init() {

    /// Topics and services
    ros::NodeHandle nh;
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 20);

}

void LineDetector::setData(sensor_msgs::LaserScan::ConstPtr msg) {
    msg_ = msg;
}

bool LineDetector::determineLine(double &x, double &y, double &phi) {

    /// Check if msg exists
    if (!msg_) {
        return false;
    }

    unsigned int startindex = (unsigned int)msg_->ranges.size()/2;
    double x0, x1, y0, y1, xleft, yleft, xright, yright, angle, delta2=0.0;
    /// Left corner
    unsigned int index = startindex;
    angle = msg_->angle_min + index * msg_->angle_increment;
    x1 = msg_->ranges[index]*cos(angle);
    y1 = msg_->ranges[index]*sin(angle);

    /// Loop until a corner has been found or you run out of laserpoints
    // ToDo: make more robust
    ++index;
    // ToDo: don't hardcode
    while (delta2 < 0.1)
    {
        if (index == msg_->ranges.size())
        {
            ROS_INFO("Cannot find left corner");
            return false;
        }

        /// Compute x and y
        x0 = x1;
        y0 = y1;
        angle = msg_->angle_min + index * msg_->angle_increment;
        x1 = msg_->ranges[index]*cos(angle);
        y1 = msg_->ranges[index]*sin(angle);

        /// Increment index
        ++index;

        /// Compute delta and check with threshold
        delta2 = (x1-x0)*(x1-x0) + (y1-y0)*(y1-y0);
    }
    int nr_samples = 3; /// Takes the average over 3 samples
    int offset = 3;     /// Gets delta, but takes the average not over the last couple of samples but a 'offset' angles off the edge
    if ( (index - offset - startindex) > nr_samples) {
        xleft = 0.0;
        yleft = 0.0;
        for (int ri = -nr_samples; ri < 0; ++ri) {
            angle  = msg_->angle_min + (index-offset+ri) * msg_->angle_increment;
            xleft += msg_->ranges[index-offset+ri]*cos(angle);
            yleft += msg_->ranges[index-offset+ri]*sin(angle);
        }
        xleft /= nr_samples;
        yleft /= nr_samples;
    } else {
        ROS_WARN("xleft and yleft might be inaccurate");
        xleft = x0;
        yleft = y0;
    }

    publishMarker(xleft,yleft,0,10,msg_->header.frame_id);
    //ROS_INFO("Index = %i of %i, angle = %f, range = %f",(int)index,(int)msg->ranges.size(),angle, msg->ranges[index-1]);
    //ROS_INFO("xleft = %f, yleft = %f", xleft, yleft);


    /// Loop until a corner has been found or you run out of laserpoints
    // ToDo: make more robust
    delta2 = 0.0;
    index = startindex;
    angle = msg_->angle_min + index * msg_->angle_increment;
    x1 = msg_->ranges[index]*cos(angle);
    y1 = msg_->ranges[index]*sin(angle);
    --index;
    while (delta2 < 0.1)
    {
        // ToDo: not very accurate
        if (index == 0)
        {
            ROS_INFO("Cannot find right corner");
            return false;
        }

        /// Compute x and y
        x0 = x1;
        y0 = y1;
        angle = msg_->angle_min + index * msg_->angle_increment;
        x1 = msg_->ranges[index]*cos(angle);
        y1 = msg_->ranges[index]*sin(angle);

        /// Increment index
        --index;

        /// Compute delta and check with threshold
        delta2 = (x1-x0)*(x1-x0) + (y1-y0)*(y1-y0);
    }
    if ( (startindex - index + offset) > nr_samples) {
        xright = 0.0;
        yright = 0.0;
        for (int ri = 1; ri <= nr_samples; ++ri) {
            angle  = msg_->angle_min + (index+offset+ri) * msg_->angle_increment;
            xright += msg_->ranges[index+offset+ri]*cos(angle);
            yright += msg_->ranges[index+offset+ri]*sin(angle);
        }
        xright /= nr_samples;
        yright /= nr_samples;
    } else {
        ROS_WARN("xleft and yleft might be inaccurate");
        xright = x0;
        yright = y0;
    }

    publishMarker(xright,yright,0,10,msg_->header.frame_id);
    //ToDo: add more robustness (range must be between 1 and 2 meters?
    //ROS_INFO("Index = %i of %i, angle = %f, range = %f",(int)index,(int)0,angle, msg->ranges[index+1]);
    //ROS_INFO("xright = %f, yright = %f", xright, yright);

    //x = (xleft + xright)/2;
    //y = (yleft + yright)/2;
    //ToDo: don't hardcode ratios:
    // Total width of wooden board is 0.848 m. Distance from left edge to corner measured by Kinect is 0.758 m, from right edge 0.900 m
    x = xleft + 0.758/0.848 * (xright-xleft);
    y = yleft + 0.758/0.848 * (yright-yleft);

    phi = atan2( (-xleft+xright), (yleft-yright) );
    ROS_INFO("[x, y, phi] = [%f\t%f\t%f]", x, y, phi);
    publishMarker(x,y,0.0,phi,msg_->header.frame_id);

    return true;

}

std::string LineDetector::getFrame() const {
    if (!msg_) {
        ROS_ERROR("No messages received");
        std::string emptystring;
        return emptystring;
    } else {
        return msg_->header.frame_id;
    }
}

void LineDetector::publishMarker(const double x, const double y, const double z, const double yaw, const std::string frame_id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.id = rand();
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    if (yaw > 7)
    {
        marker.type = 2;
    }
    else
    {
        marker.type = 0;
        marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,yaw);
    }
    marker.scale.x = 0.2;
    marker.scale.y = 0.3;
    marker.scale.z = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration(5.0);
    marker_pub_.publish(marker);
}
