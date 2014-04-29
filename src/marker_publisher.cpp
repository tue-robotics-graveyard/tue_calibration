#include "tue_calibration/marker_publisher.h"

MarkerPublisher::MarkerPublisher() {

    /// Topics and services
    ros::NodeHandle nh;
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 20);

    /// Defaults
    id_ = 0;
    duration_ = ros::Duration(5.0);

}

MarkerPublisher::~MarkerPublisher() {

}


void MarkerPublisher::publishSphere(const double x, const double y, const double z, const std::string& frame_id, const double red, const double green, const double blue, const double size) {

    /// Initialize and fill marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp    = ros::Time::now();
    marker.id              = id_; ++id_;
    marker.type            = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.scale.x         = size;
    marker.scale.y         = size;
    marker.scale.z         = size;
    marker.color.r         = red;
    marker.color.g         = green;
    marker.color.b         = blue;
    marker.color.a         = 1.0;
    marker.lifetime        = duration_;

    /// Publish
    marker_pub_.publish(marker);

}
