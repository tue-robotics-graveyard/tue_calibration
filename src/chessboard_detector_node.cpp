#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <visualization_msgs/Marker.h>
#include "tue_calibration/getPose.h"
#include "tue_calibration/detect_calibration_pattern.h"

/// For testing
#include <cv.h>
#include <highgui.h>

const double loop_rate_ = 10;

PatternDetector detector_;
sensor_msgs::ImageConstPtr image_msg_;
sensor_msgs::CameraInfoConstPtr info_msg_;
ros::Publisher marker_pub_;

void publishMarker(const geometry_msgs::PoseStamped& pose) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = pose.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.id = rand();
    marker.type = 0;
    marker.pose = pose.pose;
    marker.scale.x = 0.2;
    marker.scale.y = 0.3;
    marker.scale.z = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration(5.0);
    marker_pub_.publish(marker);
}

void cameraCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
    ROS_INFO("Image received");
    image_msg_ = image_msg;
    info_msg_  = info_msg;
}

bool toggleCallback(tue_calibration::getPose::Request   &req,
                    tue_calibration::getPose::Response  &resp) {

    //ToDo: don't hardcode
    unsigned int pattern_width  = 14;
    unsigned int pattern_height = 9;
    double       pattern_size   = 0.05;//0.1;
    unsigned int pattern_type   = CHESSBOARD;
    cv::Size grid_size(pattern_width, pattern_height);
    detector_.setPattern(grid_size, pattern_size, Pattern(pattern_type));

    ROS_INFO("Received image, performing detection");

    /// Convert image message
    cv_bridge::CvImagePtr img_bridge;
    try {
        img_bridge = cv_bridge::toCvCopy(image_msg_, "mono8");
    } catch (cv_bridge::Exception& ex) {
        ROS_ERROR("Failed to convert image");
        return false;
    }
    //cv::imshow("Image", img_bridge->image);
    //cv::waitKey(0);


    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(info_msg_);
    detector_.setCameraMatrices(cam_model.intrinsicMatrix(), cam_model.distortionCoeffs());

    Eigen::Vector3f translation;
    Eigen::Quaternionf orientation;

    if (detector_.detectPattern(img_bridge->image, translation, orientation)) {
        ROS_INFO("Detected checkerboard");
        ROS_WARN("Is %s the correct frame id???", "/amigo/top_kinect/openni_rgb_optical_frame");

        /// Correct rotation
        // ToDo: more efficient
        Eigen::Matrix3f Rmatx, Rmaty;
        Rmatx << 1.0, 0.0, 0.0,
                 0.0, 0.0,-1.0,
                 0.0, 1.0, 0.0;
        Rmaty << 0.0,-1.0, 0.0,
                 1.0, 0.0, 0.0,
                 0.0, 0.0, 1.0;
        Eigen::Quaternionf orientationx(Rmatx);
        Eigen::Quaternionf orientationy(Rmaty);
        orientation = orientation*orientationx*orientationy;

        resp.pose.header.frame_id    = "/amigo/top_kinect/openni_rgb_optical_frame"; // ToDo: don't hardcode
        resp.pose.header.stamp       = ros::Time::now();
        resp.pose.pose.position.x    = translation(0);
        resp.pose.pose.position.y    = translation(1);
        resp.pose.pose.position.z    = translation(2);
        resp.pose.pose.orientation.x = orientation.x();
        resp.pose.pose.orientation.y = orientation.y();
        resp.pose.pose.orientation.z = orientation.z();
        resp.pose.pose.orientation.w = orientation.w();

        //Eigen::Quaternionf orientation2(orientation*orientation);
        publishMarker(resp.pose);
        std::cout << "Type 0 for false, 1 for correct detection" << std::endl;
        cv::imshow("Image", img_bridge->image);
        char feedback = cv::waitKey();
        ROS_INFO("Feedback = %c", feedback);
        if (feedback == '1') {
            ROS_DEBUG("Checkerboard accepted");
        } else if (feedback == '0') {
            ROS_WARN("Checkerboard rejected, canceling measurement");
            return false;
        } else {
            ROS_ERROR("Feedback not defined, canceling measurement");
            return false;
        }
    } else {
        ROS_ERROR("No checkerboard detected");
        return false;
    }

    return true;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_detector");
  ros::NodeHandle nh_private("~");
  ros::Rate rate(loop_rate_);

  ros::ServiceServer server_    = nh_private.advertiseService("toggle_checkerboard_detector", toggleCallback);

  ros::NodeHandle nh;
  std::string image_topic("/amigo/top_kinect/rgb/image_rect");
  image_transport::ImageTransport it(nh);
  image_transport::CameraSubscriber camera_sub = it.subscribeCamera(image_topic, 1, cameraCallback);
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 20);

  while (ros::ok()) {
      ros::spinOnce();
      rate.sleep();
  }

  return 0;
}
