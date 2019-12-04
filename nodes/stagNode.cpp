#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>

#include "Stag.h"

Stag* stag;
ros::Publisher pose_publisher;

std::string CAMERA_FRAME = "/v4l_frame";
tf::TransformBroadcaster* transform_broadcaster = NULL;

geometry_msgs::PoseStamped cv_to_pose_stamped(cv::Mat translation, cv::Mat rotation, std::string frame_id, ros::Time stamp) {
    geometry_msgs::PoseStamped pose_msg;

    cv::Mat rotation_matrix(3, 3, CV_64FC1);
    cv::Rodrigues(rotation, rotation_matrix);

    tf::Matrix3x3 rotation_matrix_tf(
        rotation_matrix.at<double>(0, 0),
        rotation_matrix.at<double>(0, 1),
        rotation_matrix.at<double>(0, 2),
        rotation_matrix.at<double>(1, 0),
        rotation_matrix.at<double>(1, 1),
        rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0),
        rotation_matrix.at<double>(2, 1),
        rotation_matrix.at<double>(2, 2)
    );

    tf::Quaternion rotationQuat;
    rotation_matrix_tf.getRotation(rotationQuat);

    pose_msg.header.frame_id = frame_id;
    pose_msg.header.stamp = stamp;

    pose_msg.pose.position.x = translation.at<double>(0);
    pose_msg.pose.position.y = translation.at<double>(1);
    pose_msg.pose.position.z = translation.at<double>(2);

    pose_msg.pose.orientation.x = rotationQuat.x();
    pose_msg.pose.orientation.y = rotationQuat.y();
    pose_msg.pose.orientation.z = rotationQuat.z();
    pose_msg.pose.orientation.w = rotationQuat.w();

    return pose_msg;
}

tf::StampedTransform pose_stamped_to_transform(geometry_msgs::PoseStamped pose_msg, std::string parent_frame, std::string child_frame) {
    tf::Vector3 translation(
        pose_msg.pose.position.x,
        pose_msg.pose.position.y,
        pose_msg.pose.position.z
    );

    tf::Quaternion orientation(
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z,
        pose_msg.pose.orientation.w
    );

    tf::Transform marker_transform;
    marker_transform.setOrigin(translation);
    marker_transform.setRotation(orientation);

    tf::StampedTransform result(
        marker_transform,
        pose_msg.header.stamp,
        parent_frame,
        child_frame
    );

    return result;
}

void image_callback(const sensor_msgs::ImageConstPtr& image_message) {
    auto cv_ptr = cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::MONO8);
    auto num_tags = stag->detectMarkers(cv_ptr->image);

    cv::Mat camera_matrix = (cv::Mat1d(3, 3)<< 1066.342505, 0.000000, 940.432119, 0.000000, 1065.381509, 552.184409, 0.000000, 0.000000, 1.000000);
    cv::Mat distortion_coefficients(5, 1, CV_32FC1);

    distortion_coefficients = 0.0;

    if (num_tags > 0) {
        for (auto& detected_marker : stag->markers) {
            cv::Mat translation;
            cv::Mat rotation;

            detected_marker.getPose(camera_matrix, distortion_coefficients, 0.172, rotation, translation);

            auto pose_msg = cv_to_pose_stamped(translation, rotation, "/v4l_frame", image_message->header.stamp);
            auto marker_transform = pose_stamped_to_transform(pose_msg, "/v4l_frame", "/stag_marker_" + std::to_string(detected_marker.id));

            transform_broadcaster->sendTransform(marker_transform);
            pose_publisher.publish(pose_msg);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "stag_ros_test");

    ros::NodeHandle node_handle;
    image_transport::ImageTransport _image_transport(node_handle);

    auto image_subscriber = _image_transport.subscribe("/cv_camera/image_raw", 1, image_callback);
    pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/test_stag_ros/pose", 100);

    transform_broadcaster = new tf::TransformBroadcaster();

    stag = new Stag(21, 7, true);

    ros::spin();

    delete stag;
    delete transform_broadcaster;
}
