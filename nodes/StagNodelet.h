#ifndef STAG_ROS_STAG_NODELET
#define STAG_ROS_STAG_NODELET

#include "Stag.h"
#include "Marker.h"

#include <ros/ros.h>
#include <map>
#include <mutex>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <stag_ros/StagMarker.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>

namespace stag_ros {

class MarkerBundle {
public:
    int broadcasted_id;
    std::map<int, std::vector<cv::Point3f>> corner_world_locations;
};

class StagNodelet : public nodelet::Nodelet { 
public:
    ros::Publisher marker_message_publisher;

    tf::TransformBroadcaster* transform_broadcaster;
    tf::TransformListener* transform_listener;

    std::vector<MarkerBundle> marker_bundles;

    tf::StampedTransform camera_to_output_frame;
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    bool have_camera_info;
    bool use_marker_bundles;
    int highest_frame;
    std::string marker_frame_prefix;
    std::string output_frame_id;
    std::string marker_message_topic;
    std::map<int, double> marker_size_by_id;
    double default_marker_size;
    int frame_number;
    int tag_id_type;
    boost::shared_ptr<void> this_ptr;
    int _unused;

    ros::Subscriber image_subscriber; 
    ros::Subscriber camera_info_subscriber;

    std::mutex transform_broadcaster_mutex;
    std::mutex marker_message_mutex;
    std::mutex cam_info_mutex;

    cv::Mat undistort_map1;
    cv::Mat undistort_map2;

    StagNodelet() : 
        transform_broadcaster(NULL), 
        transform_listener(NULL),
        have_camera_info(false),
        frame_number(0),
        highest_frame(0)
        {};

    double get_marker_size(size_t marker_id);

    void get_individual_marker_pose(const stag::Marker& marker, cv::Mat cameraMatrix, cv::Mat distortionCoefficients, float sideLengthMeters, cv::Mat& resultRotation, cv::Mat& resultTranslation);

    tf::StampedTransform cv_to_tf_transform(cv::Mat translation, cv::Mat rotation, std::string image_frame_id, std::string marker_frame_id, ros::Time stamp);

    geometry_msgs::PoseStamped tf_to_pose_stamped(tf::Transform transform, std::string pose_frame_id, ros::Time stamp);

    bool marker_is_in_bundle(int id);

    std::vector<stag_ros::StagMarker> get_transforms_for_individual_markers(const std::vector<stag::Marker>& markers, tf::StampedTransform camera_to_output_frame, std::string image_frame_id, ros::Time image_time_stamp);

    stag_ros::StagMarker construct_alvar_marker_and_publish_transform(int id, cv::Mat rotation, cv::Mat translation, std::string image_frame_id, ros::Time image_time_stamp, std::string marker_frame_id, std::vector<cv::Point2d> corners);

    std::vector<stag_ros::StagMarker> get_transforms_for_bundled_markers(const std::vector<stag::Marker>& markers, tf::StampedTransform camera_to_output_frame, std::string image_frame_id, ros::Time image_time_stamp);

    void image_callback(const sensor_msgs::ImageConstPtr& image_message);

    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg);

    void parse_marker_sizes(ros::NodeHandle& private_node_handle);

    void parse_marker_bundles(ros::NodeHandle& private_node_handle);

    void onInit();

}; // class stag_ros

} // namespace stag_ros

PLUGINLIB_EXPORT_CLASS(stag_ros::StagNodelet, nodelet::Nodelet);
#endif
