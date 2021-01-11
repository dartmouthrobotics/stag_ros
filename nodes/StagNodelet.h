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
#include <stag_ros/StagMarkers.h>
#include <stag_ros/SetTrackedBundles.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>

namespace stag_ros {

class MarkerBundle {
public:
    int broadcasted_id;
    std::map<int, std::vector<cv::Point3f>> corner_world_locations;

    bool fully_visible_in_previous_frame;
    cv::Rect previous_frame_image_bounding_box;

    std::vector<int> marker_ids() const {
        std::vector<int> result;

        for (auto key_value = corner_world_locations.begin(); key_value != corner_world_locations.end(); ++key_value) {
            result.push_back(key_value->first);
        }

        return result;
    }

    bool contains_marker_id(int id) const {
        return corner_world_locations.count(id) != 0;
    }
};

class StagNodelet : public nodelet::Nodelet { 
public:
    ros::Publisher marker_message_publisher;

    tf::TransformBroadcaster* transform_broadcaster;
    tf::TransformListener* transform_listener;

    std::vector<MarkerBundle> marker_bundles;
    std::vector<MarkerBundle> previously_detected_bundles;

    tf::StampedTransform camera_to_output_frame;
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    bool have_camera_info;
    bool use_marker_bundles;
    int highest_frame;
    int marker_track_width_offset;
    int marker_track_height_offset;
    std::string marker_frame_prefix;
    std::string output_frame_id;
    std::string marker_message_topic;
    std::map<int, double> marker_size_by_id;
    double default_marker_size;
    int frame_number;
    int tag_id_type;
    boost::shared_ptr<void> this_ptr;
    std::unique_ptr<image_transport::ImageTransport> image_transport_ptr;
    int _unused;
    bool save_debug_stream;
    bool track_markers;
    bool process_images_in_parallel;
    std::string debug_stream_directory;

    bool publish_debug_images;
    image_transport::Publisher undistorted_image_publisher;
    image_transport::Publisher optimized_corner_image_publisher;

    ros::Subscriber parallel_image_subscriber; 
    ros::Subscriber camera_info_subscriber;

    image_transport::Subscriber single_threaded_image_subscriber; 

    std::mutex transform_broadcaster_mutex;
    std::mutex marker_message_mutex;
    std::mutex cam_info_mutex;

    cv::Mat undistort_map1;
    cv::Mat undistort_map2;

    std::map<size_t, std::vector<cv::Point2d>> previous_image_points;

    ros::ServiceServer set_tracked_bundle_ids_service;

    StagNodelet() : 
        transform_broadcaster(NULL), 
        transform_listener(NULL),
        have_camera_info(false),
        frame_number(0),
        highest_frame(0)
        {};

    std::vector<size_t> trackable_bundle_ids;

    double get_marker_size(size_t marker_id);

    float get_individual_marker_pose(const stag::Marker& marker, cv::Mat cameraMatrix, cv::Mat distortionCoefficients, float sideLengthMeters, cv::Mat& resultRotation, cv::Mat& resultTranslation);

    void preprocess_image(const cv::Mat& raw_image, cv::Mat& output_image);

    void do_publish_debug_images(ros::Time image_time_stamp, cv::Mat& undistorted_image, stag_ros::StagMarkers& markers_message, std::string output_frame_id, size_t seq, std::map<int, std::vector<cv::Point2d>> unrefined_corners);

    tf::StampedTransform cv_to_tf_transform(cv::Mat translation, cv::Mat rotation, std::string image_frame_id, std::string marker_frame_id, ros::Time stamp);

    float get_reprojection_error(std::vector<cv::Point3f> object_points, std::vector<cv::Point2d> image_points, cv::Mat rotation_rodrigues, cv::Mat translation, cv::Mat camera_matrix);

    geometry_msgs::PoseStamped tf_to_pose_stamped(tf::Transform transform, std::string pose_frame_id, ros::Time stamp);

    bool marker_is_in_bundle(int id);

    cv::Mat downsample_image(const cv::Mat& image);

    std::vector<cv::Point2d> get_previous_image_points(size_t id);

    std::vector<stag_ros::StagMarker> get_transforms_for_individual_markers(const std::vector<stag::Marker>& markers, tf::StampedTransform camera_to_output_frame, std::string image_frame_id, ros::Time image_time_stamp);

    stag_ros::StagMarker construct_alvar_marker_and_publish_transform(int id, cv::Mat rotation, cv::Mat translation, std::string image_frame_id, ros::Time image_time_stamp, std::string marker_frame_id, std::vector<cv::Point2d> corners);

    std::vector<stag_ros::StagMarker> get_transforms_for_bundled_markers(const std::vector<stag::Marker>& markers, tf::StampedTransform camera_to_output_frame, std::string image_frame_id, ros::Time image_time_stamp);

    void image_callback(const sensor_msgs::ImageConstPtr& image_message);

    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg);

    void parse_marker_sizes(ros::NodeHandle& private_node_handle);

    void parse_marker_bundles(ros::NodeHandle& private_node_handle);

    std::map<size_t, std::vector<cv::Point2d>> refine_corners(std::vector<stag::Marker>& markers, cv::Mat& image);

    std::vector<stag::Marker> detect_markers(const cv::Mat& image, stag::Stag& stag_instance);

    bool should_track_bundles();

    void onInit();

    void update_bundle_tracking_information(MarkerBundle& bundle, const std::vector<stag::Marker>& detected_markers, cv::Point2f marker_corner_offset, int image_cols, int image_rows);

    bool set_tracked_bundle_ids_callback(stag_ros::SetTrackedBundles::Request& request, stag_ros::SetTrackedBundles::Response& response);

    bool bundle_is_trackable(const MarkerBundle& bundle);

}; // class stag_ros

} // namespace stag_ros

PLUGINLIB_EXPORT_CLASS(stag_ros::StagNodelet, nodelet::Nodelet);
#endif
