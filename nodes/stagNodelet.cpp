#include <array>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "Stag.h"
#include "Marker.h"

namespace stag_ros {

class StagNodelet : public nodelet::Nodelet { 
public:
    ros::Publisher marker_message_publisher;

    tf::TransformBroadcaster* transform_broadcaster;
    tf::TransformListener* transform_listener;

    // need to publish the alvar markers message.

    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    bool have_camera_info;
    int highest_frame;
    std::string marker_frame_prefix;
    std::string output_frame_id;
    std::string marker_message_topic;
    std::map<int, double> marker_size_by_id;
    double default_marker_size;
    int frame_number;
    int tag_id_type;
    boost::shared_ptr<void> this_ptr;

    ros::Subscriber image_subscriber; 
    ros::Subscriber camera_info_subscriber;

    StagNodelet() : 
        transform_broadcaster(NULL), 
        transform_listener(NULL),
        have_camera_info(false),
        frame_number(0),
        highest_frame(0)
        {};


    double get_marker_size(size_t marker_id) {
        try {
            return marker_size_by_id.at(marker_id);
        } catch (std::out_of_range ex) {
            return default_marker_size;
        }
    }
    // say we have many bundles
    // we have a list of detected markers, and we have a set of bundles which are mappings between marker ids and corners.


    void get_individual_marker_pose(const Marker& marker, cv::Mat cameraMatrix, cv::Mat distortionCoefficients, float sideLengthMeters, cv::Mat& resultRotation, cv::Mat& resultTranslation) {
        // returns the result of solving the PnP problem
        std::vector<cv::Point3f> objectPoints;
        objectPoints.push_back((cv::Point3f(0.5, 0.5, 0.0) - cv::Point3f(0.5, 0.5, 0.0)) * sideLengthMeters);

        // order reversed to get a more normal coordinate orientation
        objectPoints.push_back((cv::Point3f(0.0, 1.0, 0.0) - cv::Point3f(0.5, 0.5, 0.0)) * sideLengthMeters);
        objectPoints.push_back((cv::Point3f(1.0, 1.0, 0.0) - cv::Point3f(0.5, 0.5, 0.0)) * sideLengthMeters);
        objectPoints.push_back((cv::Point3f(1.0, 0.0, 0.0) - cv::Point3f(0.5, 0.5, 0.0)) * sideLengthMeters);
        objectPoints.push_back((cv::Point3f(0.0, 0.0, 0.0) - cv::Point3f(0.5, 0.5, 0.0)) * sideLengthMeters);

        std::vector<cv::Point2f> imagePoints;

        imagePoints.push_back(marker.center);
        imagePoints.push_back(marker.corners[0]);
        imagePoints.push_back(marker.corners[1]);
        imagePoints.push_back(marker.corners[2]);
        imagePoints.push_back(marker.corners[3]);

        cv::Mat rotationRodrigues;
        cv::Mat translation;

        cv::solvePnP(
            objectPoints,
            imagePoints,
            cameraMatrix,
            distortionCoefficients,
            resultRotation,
            resultTranslation
        );
    }

    // add publishing of all of the tags to a message
    // add support for rviz

    // add support for transforming to an output frame
    // add support for marker bundles.

    tf::StampedTransform cv_to_tf_transform(cv::Mat translation, cv::Mat rotation, std::string image_frame_id, std::string marker_frame_id, ros::Time stamp) {
        tf::Transform marker_transform;

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

        tf::Vector3 translation_tf(translation.at<double>(0), translation.at<double>(1), translation.at<double>(2));

        tf::Quaternion rotationQuat;
        rotation_matrix_tf.getRotation(rotationQuat);

        marker_transform.setOrigin(translation_tf);
        marker_transform.setRotation(rotationQuat);

        tf::StampedTransform result(
            marker_transform,
            stamp,
            image_frame_id,
            marker_frame_id 
        );

        return result;
    }


    geometry_msgs::PoseStamped tf_to_pose_stamped(tf::Transform transform, std::string pose_frame_id, ros::Time stamp) {
        geometry_msgs::PoseStamped result;

        result.header.stamp = stamp;
        result.header.frame_id = pose_frame_id;

        result.pose.position.x = transform.getOrigin().x();
        result.pose.position.y = transform.getOrigin().y();
        result.pose.position.z = transform.getOrigin().z();

        result.pose.orientation.x = transform.getRotation().x();
        result.pose.orientation.y = transform.getRotation().y();
        result.pose.orientation.z = transform.getRotation().z();
        result.pose.orientation.w = transform.getRotation().w();

        return result;
    }


    bool marker_is_in_bundle(int id) {
        return false;
    }


    std::vector<ar_track_alvar_msgs::AlvarMarker> get_transforms_for_individual_markers(const std::vector<Marker>& markers, tf::StampedTransform camera_to_output_frame, std::string image_frame_id, ros::Time image_time_stamp) {
        std::vector<ar_track_alvar_msgs::AlvarMarker> alvar_markers;

        for (auto& detected_marker : markers) {
            if (marker_is_in_bundle(detected_marker.id)) {
                continue;
            }

            cv::Mat translation;
            cv::Mat rotation;

            std::string marker_frame_id = marker_frame_prefix + std::to_string(detected_marker.id);

            double marker_size_meters = get_marker_size(detected_marker.id);
            get_individual_marker_pose(detected_marker, camera_matrix, distortion_coefficients, marker_size_meters, rotation, translation);

            auto camera_to_marker_transform = cv_to_tf_transform(
                translation,
                rotation,
                image_frame_id,
                marker_frame_id,
                image_time_stamp 
            );

            auto output_to_marker_transform = camera_to_output_frame * camera_to_marker_transform;
            auto marker_pose_output_frame = tf_to_pose_stamped(
                output_to_marker_transform,
                output_frame_id, 
                image_time_stamp
            );

            transform_broadcaster->sendTransform(camera_to_marker_transform);

            ar_track_alvar_msgs::AlvarMarker serialized_marker;

            serialized_marker.id = detected_marker.id;
            serialized_marker.pose = marker_pose_output_frame;
            serialized_marker.header.stamp = ros::Time::now();
            alvar_markers.push_back(serialized_marker);
        }

        return alvar_markers;
    }


    std::vector<ar_track_alvar_msgs::AlvarMarker> get_transforms_for_bundled_markers(const std::vector<Marker>& markers, tf::StampedTransform camera_to_output_frame, std::string image_frame_id, ros::Time image_time_stamp) {
        std::vector<ar_track_alvar_msgs::AlvarMarker> result;
        return result;
    }

    void image_callback(const sensor_msgs::ImageConstPtr& image_message) {
        if (!have_camera_info) {
            return;
        }
        auto image_frame_id = image_message->header.frame_id;
        auto image_time_stamp = image_message->header.stamp;

        auto cv_ptr = cv_bridge::toCvShare(image_message, sensor_msgs::image_encodings::MONO8);

        Stag stag(tag_id_type, 7, false);

        tf::StampedTransform camera_to_output_frame;

        try {
            transform_listener->lookupTransform(output_frame_id, image_frame_id, ros::Time(0), camera_to_output_frame);
        } catch(tf::LookupException err) {
            ROS_WARN("Could get transformation from camera to output frame. Cannot process image.");
            ROS_WARN(err.what());
            return;
        }

        auto num_tags = stag.detectMarkers(cv_ptr->image);

        ar_track_alvar_msgs::AlvarMarkers markers_message;

        if (num_tags > 0) {
            auto individual_marker_messages = get_transforms_for_individual_markers(stag.markers, camera_to_output_frame, image_frame_id, image_time_stamp);
            markers_message.markers.insert(markers_message.markers.end(), individual_marker_messages.begin(), individual_marker_messages.end());

            auto bundle_marker_messages = get_transforms_for_bundled_markers(stag.markers, camera_to_output_frame, image_frame_id, image_time_stamp);
        }

        markers_message.header.stamp = ros::Time::now();
        markers_message.header.seq = image_message->header.seq;
        markers_message.header.frame_id = output_frame_id;
        marker_message_publisher.publish(markers_message);
    }

    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
        camera_matrix = (cv::Mat1d(3, 3) << camera_info_msg->K[0], camera_info_msg->K[1], camera_info_msg->K[2], camera_info_msg->K[3], camera_info_msg->K[4], camera_info_msg->K[5], camera_info_msg->K[6], camera_info_msg->K[7], camera_info_msg->K[8]);

        distortion_coefficients = (cv::Mat1d(5, 1) << camera_info_msg->D[0], camera_info_msg->D[1], camera_info_msg->D[2], camera_info_msg->D[3], camera_info_msg->D[4]);
        have_camera_info = true;
    }


    void parse_marker_sizes(ros::NodeHandle& private_node_handle) {
        XmlRpc::XmlRpcValue marker_sizes_list;
        bool param_success = private_node_handle.getParam("marker_sizes_by_id", marker_sizes_list);

        if (!param_success) {
            ROS_WARN("Could not parse marker_sizes_list or param was not provided. Ignoring.");
            return;
        }

        ROS_ASSERT(marker_sizes_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for (size_t i = 0; i < marker_sizes_list.size(); ++i) {
            ROS_ASSERT(marker_sizes_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
            marker_size_by_id[marker_sizes_list[i]["id"]] = marker_sizes_list[i]["size"];
        }
    }


    void parse_marker_bundles(ros::NodeHandle& private_node_handle) {

    }


    void onInit() {
        distortion_coefficients = cv::Mat(5, 1, CV_32FC1);

        ros::NodeHandle& private_node_handle = getMTPrivateNodeHandle();
        //ros::NodeHandle& private_node_handle = getPrivateNodeHandle();
        image_transport::ImageTransport _image_transport(private_node_handle);

        std::string camera_image_topic, camera_info_topic;

        private_node_handle.getParam("camera_image_topic", camera_image_topic);
        private_node_handle.getParam("camera_info_topic", camera_info_topic);
        private_node_handle.getParam("tag_id_type", tag_id_type);
        private_node_handle.getParam("marker_frame_prefix", marker_frame_prefix);
        private_node_handle.getParam("default_marker_size", default_marker_size);
        private_node_handle.getParam("output_frame_id", output_frame_id);
        private_node_handle.getParam("marker_message_topic", marker_message_topic);

        parse_marker_sizes(private_node_handle);
        parse_marker_bundles(private_node_handle);

        ros::SubscribeOptions opts;
        this_ptr = boost::shared_ptr<void>(static_cast<void*>(this));
        boost::function<void (const boost::shared_ptr<const sensor_msgs::Image>& )> f2( boost::bind( &StagNodelet::image_callback, this, _1 ) );
        opts = opts.template create<sensor_msgs::Image>(camera_image_topic, 10, f2, this_ptr, NULL);
        opts.allow_concurrent_callbacks = true;
        opts.transport_hints = ros::TransportHints();

        image_subscriber = private_node_handle.subscribe(opts);

        camera_info_subscriber = private_node_handle.subscribe(camera_info_topic, 10, &StagNodelet::camera_info_callback, this);

        // node handle where alvar_markers messages are published.
        marker_message_publisher = private_node_handle.advertise<ar_track_alvar_msgs::AlvarMarkers>(marker_message_topic, 100);

        transform_broadcaster = new tf::TransformBroadcaster();
        transform_listener = new tf::TransformListener();
    }
}; // class stag_ros

PLUGINLIB_EXPORT_CLASS(stag_ros::StagNodelet, nodelet::Nodelet)
} // namespace stag_ros
