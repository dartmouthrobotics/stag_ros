#include "StagNodelet.h"

#include <array>

#include <ros/ros.h>
#include <mutex>
#include <tf/LinearMath/Matrix3x3.h>
#include <stag_ros/StagMarkers.h>
#include <opencv2/core/core.hpp>

namespace stag_ros {

double StagNodelet::get_marker_size(size_t marker_id) {
    try {
        return marker_size_by_id.at(marker_id);
    } catch (std::out_of_range ex) {
        return default_marker_size;
    }
}


float StagNodelet::get_reprojection_error(std::vector<cv::Point3f> object_points, std::vector<cv::Point2d> image_points, cv::Mat rotation_rodrigues, cv::Mat translation, cv::Mat camera_matrix) {
    std::vector<cv::Point2f> reprojected_points;

    std::vector<cv::Point2f> image_points_float;
    for (auto& pt : image_points) {
        image_points_float.push_back(pt);
    }

    cv::projectPoints(object_points, rotation_rodrigues, translation, camera_matrix, cv::Mat(), reprojected_points);

    float total_error = 0;
    for (size_t point_index = 0; point_index < image_points.size(); ++point_index) {
        auto real_image_point = image_points_float[point_index];
        auto predicted_image_point = reprojected_points[point_index];
        auto diff = real_image_point - predicted_image_point;

        total_error += cv::sqrt(diff.x*diff.x + diff.y*diff.y);
    }

    return total_error;
}


std::vector<cv::Point2d> StagNodelet::get_previous_image_points(size_t id) {
    if (previous_image_points.count(id) == 0) {
        std::vector<cv::Point2d> result;
        return result;
    }

    return previous_image_points[id];
}


float StagNodelet::get_individual_marker_pose(const stag::Marker& marker, cv::Mat cameraMatrix, cv::Mat distortionCoefficients, float sideLengthMeters, cv::Mat& resultRotation, cv::Mat& resultTranslation) {
    // returns the result of solving the PnP problem
    std::vector<cv::Point3f> objectPoints;

    objectPoints.push_back((cv::Point3f(0.5, 0.5, 0.0) - cv::Point3f(0.5, 0.5, 0.0)) * sideLengthMeters);

    // order reversed to get a more normal coordinate orientation
    objectPoints.push_back((cv::Point3f(0.0, 1.0, 0.0) - cv::Point3f(0.5, 0.5, 0.0)) * sideLengthMeters);
    objectPoints.push_back((cv::Point3f(1.0, 1.0, 0.0) - cv::Point3f(0.5, 0.5, 0.0)) * sideLengthMeters);
    objectPoints.push_back((cv::Point3f(1.0, 0.0, 0.0) - cv::Point3f(0.5, 0.5, 0.0)) * sideLengthMeters);
    objectPoints.push_back((cv::Point3f(0.0, 0.0, 0.0) - cv::Point3f(0.5, 0.5, 0.0)) * sideLengthMeters);

    std::vector<cv::Point2d> imagePoints;

    imagePoints.push_back(marker.center);
    imagePoints.push_back(marker.corners[0]);
    imagePoints.push_back(marker.corners[1]);
    imagePoints.push_back(marker.corners[2]);
    imagePoints.push_back(marker.corners[3]);

    cv::Mat rotationRodrigues;
    cv::Mat translation;

    cam_info_mutex.lock();
    cv::solvePnP(
        objectPoints,
        imagePoints,
        cameraMatrix,
        cv::Mat(),
        resultRotation,
        resultTranslation
    );

    auto reprojection_error = get_reprojection_error(objectPoints, imagePoints, resultRotation, resultTranslation, cameraMatrix);
    cam_info_mutex.unlock();

    return reprojection_error;
}

tf::StampedTransform StagNodelet::cv_to_tf_transform(cv::Mat translation, cv::Mat rotation, std::string image_frame_id, std::string marker_frame_id, ros::Time stamp) {
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

geometry_msgs::PoseStamped StagNodelet::tf_to_pose_stamped(tf::Transform transform, std::string pose_frame_id, ros::Time stamp) {
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


bool StagNodelet::marker_is_in_bundle(int id) {
    for (auto& bundle : marker_bundles) {
        if (bundle.corner_world_locations.count(id) > 0) {
            return true;
        }
    }

    return false;
}


std::vector<stag_ros::StagMarker> StagNodelet::get_transforms_for_individual_markers(const std::vector<stag::Marker>& markers, tf::StampedTransform camera_to_output_frame, std::string image_frame_id, ros::Time image_time_stamp) {
    std::vector<stag_ros::StagMarker> alvar_markers;

    for (auto& detected_marker : markers) {
        if (marker_is_in_bundle(detected_marker.id) && use_marker_bundles) {
            continue;
        }

        cv::Mat translation;
        cv::Mat rotation;

        std::string marker_frame_id = marker_frame_prefix + std::to_string(detected_marker.id);

        double marker_size_meters = get_marker_size(detected_marker.id);
        float reprojection_error = get_individual_marker_pose(detected_marker, camera_matrix, distortion_coefficients, marker_size_meters, rotation, translation);

        auto serialized_marker = construct_alvar_marker_and_publish_transform(detected_marker.id, rotation, translation, image_frame_id, image_time_stamp, marker_frame_id, detected_marker.corners);

        serialized_marker.reprojection_error = reprojection_error;
        alvar_markers.push_back(serialized_marker);
    }

    return alvar_markers;
}


stag_ros::StagMarker StagNodelet::construct_alvar_marker_and_publish_transform(int id, cv::Mat rotation, cv::Mat translation, std::string image_frame_id, ros::Time image_time_stamp, std::string marker_frame_id, std::vector<cv::Point2d> corners) {
    auto camera_to_marker_transform = cv_to_tf_transform(
        translation,
        rotation,
        image_frame_id,
        marker_frame_id,
        image_time_stamp 
    );

    auto output_to_marker_transform = camera_to_output_frame * camera_to_marker_transform;

    transform_broadcaster_mutex.lock();
    transform_broadcaster->sendTransform(camera_to_marker_transform);
    transform_broadcaster_mutex.unlock();

    auto marker_pose_output_frame = tf_to_pose_stamped(
        output_to_marker_transform,
        output_frame_id, 
        image_time_stamp
    );

    stag_ros::StagMarker serialized_marker;

    serialized_marker.id = id;
    serialized_marker.pose = marker_pose_output_frame;
    serialized_marker.header.stamp = image_time_stamp;
    
    for (size_t corner_num = 0; corner_num < corners.size(); ++corner_num) {
        geometry_msgs::Point corner;

        corner.z = 0;
        corner.x = corners[corner_num].x;
        corner.y = corners[corner_num].y;

        serialized_marker.corners.push_back(corner); 
    }

    return serialized_marker;
}


std::vector<stag_ros::StagMarker> StagNodelet::get_transforms_for_bundled_markers(const std::vector<stag::Marker>& markers, tf::StampedTransform camera_to_output_frame, std::string image_frame_id, ros::Time image_time_stamp) {
    std::vector<stag_ros::StagMarker> result;

    for (auto& bundle : marker_bundles) {
        std::vector<cv::Point2d> image_points;
        std::vector<cv::Point3f> world_points;
        std::array<size_t, 4> bundle_corner_indices {3, 0, 2, 1};

        cv::Mat rotation_rodrigues;
        cv::Mat translation;

        for (auto& visible_marker : markers) {
            if (bundle.corner_world_locations.count(visible_marker.id) > 0) { 
                for (size_t corner_number = 0; corner_number < 4; ++corner_number) {
                    image_points.push_back(visible_marker.corners[corner_number]);
                    world_points.push_back(bundle.corner_world_locations[visible_marker.id][bundle_corner_indices[corner_number]]);
                }
            }
        }

        if (!image_points.empty()) {
            try {
                cam_info_mutex.lock();
                cv::solvePnP(
                    world_points,
                    image_points,
                    camera_matrix,
                    cv::Mat(),
                    rotation_rodrigues,
                    translation,
                    false
                );

                auto reprojection_error = get_reprojection_error(world_points, image_points, rotation_rodrigues, translation, camera_matrix);
                cam_info_mutex.unlock();

                std::string marker_frame_id = marker_frame_prefix + std::to_string(bundle.broadcasted_id);
                auto marker_message = construct_alvar_marker_and_publish_transform(bundle.broadcasted_id, rotation_rodrigues, translation, image_frame_id, image_time_stamp, marker_frame_id, image_points);
                marker_message.reprojection_error = reprojection_error;

                result.push_back(marker_message);
            } catch (cv::Exception error) {
                ROS_WARN_STREAM("Failed to get pose for bundled marker. Skipping bundle id " << bundle.broadcasted_id);
                cam_info_mutex.unlock();
            }
        }
    }

    return result;
}


void StagNodelet::preprocess_image(const cv::Mat& raw_image, cv::Mat& output_image) {
    cam_info_mutex.lock();
    cv::remap(raw_image, output_image, undistort_map1, undistort_map2, cv::INTER_CUBIC);
    cam_info_mutex.unlock();
}


void StagNodelet::do_publish_debug_images(ros::Time image_time_stamp, cv::Mat& undistorted_image, stag_ros::StagMarkers& markers_message, std::string output_frame_id, size_t seq, std::map<int, std::vector<cv::Point2d>> unrefined_corners) {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", undistorted_image).toImageMsg();
    msg->header.stamp = image_time_stamp;
    msg->header.seq = seq;
    msg->header.frame_id = output_frame_id;
    undistorted_image_publisher.publish(msg);

    cv::Mat corner_marked_image;

    cv::cvtColor(undistorted_image, corner_marked_image, cv::COLOR_GRAY2BGR);

    for (auto& marker : markers_message.markers) {
        for (auto& corner : marker.corners) {
            cv::Vec3b& intensity = corner_marked_image.at<cv::Vec3b>(corner.y, corner.x);

            intensity.val[0] = 0;
            intensity.val[1] = 0;
            intensity.val[2] = 255;
        }
    }

    sensor_msgs::ImagePtr corner_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", corner_marked_image).toImageMsg();

    corner_msg->header.stamp = image_time_stamp;
    corner_msg->header.seq = seq;
    corner_msg->header.frame_id = output_frame_id;
    optimized_corner_image_publisher.publish(corner_msg);

    seq = frame_number++;

    if (save_debug_stream) {
        std::string corner_file_name = debug_stream_directory + "/corner_locations_" + std::to_string(seq) + ".txt";
        std::string unrefined_corner_file_name = debug_stream_directory + "/unrefined_corner_locations_" + std::to_string(seq) + ".txt";

        std::ofstream ofs(corner_file_name);
        std::ofstream unrefined_stream(unrefined_corner_file_name);
        for (auto& marker : markers_message.markers) {
            ofs << marker.id << std::endl;
            unrefined_stream << marker.id << std::endl;
            for (auto& corner : marker.corners) {
                ofs << corner.x << " " << corner.y << std::endl;
            }

            for (auto& corner : unrefined_corners[marker.id]) {
                unrefined_stream << corner.x << " " << corner.y << std::endl; 
            }
        }
        ofs.close();

        std::string image_file_name = debug_stream_directory + "/preprocessed_" + std::to_string(seq) + ".pgm";
        cv::imwrite(image_file_name, undistorted_image);
    }
}


std::map<size_t, std::vector<cv::Point2d>> StagNodelet::refine_corners(std::vector<stag::Marker>& markers, cv::Mat& image) {
    std::map<size_t, std::vector<cv::Point2d>> result;

    for (auto& marker : markers) {
        cv::TermCriteria terminationCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS, 100000, 0.00001);

        cv::Mat corners_mat(marker.corners);
        corners_mat.convertTo(corners_mat, CV_32FC1);

        cv::cornerSubPix(image, corners_mat, cv::Size(2, 2), cv::Size(-1, -1), terminationCriteria);

        for (size_t i = 0; i < corners_mat.rows; ++i) {
            marker.corners[i] = cv::Point2d(corners_mat.at<float>(i, 0), corners_mat.at<float>(i, 1));
        }
    }

    return result;
}


bool StagNodelet::should_track_markers() {
    return previously_detected_markers.size() > 0 && track_markers;
}

int clamp_to_range(int to_clamp, int minimum, int maximum) {
    return std::min(
        std::max(
            to_clamp,
            minimum
        ),
        maximum
    );
}

std::vector<stag::Marker> StagNodelet::detect_markers(const cv::Mat& image, stag::Stag& stag_instance) {

    if (should_track_markers()) {
        std::vector<stag::Marker> markers;

        for (auto& marker : previously_detected_markers) {
            std::vector<cv::Point2f> corners_float;
            cv::Mat(marker.corners).convertTo(corners_float, cv::Mat(corners_float).type());
            cv::Rect marker_bounding_box = cv::boundingRect(corners_float);

            marker_bounding_box.x = clamp_to_range(marker_bounding_box.x - marker_track_width_offset, 0, image.cols);
            marker_bounding_box.y = clamp_to_range(marker_bounding_box.y - marker_track_height_offset, 0, image.rows);

            if (marker_bounding_box.x + marker_bounding_box.width + marker_track_width_offset * 2 > image.cols) {
                marker_bounding_box.width = image.cols - marker_bounding_box.x;
            } else {
                marker_bounding_box.width = marker_bounding_box.width + marker_track_width_offset * 2;
            }

            if (marker_bounding_box.y + marker_bounding_box.height + marker_track_height_offset * 2 > image.rows) {
                marker_bounding_box.height = image.rows - marker_bounding_box.y;
            } else {
                marker_bounding_box.height = marker_bounding_box.height + marker_track_height_offset * 2;
            }

            cv::Mat sub_image(marker_bounding_box.height, marker_bounding_box.width, image.type());
            image(marker_bounding_box).copyTo(sub_image);

            stag_instance.markers.clear();
            size_t num_markers = stag_instance.detectMarkers(sub_image);

            cv::Point2d corner_offset(marker_bounding_box.x, marker_bounding_box.y);

            for (auto& detected_marker : stag_instance.markers) {
                for (size_t corner_index = 0; corner_index < 4; ++ corner_index) {
                    detected_marker.corners[corner_index] = corner_offset + detected_marker.corners[corner_index];
                }
                detected_marker.center += corner_offset;
            }

            markers.insert(markers.end(), stag_instance.markers.begin(), stag_instance.markers.end());
        }

        previously_detected_markers = markers;
        return markers;
    }

    stag_instance.detectMarkers(image);
    previously_detected_markers = stag_instance.markers;

    return stag_instance.markers;
}


void StagNodelet::image_callback(const sensor_msgs::ImageConstPtr& image_message) {
    if (!have_camera_info) {
        ROS_WARN_STREAM("No camera info received yet. Cannot process image frame.");
        return;
    }

    auto image_frame_id = image_message->header.frame_id;
    auto image_time_stamp = image_message->header.stamp;

    auto cv_ptr = cv_bridge::toCvShare(image_message, sensor_msgs::image_encodings::MONO8);

    stag::Stag stag_instance(tag_id_type, 7, false);

    cv::Mat preprocessed_image;
    preprocess_image(cv_ptr->image, preprocessed_image);

    auto markers = detect_markers(preprocessed_image, stag_instance);
    auto num_tags = markers.size();

    std::sort(
        markers.begin(),
        markers.end(),
        [](const stag::Marker& a, const stag::Marker& b){
            return a.id < b.id;
        }
    );

    std::map<int, std::vector<cv::Point2d>> unrefined_corners;

    for (auto& marker : markers) {
        unrefined_corners[marker.id] = marker.corners;
    }

    refine_corners(markers, preprocessed_image);
    stag_ros::StagMarkers markers_message;

    if (num_tags > 0) {
        auto individual_marker_messages = get_transforms_for_individual_markers(markers, camera_to_output_frame, image_frame_id, image_time_stamp);
        markers_message.markers.insert(markers_message.markers.end(), individual_marker_messages.begin(), individual_marker_messages.end());

        if (use_marker_bundles) {
           auto bundled_marker_messages = get_transforms_for_bundled_markers(markers, camera_to_output_frame, image_frame_id, image_time_stamp);
           markers_message.markers.insert(markers_message.markers.end(), bundled_marker_messages.begin(), bundled_marker_messages.end());
        }
    }

    if (publish_debug_images) {
        do_publish_debug_images(
            image_time_stamp,
            preprocessed_image,
            markers_message,
            output_frame_id,
            image_message->header.seq,
            unrefined_corners
        );
    }

    markers_message.header.stamp = image_time_stamp;
    markers_message.header.seq = image_message->header.seq;
    markers_message.header.frame_id = output_frame_id;

    marker_message_mutex.lock();
    marker_message_publisher.publish(markers_message);
    marker_message_mutex.unlock();
}

void StagNodelet::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
    if (!have_camera_info) {
        cam_info_mutex.lock();
        have_camera_info = true;
        camera_matrix = (cv::Mat1d(3, 3) << camera_info_msg->K[0], camera_info_msg->K[1], camera_info_msg->K[2], camera_info_msg->K[3], camera_info_msg->K[4], camera_info_msg->K[5], camera_info_msg->K[6], camera_info_msg->K[7], camera_info_msg->K[8]);

        distortion_coefficients = (cv::Mat1d(5, 1) << camera_info_msg->D[0], camera_info_msg->D[1], camera_info_msg->D[2], camera_info_msg->D[3], camera_info_msg->D[4]);


        cv::initUndistortRectifyMap(camera_matrix, distortion_coefficients, cv::Mat(), camera_matrix, cv::Size(camera_info_msg->width, camera_info_msg->height), CV_32FC1, undistort_map1, undistort_map2);

        cam_info_mutex.unlock();
    }
}


void StagNodelet::parse_marker_sizes(ros::NodeHandle& private_node_handle) {
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


void StagNodelet::parse_marker_bundles(ros::NodeHandle& private_node_handle) {
    XmlRpc::XmlRpcValue marker_bundles_yaml;
    bool param_exists = private_node_handle.getParam("marker_bundles", marker_bundles_yaml);

    if (!param_exists) {
        ROS_INFO("No marker bundle provided. Aborting marker bundle parsing.");
        return;
    }

    for (size_t bundle_index = 0; bundle_index < marker_bundles_yaml.size(); ++bundle_index) {
        MarkerBundle bundle_parsed;
        auto bundle = marker_bundles_yaml[bundle_index];

        bundle_parsed.broadcasted_id = bundle["markers"][0]["id"];

        for (size_t marker_index = 0; marker_index < bundle["markers"].size(); ++marker_index) {
            auto marker_yaml = bundle["markers"][marker_index];

            int marker_id = marker_yaml["id"];
            bundle_parsed.corner_world_locations[marker_id] = std::vector<cv::Point3f>();

            for (size_t corner_index = 0; corner_index < marker_yaml["corners"].size(); ++corner_index) {
                auto corner_yaml = marker_yaml["corners"][corner_index];

                bundle_parsed.corner_world_locations[marker_id].push_back(
                    cv::Point3f(
                        static_cast<double>(corner_yaml[0]),
                        static_cast<double>(corner_yaml[1]),
                        static_cast<double>(corner_yaml[2])
                    )
                );
            }
        }
        marker_bundles.push_back(bundle_parsed);
    }

    ROS_INFO_STREAM("stag_ros parsed " << marker_bundles.size() << " marker bundles ");
}


void StagNodelet::onInit() {
    ROS_INFO("Initializing stag_ros");
    distortion_coefficients = cv::Mat(5, 1, CV_32FC1);
    use_marker_bundles = true;
    publish_debug_images = false;
    track_markers = false;

    ros::NodeHandle& private_node_handle = getMTPrivateNodeHandle();
    image_transport::ImageTransport _image_transport(private_node_handle);

    std::string camera_image_topic, camera_info_topic;

    private_node_handle.getParam("camera_image_topic", camera_image_topic);
    private_node_handle.getParam("camera_info_topic", camera_info_topic);
    private_node_handle.getParam("tag_id_type", tag_id_type);
    private_node_handle.getParam("marker_frame_prefix", marker_frame_prefix);
    private_node_handle.getParam("default_marker_size", default_marker_size);
    private_node_handle.getParam("output_frame_id", output_frame_id);
    private_node_handle.getParam("marker_message_topic", marker_message_topic);
    private_node_handle.getParam("publish_debug_images", publish_debug_images);
    private_node_handle.getParam("use_marker_bundles", use_marker_bundles);
    private_node_handle.getParam("save_debug_stream", save_debug_stream);
    private_node_handle.getParam("debug_stream_directory", debug_stream_directory);
    private_node_handle.getParam("track_markers", track_markers);
    private_node_handle.getParam("marker_track_height_offset", marker_track_height_offset);
    private_node_handle.getParam("marker_track_width_offset", marker_track_width_offset);

    if (save_debug_stream) {
        ROS_INFO_STREAM("Saving debug stream to directory " << debug_stream_directory);
    }

    std::string image_frame_id;
    private_node_handle.getParam("image_frame_id", image_frame_id);

    ROS_INFO("Parsing individual marker sizes");
    parse_marker_sizes(private_node_handle);

    ROS_INFO("Parsing marker bundles");

    ros::SubscribeOptions opts;
    this_ptr = boost::shared_ptr<void>(static_cast<void*>(&(this->_unused)));
    boost::function<void (const boost::shared_ptr<const sensor_msgs::Image>& )> f2( boost::bind( &StagNodelet::image_callback, this, _1 ) );
    opts = opts.template create<sensor_msgs::Image>(camera_image_topic, 1, f2, this_ptr, NULL);
    opts.allow_concurrent_callbacks = true;
    opts.transport_hints = ros::TransportHints();

    camera_info_subscriber = private_node_handle.subscribe(camera_info_topic, 1, &StagNodelet::camera_info_callback, this);

    marker_message_publisher = private_node_handle.advertise<stag_ros::StagMarkers>(marker_message_topic, 1);

    transform_broadcaster = new tf::TransformBroadcaster();
    transform_listener = new tf::TransformListener();

    ROS_INFO("Waiting for transformation from image to output frame...");
    transform_listener->waitForTransform(output_frame_id, image_frame_id, ros::Time(0), ros::Duration(30.0));
    ROS_INFO("Finished waiting for image to output frame transform");

    try {
        transform_listener->lookupTransform(output_frame_id, image_frame_id, ros::Time(0), camera_to_output_frame);
    } catch(tf::LookupException err) {
        ROS_ERROR("Could get transformation from camera to output frame. Cannot process image.");
        ROS_ERROR(err.what());

        throw err;
    }

    parse_marker_bundles(private_node_handle);
    ROS_INFO_STREAM("Marker bundles:");
    for (auto& bundle : marker_bundles) {
        ROS_INFO_STREAM("    " << bundle.broadcasted_id << " with " << bundle.corner_world_locations.size() << " tags.");
    }
    ROS_INFO_STREAM("End marker bundles");

    ROS_INFO_STREAM("Got marker sizes");
    for (auto& key_value : marker_size_by_id) {
        ROS_INFO_STREAM("marker id " << key_value.first << " size " << key_value.second);
    }
    ROS_INFO_STREAM("End marker sizes");

    if (publish_debug_images) {
        image_transport_ptr = std::unique_ptr<image_transport::ImageTransport>(
            new image_transport::ImageTransport(private_node_handle)
        );
        
        undistorted_image_publisher = image_transport_ptr->advertise(
            "debug/undistorted_image",
            1
        );

        optimized_corner_image_publisher = image_transport_ptr->advertise(
            "debug/optimized_corners",
            1
        );
    }

    image_subscriber = private_node_handle.subscribe(opts);
}

} // namespace stag_ros
