#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "Stag.h"

Stag* stag;

void image_callback(const sensor_msgs::ImageConstPtr& image_message) {
    auto cv_ptr = cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::MONO8);
    stag->detectMarkers(cv_ptr->image);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "stag_ros_test");

    ros::NodeHandle node_handle;
    image_transport::ImageTransport _image_transport(node_handle);

    auto image_subscriber = _image_transport.subscribe("/cv_camera/image_raw", 1, image_callback);

    stag = new Stag(15, 7, false);

    ros::spin();

    delete stag;
}
