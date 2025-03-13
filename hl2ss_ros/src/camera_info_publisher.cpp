#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/Image.h>

// Global variables needed for callback
ros::Publisher* info_pub_ptr;
camera_info_manager::CameraInfoManager* camera_info_manager_ptr;

void cameraInfoCallback(const sensor_msgs::Image::ConstPtr& msg) {
    if (info_pub_ptr && camera_info_manager_ptr) {
        sensor_msgs::CameraInfo camera_info = camera_info_manager_ptr->getCameraInfo();
        camera_info.header = msg->header;  // Use same header as image
        info_pub_ptr->publish(camera_info);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_info_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Get parameters
    std::string ag_n;
    std::string camera_info_path;
    private_nh.param<std::string>("ag_n", ag_n, "0");
    private_nh.param<std::string>("camera_info_path", camera_info_path, "");

    std::string camera_name = "hololens_ag" + ag_n;
    std::string topic_name = "hololens_ag" + ag_n + "/camera_info";

    // Create publisher
    ros::Publisher info_pub = nh.advertise<sensor_msgs::CameraInfo>(topic_name, 10);
    info_pub_ptr = &info_pub;

    // Setup camera info manager
    camera_info_manager::CameraInfoManager camera_info_manager(nh, camera_name, camera_info_path);
    camera_info_manager_ptr = &camera_info_manager;

    //Load the calibration data
    camera_info_manager.loadCameraInfo(camera_info_path);

    // Check if the camera is calibrated
    if (camera_info_manager.isCalibrated()) {
        ROS_INFO_STREAM("Camera is calibrated.");
    } else {
        ROS_WARN_STREAM("Camera is not calibrated.");
        return 1;
    }

    // Create subscriber after setting up globals
    ros::Subscriber sub = nh.subscribe(camera_name + "/vlc_image", 10, &cameraInfoCallback);

    ROS_INFO_STREAM("Publishing VLC camera info to topic '" << topic_name << "'...");

    ros::spin();

    return 0;
}