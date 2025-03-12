#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

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

    // Setup camera info manager
    camera_info_manager::CameraInfoManager camera_info_manager(nh, camera_name, camera_info_path);

    //Load the calibration data
    camera_info_manager.loadCameraInfo(camera_info_path);

    // Check if the camera is calibrated
    if (camera_info_manager.isCalibrated()) {
        ROS_INFO_STREAM("Camera is calibrated.");
    } else {
        ROS_WARN_STREAM("Camera is not calibrated.");
        return 1;
    }

    ros::Rate rate(30); // 10 Hz
    ROS_INFO_STREAM("Publishing VLC camera info to topic '" << topic_name << "'...");

    while (ros::ok()) {
        sensor_msgs::CameraInfo camera_info = camera_info_manager.getCameraInfo();
        camera_info.header.stamp = ros::Time::now();

        info_pub.publish(camera_info);
        rate.sleep();
    }

    return 0;
}