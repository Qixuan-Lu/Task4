#ifndef HIK_CAMERA_NODE_HPP
#define HIK_CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include "MvCameraControl.h"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <memory>
#include <string>
#include <thread>
#include <atomic>

class HikCameraNode : public rclcpp::Node
{
public:
    explicit HikCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~HikCameraNode();

private:
    // ROS2相关
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    image_transport::CameraPublisher camera_pub_;
    
    // 海康相机相关
    void* camera_handle_;
    std::atomic<bool> is_grabbing_;
    std::thread grab_thread_;
    
    // 参数
    std::string camera_name_;
    std::string camera_ip_;
    std::string camera_serial_;
    std::string camera_info_url_;
    double frame_rate_;
    double exposure_time_;
    double gain_;
    std::string pixel_format_;
    
    // 方法
    void initialize();
    bool initialize_camera();
    void start_grabbing();
    void stop_grabbing();
    void grab_loop();
    bool set_camera_parameters();
    bool reconnect_camera();
    rclcpp::TimerBase::SharedPtr initialize_timer_;
    
    // 参数回调
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> & parameters);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

#endif