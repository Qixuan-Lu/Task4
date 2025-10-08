#include "hik_camera/hik_camera_node.hpp"
using namespace std::chrono_literals;  // 使用时间字面量（如100ms、2s等）

/**
 * @brief HikCameraNode类构造函数
 * @param options 节点选项参数
 * 初始化节点名称，声明并获取相机相关参数，创建初始化定时器
 */
HikCameraNode::HikCameraNode(const rclcpp::NodeOptions & options)
: Node("hik_camera_node", options),
  camera_handle_(nullptr),  // 相机句柄初始化为空
  is_grabbing_(false)       // 采集状态初始化为未采集
{
    // 声明节点参数（参数名，默认值）
    this->declare_parameter("camera_name", "hik_camera");       // 相机名称
    this->declare_parameter("camera_ip", "192.168.1.10");       // 相机IP地址
    this->declare_parameter("camera_serial", "");               // 相机序列号
    this->declare_parameter("frame_rate", 30.0);                // 帧率
    this->declare_parameter("exposure_time", 10000.0);          // 曝光时间
    this->declare_parameter("gain", 5.0);                       // 增益
    this->declare_parameter("pixel_format", "BGR8");            // 像素格式
    this->declare_parameter("camera_info_url", "");             // 相机信息URL（校准文件路径）
    
    // 获取参数值到成员变量
    this->get_parameter("camera_name", camera_name_);
    this->get_parameter("camera_ip", camera_ip_);
    this->get_parameter("camera_serial", camera_serial_);
    this->get_parameter("frame_rate", frame_rate_);
    this->get_parameter("exposure_time", exposure_time_);
    this->get_parameter("gain", gain_);
    this->get_parameter("pixel_format", pixel_format_);
    this->get_parameter("camera_info_url", camera_info_url_);
    
    // 打印获取到的参数信息
    RCLCPP_INFO(this->get_logger(), "camera_ip: %s", camera_ip_.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_serial: %s", camera_serial_.c_str());
    RCLCPP_INFO(this->get_logger(), "exposure_time: %f", exposure_time_);
    RCLCPP_INFO(this->get_logger(), "gain: %f", gain_);
    RCLCPP_INFO(this->get_logger(), "frame_rate: %f", frame_rate_);

    // 创建初始化定时器（100ms后执行initialize函数，仅执行一次）
    initialize_timer_ = this->create_wall_timer(
        100ms, std::bind(&HikCameraNode::initialize, this));
}

/**
 * @brief 初始化函数
 * 取消初始化定时器，初始化图像传输、相机信息管理器，创建发布器，设置参数回调，初始化并启动相机
 */
void HikCameraNode::initialize()
{
    initialize_timer_->cancel();  // 取消初始化定时器（避免重复执行）

    // 初始化图像传输器（用于发布图像消息）
    image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

    // 初始化相机信息管理器（用于加载相机内参等信息）
    camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name_, camera_info_url_);
    
    // 创建相机图像发布器（发布图像和相机信息，队列长度10）
    camera_pub_ = image_transport_->advertiseCamera("image_raw", 10);
    
    // 设置参数回调函数（处理动态参数更新）
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&HikCameraNode::parameters_callback, this, std::placeholders::_1));

    // 初始化相机，成功则启动采集，失败则关闭节点
    if (initialize_camera()) {
        RCLCPP_INFO(this->get_logger(), "相机初始化成功");
        start_grabbing();
    } else {
        RCLCPP_ERROR(this->get_logger(), "初始化相机失败");
        rclcpp::shutdown();
    }
}

/**
 * @brief 析构函数
 * 停止图像采集，销毁相机句柄释放资源
 */
HikCameraNode::~HikCameraNode()
{
    stop_grabbing();  // 停止采集
    if (camera_handle_) {  // 若相机句柄存在，销毁句柄
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
    }
}

/**
 * @brief 初始化相机
 * 枚举设备，根据IP或序列号选择相机，创建并打开相机句柄，设置相机参数
 * @return 初始化成功返回true，失败返回false
 */
bool HikCameraNode::initialize_camera()
{
    int nRet = MV_OK;  // 海康SDK返回值（MV_OK表示成功）
    
    MV_CC_DEVICE_INFO_LIST stDeviceList;  // 设备列表结构体
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));  // 初始化结构体
    
    // 枚举设备（支持GigE和USB设备）
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "枚举设备失败! 错误码 [0x%x]", nRet);
        return false;
    }
    
    // 检查是否找到设备
    if (stDeviceList.nDeviceNum == 0) {
        RCLCPP_ERROR(this->get_logger(), "未找到任何设备!");
        return false;
    }
    
    unsigned int nIndex = 0;  // 选中的设备索引
    bool found = false;       // 是否找到目标设备
    
    // 遍历设备列表，根据IP或序列号选择设备
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
        MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];  // 设备信息
        
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {  // GigE相机
            char strIp[64] = {0};
            // 解析IP地址（从32位整数转换为点分十进制）
            sprintf(strIp, "%u.%u.%u.%u", 
                    pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xFF,
                    (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp >> 8) & 0xFF,
                    (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp >> 16) & 0xFF,
                    (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp >> 24) & 0xFF);
            
            RCLCPP_INFO(this->get_logger(), "找到相机IP: %s", strIp);
            
            // 若IP匹配，或未指定序列号且为第一个设备，则选中该设备
            if (camera_ip_ == strIp || (camera_serial_.empty() && i == 0)) {
                nIndex = i;
                found = true;
                RCLCPP_INFO(this->get_logger(), "选中相机IP: %s", strIp);
                break;
            }
        } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {  // USB相机
            if (!camera_serial_.empty()) {  // 若指定了序列号
                std::string serial_num = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
                if (camera_serial_ == serial_num) {  // 序列号匹配
                    nIndex = i;
                    found = true;
                    RCLCPP_INFO(this->get_logger(), "选中相机序列号: %s", serial_num.c_str());
                    break;
                }
            }
        }
    }
    
    // 若未找到目标设备，使用第一个可用设备
    if (!found && stDeviceList.nDeviceNum > 0) {
        RCLCPP_WARN(this->get_logger(), "未找到指定相机，使用第一个可用相机");
        nIndex = 0;
        found = true;
    }
    
    // 仍未找到设备，返回失败
    if (!found) {
        RCLCPP_ERROR(this->get_logger(), "未找到IP为: %s 或序列号为: %s 的相机", 
                    camera_ip_.c_str(), camera_serial_.c_str());
        return false;
    }
    
    // 创建相机句柄
    nRet = MV_CC_CreateHandle(&camera_handle_, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "创建句柄失败! 错误码 [0x%x]", nRet);
        return false;
    }
    
    // 打开相机设备
    nRet = MV_CC_OpenDevice(camera_handle_);
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "打开设备失败! 错误码 [0x%x]", nRet);
        MV_CC_DestroyHandle(camera_handle_);  // 销毁句柄释放资源
        camera_handle_ = nullptr;
        return false;
    }
    
    // 设置相机参数（帧率、曝光等）
    return set_camera_parameters();
}

/**
 * @brief 设置相机参数
 * 配置触发模式、帧率、曝光时间、增益、像素格式等参数
 * @return 参数设置成功返回true，关键参数失败返回false
 */
bool HikCameraNode::set_camera_parameters()
{
    int nRet = MV_OK;
    
    // 设置触发模式为连续采集（0表示关闭触发，即连续采集）
    nRet = MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "设置触发模式失败! 错误码 [0x%x]", nRet);
        return false;
    }
    
    // 设置帧率
    nRet = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", frame_rate_);
    if (MV_OK != nRet) {
        RCLCPP_WARN(this->get_logger(), "设置帧率失败! 错误码 [0x%x]", nRet);
    }
    
    // 设置曝光时间
    nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time_);
    if (MV_OK != nRet) {
        RCLCPP_WARN(this->get_logger(), "设置曝光时间失败! 错误码 [0x%x]", nRet);
    }
    
    // 设置增益
    nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", gain_);
    if (MV_OK != nRet) {
        RCLCPP_WARN(this->get_logger(), "设置增益失败! 错误码 [0x%x]", nRet);
    }
    
    // 设置像素格式
    if (pixel_format_ == "BGR8") {
        nRet = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_BGR8_Packed);
    } else if (pixel_format_ == "RGB8") {
        nRet = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_RGB8_Packed);
    } else if (pixel_format_ == "Mono8") {
        nRet = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_Mono8);
    } else {
        RCLCPP_WARN(this->get_logger(), "不支持的像素格式: %s，使用BGR8", pixel_format_.c_str());
        nRet = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_BGR8_Packed);
    }
    
    if (MV_OK != nRet) {
        RCLCPP_WARN(this->get_logger(), "设置像素格式失败! 错误码 [0x%x]", nRet);
    }
    
    // 打印参数设置结果
    RCLCPP_INFO(this->get_logger(), "相机参数已设置: %.1f FPS, %.1f 曝光时间, %.1f 增益, %s 格式",
                frame_rate_, exposure_time_, gain_, pixel_format_.c_str());
    
    return true;
}

/**
 * @brief 开始图像采集
 * 启动相机采集，创建采集线程执行采集循环
 */
void HikCameraNode::start_grabbing()
{
    // 启动相机采集
    int nRet = MV_CC_StartGrabbing(camera_handle_);
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "启动采集失败! 错误码 [0x%x]", nRet);
        return;
    }
    
    is_grabbing_ = true;  // 更新采集状态
    // 创建采集线程，执行grab_loop函数
    grab_thread_ = std::thread(&HikCameraNode::grab_loop, this);
    RCLCPP_INFO(this->get_logger(), "开始图像采集");
}

/**
 * @brief 停止图像采集
 * 停止采集线程，关闭相机采集
 */
void HikCameraNode::stop_grabbing()
{
    is_grabbing_ = false;  // 标记停止采集
    if (grab_thread_.joinable()) {  // 等待采集线程结束
        grab_thread_.join();
    }
    
    if (camera_handle_) {  // 停止相机采集
        MV_CC_StopGrabbing(camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "停止图像采集");
}

/**
 * @brief 图像采集循环
 * 持续从相机获取图像，转换为ROS图像消息并发布，处理采集过程中的错误
 */
void HikCameraNode::grab_loop()
{
    MV_FRAME_OUT stImageInfo;  // 图像帧信息结构体
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));  // 初始化结构体
    int nRet = MV_OK;
    
    // 循环采集（节点运行且采集状态为true）
    while (rclcpp::ok() && is_grabbing_) {
        // 获取图像缓冲区（超时时间1000ms）
        nRet = MV_CC_GetImageBuffer(camera_handle_, &stImageInfo, 1000);
        if (nRet == MV_OK) {  // 成功获取图像
            // 创建ROS图像消息
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = this->now();  // 设置时间戳
            msg->header.frame_id = camera_name_ + "_optical_frame";  // 设置坐标系ID
            msg->height = stImageInfo.stFrameInfo.nHeight;  // 图像高度
            msg->width = stImageInfo.stFrameInfo.nWidth;    // 图像宽度
            
            // 根据像素格式设置消息编码和步长（一行数据的字节数）
            if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_BGR8_Packed) {
                msg->encoding = "bgr8";
                msg->step = stImageInfo.stFrameInfo.nWidth * 3;  // 每个像素3字节（BGR）
            } else if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed) {
                msg->encoding = "rgb8";
                msg->step = stImageInfo.stFrameInfo.nWidth * 3;
            } else if (stImageInfo.stFrameInfo.enPixelType == PixelType_Gvsp_Mono8) {
                msg->encoding = "mono8";
                msg->step = stImageInfo.stFrameInfo.nWidth;  // 每个像素1字节（单通道）
            } else {
                msg->encoding = "bgr8";
                msg->step = stImageInfo.stFrameInfo.nWidth * 3;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                   "不支持的像素格式: 0x%lx，使用bgr8", 
                                   stImageInfo.stFrameInfo.enPixelType);
            }
            
            msg->is_bigendian = 0;  // 小端模式
            
            // 复制图像数据到消息
            size_t image_size = stImageInfo.stFrameInfo.nFrameLen;  // 图像数据长度
            msg->data.resize(image_size);
            memcpy(msg->data.data(), stImageInfo.pBufAddr, image_size);
            
            // 获取相机信息并发布（图像+相机信息）
            auto camera_info = camera_info_manager_->getCameraInfo();
            camera_info.header = msg->header;  // 相机信息时间戳与图像一致
            camera_pub_.publish(*msg, camera_info);
            
            // 释放图像缓冲区（必须调用，否则内存泄漏）
            MV_CC_FreeImageBuffer(camera_handle_, &stImageInfo);
        } else if (nRet == static_cast<int>(MV_E_NODATA)) {  // 无数据（正常超时）
            continue;
        } else {  // 其他错误（每5秒打印一次警告）
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                               "获取图像缓冲区失败: 0x%x", nRet);
        }
    }
}

/**
 * @brief 重新连接相机
 * 停止当前采集，关闭并销毁句柄，重试连接相机（最多5次）
 * @return 重连成功返回true，失败返回false
 */
bool HikCameraNode::reconnect_camera()
{
    stop_grabbing();  // 停止当前采集
    
    // 关闭并销毁相机句柄
    if (camera_handle_) {
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
    }
    
    // 重试连接（最多5次）
    for (int i = 0; i < 5; ++i) {
        RCLCPP_INFO(this->get_logger(), "尝试重新连接相机（第 %d/5 次）", i + 1);
        
        if (initialize_camera()) {  // 初始化成功则启动采集
            start_grabbing();
            RCLCPP_INFO(this->get_logger(), "相机重新连接成功");
            return true;
        }
        
        std::this_thread::sleep_for(2s);  // 间隔2秒重试
    }
    
    RCLCPP_ERROR(this->get_logger(), "相机重新连接失败");
    return false;
}

/**
 * @brief 参数回调函数
 * 处理动态参数更新（曝光时间、增益、帧率、像素格式）
 * @param parameters 更新的参数列表
 * @return 参数设置结果
 */
rcl_interfaces::msg::SetParametersResult HikCameraNode::parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;  // 默认设置成功
    
    for (const auto & param : parameters) {  // 遍历更新的参数
        if (param.get_name() == "exposure_time") {  // 曝光时间更新
            exposure_time_ = param.as_double();
            int nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time_);
            if (MV_OK == nRet) {
                RCLCPP_INFO(this->get_logger(), "曝光时间已设置为: %f", exposure_time_);
            } else {
                RCLCPP_WARN(this->get_logger(), "设置曝光时间失败: 0x%x", nRet);
            }
        } else if (param.get_name() == "gain") {  // 增益更新
            gain_ = param.as_double();
            int nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", gain_);
            if (MV_OK == nRet) {
                RCLCPP_INFO(this->get_logger(), "增益已设置为: %f", gain_);
            } else {
                RCLCPP_WARN(this->get_logger(), "设置增益失败: 0x%x", nRet);
            }
        } else if (param.get_name() == "frame_rate") {  // 帧率更新
            frame_rate_ = param.as_double();
            int nRet = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", frame_rate_);
            if (MV_OK == nRet) {
                RCLCPP_INFO(this->get_logger(), "帧率已设置为: %f", frame_rate_);
            } else {
                RCLCPP_WARN(this->get_logger(), "设置帧率失败: 0x%x", nRet);
            }
        } else if (param.get_name() == "pixel_format") {  // 像素格式更新（需重启相机）
            pixel_format_ = param.as_string();
            RCLCPP_WARN(this->get_logger(), "像素格式更改需要重启相机");
        }
    }
    
    return result;
}