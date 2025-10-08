# 指定CMake的最低版本要求为3.8
cmake_minimum_required(VERSION 3.8)
# 设置项目名称为hik_camera
project(hik_camera)

# 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17)
# 要求必须使用指定的C++标准，不允许降级
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 禁用编译器特定的C++扩展
set(CMAKE_CXX_EXTENSIONS OFF)

# 如果使用GNU C++编译器或Clang编译器
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # 添加编译选项：启用所有警告、额外警告和严格模式
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
# 查找ament_cmake构建系统
find_package(ament_cmake REQUIRED)
# 查找ROS 2 C++客户端库
find_package(rclcpp REQUIRED)
# 查找传感器消息包
find_package(sensor_msgs REQUIRED)
# 查找cv_bridge（用于ROS消息与OpenCV图像转换）
find_package(cv_bridge REQUIRED)
# 查找image_transport（用于图像传输）
find_package(image_transport REQUIRED)
# 查找camera_info_manager（用于相机信息管理）
find_package(camera_info_manager REQUIRED)
# 查找OpenCV库
find_package(OpenCV REQUIRED)

# 查找海康威视MVS SDK的头文件目录
find_path(MVS_INCLUDE_DIR
  NAMES MvCameraControl.h  # 要查找的头文件
  PATHS /opt/MVS/include  # 查找路径
  NO_DEFAULT_PATH         # 不搜索默认路径
)

# 查找海康威视MVS SDK的库文件
find_library(MVS_LIBRARY
  NAMES MvCameraControl    # 要查找的库名称
  PATHS /opt/MVS/lib/64    # 查找路径（64位库）
  NO_DEFAULT_PATH         # 不搜索默认路径
)

# 检查是否找到MVS SDK
if(MVS_INCLUDE_DIR AND MVS_LIBRARY)
  # 找到SDK时输出状态信息
  message(STATUS "找到海康威视MVS SDK: ${MVS_INCLUDE_DIR}, ${MVS_LIBRARY}")
else()
  # 未找到SDK时输出错误并终止构建
  message(FATAL_ERROR "未找到海康威视MVS SDK!")
endif()

# 如果启用测试构建
if(BUILD_TESTING)
  # 查找ament_lint_auto（用于代码检查）
  find_package(ament_lint_auto REQUIRED)
  # 以下行跳过检查版权的linter
  # 当所有源文件都添加了版权和许可证信息时，请注释掉此行
  set(ament_cmake_copyright_FOUND TRUE)
  # 以下行跳过cpplint检查（仅在git仓库中有效）
  # 当此包位于git仓库中且所有源文件都添加了版权和许可证信息时，请注释掉此行
  set(ament_cmake_cpplint_FOUND TRUE)
  # 自动查找测试依赖并运行所有检查
  ament_lint_auto_find_test_dependencies()
endif()

# 添加可执行文件
add_executable(hik_camera_node
  src/hik_camera_node.cpp  # 节点实现文件
  src/main.cpp             # 主函数文件
)

# 设置hik_camera_node的头文件包含目录
target_include_directories(hik_camera_node PRIVATE
  include                  # 项目自身的头文件目录
  ${MVS_INCLUDE_DIR}       # MVS SDK的头文件目录
)

# 为hik_camera_node链接所需的库
target_link_libraries(hik_camera_node
  cv_bridge::cv_bridge                  # cv_bridge库
  image_transport::image_transport      # image_transport库
  camera_info_manager::camera_info_manager  # camera_info_manager库
  rclcpp::rclcpp                        # rclcpp库
  ${sensor_msgs_LIBRARIES}              # sensor_msgs库
  ${MVS_LIBRARY}                        # MVS SDK库
  ${OpenCV_LIBRARIES}                   # OpenCV库
)

# 安装目标文件
install(TARGETS
  hik_camera_node         # 要安装的可执行文件
  DESTINATION lib/${PROJECT_NAME}  # 安装路径（lib/项目名）
)

# 安装launch文件
install(DIRECTORY
  launch                  # 要安装的launch目录
  DESTINATION share/${PROJECT_NAME}  # 安装路径（share/项目名）
)

# 安装配置文件
install(DIRECTORY
  config                  # 要安装的config目录
  DESTINATION share/${PROJECT_NAME}  # 安装路径（share/项目名）
)

# 声明ament包（ROS 2包的必要收尾命令）
ament_package()