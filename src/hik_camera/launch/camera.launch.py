import launch
from launch import LaunchDescription  # 用于创建启动描述的类
from launch_ros.actions import Node  # 用于创建ROS节点的类
from launch.actions import DeclareLaunchArgument, ExecuteProcess  # 用于声明启动参数和执行进程的类
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  # 用于处理启动配置和路径拼接的类
from launch_ros.substitutions import FindPackageShare  # 用于查找ROS包共享目录的类

def generate_launch_description():
    # 定义RViz配置文件路径的启动参数
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',  # 参数名称
        default_value='camera_display.rviz',  # 默认值：RViz配置文件名
        description='RViz配置文件的名称'  # 参数描述
    )
    
    # 定义是否启动RViz2的启动参数
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',  # 参数名称
        default_value='true',  # 默认值：启动RViz2
        description='是否启动RViz2'  # 参数描述
    )
    
    # 返回启动描述，包含所有要启动的组件
    return LaunchDescription([
        rviz_config_arg,  # 添加RViz配置文件参数
        use_rviz_arg,     # 添加是否启动RViz参数
        
        # 启动海康相机节点
        Node(
            package='hik_camera',  # 节点所属的包
            executable='hik_camera_node',  # 要执行的可执行文件
            name='hik_camera_node',  # 节点名称
            parameters=[{  # 节点参数配置
                'camera_serial': '00F26632041',  # 相机序列号
                'camera_ip': '',  # 相机IP（为空时使用序列号连接）
                'frame_rate': 30.0,  # 帧率（30帧/秒）
                'exposure_time': 10000.0,  # 曝光时间
                'gain': 5.0,  # 增益
                'pixel_format': 'BGR8'  # 像素格式（BGR8格式）
            }],
            output='screen'  # 输出信息到终端
        ),
        
        # 启动RViz2节点（根据use_rviz参数决定是否启动）
        Node(
            package='rviz2',  # RViz2所属的包
            executable='rviz2',  # 要执行的可执行文件
            name='rviz2',  # 节点名称
            # 加载指定的RViz配置文件，路径由包共享目录+config文件夹+配置文件名组成
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('hik_camera'),  # 查找hik_camera包的共享目录
                'config',  # 配置文件所在文件夹
                LaunchConfiguration('rviz_config')  # 使用前面定义的rviz_config参数作为文件名
            ])],
            condition=launch.conditions.IfCondition(LaunchConfiguration('use_rviz')),  # 启动条件：当use_rviz为true时启动
            output='screen'  # 输出信息到终端
        )
    ])