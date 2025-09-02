# ======================================================================================
# 全功能导航启动文件 (RTAB-Map 定位 + Nav2 导航)
#
# 功能:
# 1. 启动RTAB-Map进行定位，并由它发布 /map 话题和 map -> odom 的TF变换。
# 2. 启动Nav2的核心导航服务器（规划器、控制器等），但不启动其自带的amcl和map_server。
# 3. 启动必要的辅助节点，如深度图转LaserScan。
# ======================================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. 定义路径和配置文件 ---
    
    # 获取您自己的功能包路径 ('drone_bringup')
    pkg_drone_bringup = get_package_share_directory('drone_bringup')
    
    # 获取Nav2官方bringup包的路径
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # 我们精心配置好的Nav2参数文件
    nav2_params_file = os.path.join(pkg_drone_bringup, 'params', 'nav2_params.yaml')

    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # --- 2. 定义要启动的组件 ---

    # 组件A: 启动我们配置好的RTAB-Map定位节点
    # 这个launch文件负责提供定位(map->odom)和地图(/map)
    rtabmap_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_drone_bringup, 'launch', 'rtabmap_localization.launch.py')
        ),
        # 如果rtabmap_localization.launch.py也需要use_sim_time, 在这里传入
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 组件B: 启动Nav2的核心导航功能
    # 重要: 我们调用 navigation_launch.py 而不是 bringup_launch.py
    # 因为前者不包含定位模块(amcl, map_server)，正好符合我们的需求。
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',  # 自动启动Nav2所有节点
        }.items()
    )
    
    # 组件C: 启动深度图转LaserScan节点，为Nav2的局部代价地图提供数据
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            # ❗【重要】请再次确认这两个话题名与您的摄像头驱动发布的一致
            ('image', '/D400/aligned_depth_to_color/image_raw'),
            ('camera_info', '/D400/color/camera_info'),
            ('scan', '/scan') # Nav2默认监听/scan话题
        ],
        parameters=[{
            'output_frame': 'base_footprint', # ❗【重要】应与RTAB-Map配置中的frame_id一致
            'range_min': 0.3, # 根据无人机安全距离设置
            'range_max': 5.0, # 根据D435有效范围设置
            'scan_height': 10, # 在图像中心取10个像素高进行转换，减少计算量
            'queue_size': 5,
        }]
    )
    cmd_vel_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        # ROS 2的relay节点使用位置参数：[输入话题] [输出话题]
        arguments=['/cmd_vel', '/mavros/setpoint_velocity/cmd_vel_unstamped']
    )
    
    # ... 您还可以在这里添加其他节点，例如MAVROS的启动 ...

    # --- 3. 组合所有组件并返回 ---
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # 按顺序启动
        rtabmap_localization_launch,
        nav2_navigation_launch,
        depth_to_scan_node,
        cmd_vel_relay_node,
    ])
