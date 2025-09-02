import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # 获取参数文件的路径
    params_file = os.path.join(
        get_package_share_directory('drone_bringup'), # 注意：请将'drone_bringup'替换为您的功能包名
        'params',
        'rtabmap_localization_params.yaml'
    )

    # 话题重映射列表
    rtabmap_remaps = [
        ('rgb/image', '/D400/color/image_raw'),
        ('rgb/camera_info', '/D400/color/camera_info'),
        ('depth/image', '/D400/aligned_depth_to_color/image_raw'),
        ('odom', '/T265/pose/sample'), # ❗【重要】应使用Odometry类型的消息
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),

        # RTAB-Map 主节点
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            # 直接加载YAML文件作为参数
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=rtabmap_remaps,
            output='screen',
        ),

        # RTAB-Map 可视化节点
        # Node(
        #     package='rtabmap_viz',
        #     executable='rtabmap_viz',
        #     name='rtabmap_viz',
        #     # 可视化节点也加载相同的参数
        #     parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        #     remappings=rtabmap_remaps,
        #     output='screen',
        # )
    ])
