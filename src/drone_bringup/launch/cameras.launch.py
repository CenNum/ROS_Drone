# 文件路径: ~/ros2_ws/src/drone_bringup/launch/drone_sensors.launch.py
# [已修改] 強制對齊深度並匹配實際話題名稱

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    realsense_ros_dir = get_package_share_directory('realsense2_camera')
    drone_description_dir = get_package_share_directory('drone_description')


    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_ros_dir, 'launch', 'rs_multi_camera_launch.py')
        ),
        launch_arguments={
            # --- D435/D400 series camera ---
            'camera_name1': 'D400',           
            'serial_no1': '_152122076395',     # D435的序列号 _146322077649 _152122076395
            'depth_module.profile1': '640,480,6',
            'rgb_camera.profile1': '640,480,6',
            'enable_depth1': 'true',
            'align_depth.enable1': 'true',    
            'pointcloud.enable1': 'true',

            # --- T265 camera ---
            'camera_name2': 'T265',            
            'serial_no2': '_230322110723',     # T265的序列号 _230322111176 _230322110723
            'enable_pose2': 'true',
            'enable_fisheye12': 'false',
            'enable_fisheye22': 'false',
            'publish_odom_tf2': 'true',
        }.items()
    )

    drone_display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(drone_description_dir, 'launch', 'display.launch.py')
        )
    )


    static_tf_T265_link_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_T265_link_to_base_footprint',
        arguments=[
            '-0.16', '0.0', '-0.02', '0.0', '0.0', '0.0',
            'T265_link', 'base_footprint'  # [修改] 匹配相機名稱
        ]
    )

    static_tf_odom_to_odom_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_odom_frame',
        arguments=[
            '0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
            'odom', 'odom_frame'  
        ]
    )

    return LaunchDescription([
        sensors_launch,
        drone_display_launch,
        static_tf_T265_link_to_base_footprint,
        static_tf_odom_to_odom_frame,
    ])
   
