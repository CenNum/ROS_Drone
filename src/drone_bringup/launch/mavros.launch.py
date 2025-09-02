import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
    # 获取 MAVROS 包的路径
    mavros_launch_dir = os.path.join(get_package_share_directory('mavros'), 'launch')
    pkg_drone_bringup = get_package_share_directory('drone_bringup')

    # 设置 FCU URL 和参数文件路径
    fcu_url_value = '/dev/ttyUSB0:921600'  # 默认的 FCU URL
    mavros_params_file = os.path.join(pkg_drone_bringup, 'params', 'apm_config.yaml')
    pluginlists_yaml_file = os.path.join(pkg_drone_bringup, 'params', 'apm_pluginlists.yaml')

    # 声明 Launch 配置项
    fcu_url = LaunchConfiguration('fcu_url', default=fcu_url_value)
    gcs_url = LaunchConfiguration('gcs_url', default='udp://@10.0.0.112:14550')
    tgt_system = LaunchConfiguration('tgt_system', default='1')
    tgt_component = LaunchConfiguration('tgt_component', default='1')
    log_output = LaunchConfiguration('log_output', default='screen')
    fcu_protocol = LaunchConfiguration('fcu_protocol', default='v2.0')
    respawn_mavros = LaunchConfiguration('respawn_mavros', default='false')
    namespace = LaunchConfiguration('namespace', default='mavros')

    # 引入 MAVROS 的 node.launch 文件
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(mavros_launch_dir, 'node.launch')
        ),
        launch_arguments={
            'fcu_url': fcu_url,
            'pluginlists_yaml': pluginlists_yaml_file,
            'config_yaml': mavros_params_file,
            'gcs_url': gcs_url,
            'tgt_system': tgt_system,
            'tgt_component': tgt_component,
            'fcu_protocol': fcu_protocol,
            'respawn_mavros': respawn_mavros,
            #'namespace': namespace
        }.items()
    )

    t265_pose_relay_node = Node(
        package='drone_utilities',
        executable='odom_to_pose_converter',
        name='t265_pose_relay',
        output='screen'
    )


    # 返回 LaunchDescription，包含所有的参数声明和 MAVROS 启动
    return LaunchDescription([
        # 声明必要的 Launch 参数
        DeclareLaunchArgument('fcu_url', default_value=fcu_url_value, description="FCU URL"),
        DeclareLaunchArgument('gcs_url', default_value='udp://@10.0.0.112:14550', description="GCS URL"),
        DeclareLaunchArgument('tgt_system', default_value='1', description="Target system ID"),
        DeclareLaunchArgument('tgt_component', default_value='1', description="Target component ID"),
        DeclareLaunchArgument('log_output', default_value='screen', description="Log output destination"),
        DeclareLaunchArgument('fcu_protocol', default_value='v2.0', description="FCU Protocol version"),
        DeclareLaunchArgument('respawn_mavros', default_value='false', description="Whether to respawn MAVROS"),
        DeclareLaunchArgument('namespace', default_value='mavros', description="Namespace for MAVROS"),
        
        # 包含 MAVROS 启动文件
        mavros_launch,
        t265_pose_relay_node
    ])
