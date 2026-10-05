# AMR 매핑 통합 런치 — static TF x2, 라이다, 브릿지, rf2o, EKF, slam
# 사용: ros2 launch /home/daehan/ros2_ws/launch/amr_mapping.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

HOME = os.path.expanduser('~')

def generate_launch_description():
    turn_gain = LaunchConfiguration('turn_gain')
    cmd_timeout = LaunchConfiguration('cmd_timeout')

    return LaunchDescription([
        DeclareLaunchArgument('turn_gain', default_value='5.0',
                              description='teleop 수동주행 5.0 / Nav2 1.0'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.8'),

        # --- static TF ---
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_base_laser',
             arguments=['--x','0.02','--y','0','--z','0.20',
                        '--yaw','3.14159','--pitch','0','--roll','0',
                        '--frame-id','base_link','--child-frame-id','laser']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_base_imu',
             arguments=['--x','0','--y','0','--z','0',
                        '--yaw','0','--pitch','0','--roll','0',
                        '--frame-id','base_link','--child-frame-id','imu_link']),

        # --- 라이다 ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch', 'sllidar_a1_launch.py')),
            launch_arguments={'serial_port': '/dev/rplidar',
                              'frame_id': 'laser'}.items()),

        # --- 브릿지 (★gyro_scale 65.5 = ±500dps) ---
        Node(package='amr_bridge', executable='bridge_node', name='amr_bridge',
             output='screen',
             parameters=[{'turn_gain': turn_gain,
                          'cmd_timeout': cmd_timeout,
                          'gyro_scale': 65.5,
                          'gyro_sign': -1.0}]),

        # --- rf2o (TF는 EKF가 담당하므로 publish_tf False) ---
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry', output='screen',
             parameters=[{'laser_scan_topic': '/scan',
                          'odom_topic': '/odom_rf2o',
                          'publish_tf': False,
                          'base_frame_id': 'base_link',
                          'odom_frame_id': 'odom',
                          'init_pose_from_topic': '',
                          'freq': 20.0}]),

        # --- EKF (라이다/브릿지가 뜬 뒤 시작) ---
        TimerAction(period=3.0, actions=[
            Node(package='robot_localization', executable='ekf_node',
                 name='ekf_filter_node', output='screen',
                 parameters=[os.path.join(HOME, 'ros2_ws/config/ekf.yaml')]),
        ]),

        # --- slam_toolbox (EKF가 TF 쏘기 시작한 뒤) ---
        TimerAction(period=6.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch', 'online_async_launch.py')),
                launch_arguments={'slam_params_file': os.path.join(
                    HOME, 'ros2_ws/config/my_mapper.yaml')}.items()),
        ]),
    ])
