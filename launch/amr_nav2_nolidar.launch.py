# AMR 자율주행 런치 — 센서/오도메트리 + Nav2 스택
# 매핑 런치와의 차이: slam_toolbox 자리에 map_server + AMCL + Nav2
# 사용: ros2 launch /home/daehan/ros2_ws/launch/amr_nav2.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

HOME = os.path.expanduser('~')
PARAMS = os.path.join(HOME, 'ros2_ws/config/nav2_params.yaml')
MAP = os.path.join(HOME, 'ros2_ws/maps/my_real_map_v5.yaml')

def generate_launch_description():
    turn_gain = LaunchConfiguration('turn_gain')

    lifecycle_nodes = ['map_server', 'amcl',
                       'controller_server', 'planner_server',
                       'behavior_server', 'bt_navigator',
                       'waypoint_follower', 'velocity_smoother']

    return LaunchDescription([
        # ★Nav2는 컨트롤러가 각속도를 직접 계산 → 증폭 금지(1.0)
        DeclareLaunchArgument('turn_gain', default_value='1.0'),

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

        # --- 브릿지 (gyro_scale 65.5 = ±500dps) ---
        Node(package='amr_bridge', executable='bridge_node', name='amr_bridge',
             output='screen',
             parameters=[{'turn_gain': turn_gain,
                          'cmd_timeout': 0.8,
                          'gyro_scale': 65.5,
                          'turn_then_drive': True,
                          'ttd_ang_thresh': 0.5,
                          'stop_epsilon': 0.001,
                          'gyro_sign': -1.0,
                          'onset_int8': 127,
                          'pulse_on_ticks': 3,
                          'pulse_off_ticks': 12}]),

        # --- rf2o (TF는 EKF 담당) ---
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry', output='screen',
             parameters=[{'laser_scan_topic': '/scan',
                          'odom_topic': '/odom_rf2o',
                          'publish_tf': False,
                          'base_frame_id': 'base_link',
                          'odom_frame_id': 'odom',
                          'init_pose_from_topic': '',
                          'freq': 20.0}]),

        # --- EKF (odom→base_link) ---
        TimerAction(period=3.0, actions=[
            Node(package='robot_localization', executable='ekf_node',
                 name='ekf_filter_node', output='screen',
                 parameters=[os.path.join(HOME, 'ros2_ws/config/ekf.yaml')]),
        ]),

        # --- Nav2 스택 (EKF가 TF 쏘기 시작한 뒤) ---
        TimerAction(period=12.0, actions=[
            Node(package='nav2_map_server', executable='map_server',
                 name='map_server', output='screen',
                 parameters=[PARAMS, {'yaml_filename': MAP}]),
            Node(package='nav2_amcl', executable='amcl',
                 name='amcl', output='screen', parameters=[PARAMS]),
            Node(package='nav2_controller', executable='controller_server',
                 name='controller_server', output='screen', parameters=[PARAMS]),
            Node(package='nav2_planner', executable='planner_server',
                 name='planner_server', output='screen', parameters=[PARAMS]),
            Node(package='nav2_behaviors', executable='behavior_server',
                 name='behavior_server', output='screen', parameters=[PARAMS]),
            Node(package='nav2_bt_navigator', executable='bt_navigator',
                 name='bt_navigator', output='screen', parameters=[PARAMS]),
            Node(package='nav2_waypoint_follower', executable='waypoint_follower',
                 name='waypoint_follower', output='screen', parameters=[PARAMS]),
            Node(package='nav2_velocity_smoother', executable='velocity_smoother',
                 name='velocity_smoother', output='screen', parameters=[PARAMS]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=[{'use_sim_time': False,
                              'autostart': True,
                              'bond_timeout': 20.0,
                              'attempt_respawn_reconnection': True,
                              'node_names': lifecycle_nodes}]),
        ]),
    ])
