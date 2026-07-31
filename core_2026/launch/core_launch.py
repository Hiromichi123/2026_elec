import os
import launch
from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    start_mavros = LaunchConfiguration('start_mavros')
    real_robot_odom_topic = LaunchConfiguration('real_robot_odom_topic')
    platform_mode = LaunchConfiguration('platform_mode')
    carrier_pose_topic = LaunchConfiguration('carrier_pose_topic')
    field_frame_enabled = LaunchConfiguration('field_frame_enabled')
    field_origin_x = LaunchConfiguration('field_origin_x')
    field_origin_y = LaunchConfiguration('field_origin_y')
    field_origin_z = LaunchConfiguration('field_origin_z')
    field_origin_yaw = LaunchConfiguration('field_origin_yaw')

    mavros = Node(
        package='mavros',
        executable='mavros_node',
        condition=IfCondition(start_mavros),
        parameters=[{
            'fcu_url': fcu_url,
            'tgt_system': 1,
            'tgt_component': 1,
            'fcu_protocol': 'v2.0'
        }]
    )

    isolated_lidar_remaps = [
        SetRemap(src="/livox/lidar", dst="/drone/livox/lidar"),
        SetRemap(src="/livox/imu", dst="/drone/livox/imu"),
        SetRemap(src="/aft_mapped_to_init", dst="/drone/aft_mapped_to_init"),
        SetRemap(src="/path", dst="/drone/path"),
        SetRemap(src="/Laser_map", dst="/drone/Laser_map"),
        SetRemap(src="/cloud_registered", dst="/drone/cloud_registered"),
        SetRemap(src="/cloud_registered_body", dst="/drone/cloud_registered_body"),
        SetRemap(src="/cloud_effected", dst="/drone/cloud_effected"),
    ]

    livox_ros_driver = GroupAction([
        PushRosNamespace("drone"),
        *isolated_lidar_remaps,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py')
            ])
        ),
    ])

    tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="drone_base_to_livox_tf",
        arguments=["0", "0", "0", "0", "0", "0", "1", "drone/base_link", "drone/livox_frame"]
    )

    slam = TimerAction(
        period=5.0,  # 延迟 10s 启动 PointLIO
        actions=[
            GroupAction([
                *isolated_lidar_remaps,
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare("point_lio"),
                            "launch",
                            "point_lio.launch.py"
                        ])
                    ]),
                    launch_arguments={
                        "namespace": "drone",
                        "rviz": "False",
                    }.items(),
                ),
            ])
        ]
    )
    
    ros2_tools_nodes = [
        Node(package='ros2_tools', 
             executable='lidar_data_node',
             name='drone_lidar_data_node',
             remappings=[
                ('/aft_mapped_to_init', '/drone/aft_mapped_to_init'),
                ('lidar_data', '/drone/lidar_data'),
             ],
             parameters=[{
                'use_simulation': False, # 仿真开关
                'simulation_odom_topic': '/absolute_pose', # gazebo的里程计话题
                'real_robot_odom_topic': real_robot_odom_topic, # PointLIO/FastLIO的里程计话题
                'reset_origin_on_start': False,
                'field_origin_x': ParameterValue(field_origin_x, value_type=float),
                'field_origin_y': ParameterValue(field_origin_y, value_type=float),
                'field_origin_z': ParameterValue(field_origin_z, value_type=float),
                'field_origin_yaw': ParameterValue(field_origin_yaw, value_type=float),
             }]),
        Node(
            package='ros2_tools',
            executable='lidar_to_px4_bridge',
            name='drone_lidar_to_px4_bridge',
            remappings=[
                ('/aft_mapped_to_init', '/drone/aft_mapped_to_init'),
            ],
            parameters=[{
                'real_robot_odom_topic': real_robot_odom_topic,
                'vision_pose_topic': '/mavros/vision_pose/pose',
                'reset_origin_on_start': False,
            }]
        )
    ]

    core = Node(
        package='core_2026',
        executable='quad_node',
        parameters=[{
            'platform_mode': platform_mode,
            'platform_target_topic': '/platform/target',
            'position_setpoint_topic': '/mavros/setpoint_position/local',
            'velocity_setpoint_topic': '/mavros/setpoint_velocity/cmd_vel',
            'lidar_pose_topic': '/drone/lidar_data',
            'carrier_pose_topic': carrier_pose_topic,
            'car_status_topic': '/car/status',
            'mission_command_topic': '/mission/command',
            'drone_status_topic': '/drone/status',
            'airdrop_cmd_topic': '/drone/airdrop_cmd',
            'field_frame_enabled': ParameterValue(field_frame_enabled, value_type=bool),
            'field_origin_x': ParameterValue(field_origin_x, value_type=float),
            'field_origin_y': ParameterValue(field_origin_y, value_type=float),
            'field_origin_z': ParameterValue(field_origin_z, value_type=float),
            'field_origin_yaw': ParameterValue(field_origin_yaw, value_type=float),
        }],
    )

    airdrop_pwm = Node(
        package='ros2_tools',
        executable='pwm_node',
        name='airdrop_pwm_node',
        output='screen',
        parameters=[{
            'service': '/drone/servo_control',
            'command_topic': '/drone/airdrop_cmd',
            'pwm_pin': 13,
            'frequency': 50.0,
            'release_command': 1,
            'neutral_command': 0,
            'pulse_hold_sec': 1.0,
            'auto_return_neutral': False,
        }],
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'real_robot_odom_topic',
            default_value='/drone/aft_mapped_to_init',
            description='Odometry topic from PointLIO/FastLIO, for lidar_data_node and lidar_to_px4_bridge.'
        ),
        DeclareLaunchArgument(
            'field_frame_enabled',
            default_value='true',
            description='Convert mission field-frame commands back to PX4 local frame.'
        ),
        DeclareLaunchArgument(
            'field_origin_x',
            default_value='1.125',
            description='Field X coordinate of the drone PointLIO origin at startup.'
        ),
        DeclareLaunchArgument(
            'field_origin_y',
            default_value='1.125',
            description='Field Y coordinate of the drone PointLIO origin at startup.'
        ),
        DeclareLaunchArgument(
            'field_origin_z',
            default_value='0.0',
            description='Field Z coordinate of the drone PointLIO origin at startup.'
        ),
        DeclareLaunchArgument(
            'field_origin_yaw',
            default_value='1.57079632679',
            description='Field yaw offset of the drone PointLIO local frame at startup.'
        ),
        DeclareLaunchArgument(
            'platform_mode',
            default_value='px4_drone',
            description='Control backend: px4_drone, px4_diff_car, or custom_ackermann.'
        ),
        DeclareLaunchArgument(
            'carrier_pose_topic',
            default_value='/carrier/lidar_pose',
            description='Ground carrier pose topic. Type: ros2_tools/msg/LidarPose.'
        ),
        DeclareLaunchArgument(
            'fcu_url',
            default_value='serial:///dev/ttyACM0:57600',
            description='MAVROS FCU URL used when start_mavros is true.'
        ),
        DeclareLaunchArgument(
            'start_mavros',
            default_value='true',
            description='Start MAVROS from this launch file.'
        ),
        mavros,             # ros2 run mavros mavros_node --ros-args -p fcu_url:=serial:///dev/ttyACM0:57600 -p tgt_system:=1 -p tgt_component:=1 -p fcu_protocol:=v2.0
        livox_ros_driver,   # ros2 launch livox_ros_driver2 msg_MID360_launch.py
        tf,                 # ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link livox_frame
        slam,               # ros2 launch point_lio point_lio.launch.py rviz:=False
        *ros2_tools_nodes,  # ros2 run ros2_tools lidar_data_node --ros-args -p use_simulation:=False -p simulation_odom_topic:=/absolute_pose -p real_robot_odom_topic:=/aft_mapped_to_init
                            # ros2 run ros2_tools lidar_to_px4_bridge
        core,               # ros2 run core_2026 quad_node
        airdrop_pwm         # ros2 run ros2_tools pwm_node
    ])
