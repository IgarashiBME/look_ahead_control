"""Launch file for look_ahead_following node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Topic remapping arguments
    gnss_odom_topic = LaunchConfiguration('gnss_odom_topic')
    gnss_topic = LaunchConfiguration('gnss_topic')
    mission_topic = LaunchConfiguration('mission_topic')
    modes_topic = LaunchConfiguration('modes_topic')
    rc_pwm_topic = LaunchConfiguration('rc_pwm_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    auto_log_topic = LaunchConfiguration('auto_log_topic')

    return LaunchDescription([
        # Topic arguments
        DeclareLaunchArgument(
            'gnss_odom_topic', default_value='/gnss_odom',
            description='Odometry topic (used when odom_source=odom)'),
        DeclareLaunchArgument(
            'gnss_topic', default_value='/gnss/solution',
            description='GNSS topic (used when odom_source=gnss)'),
        DeclareLaunchArgument(
            'mission_topic', default_value='/mav/mission',
            description='MAVLink mission waypoint topic'),
        DeclareLaunchArgument(
            'modes_topic', default_value='/mav/modes',
            description='MAVLink modes topic'),
        DeclareLaunchArgument(
            'rc_pwm_topic', default_value='/rc_pwm',
            description='RC PWM output topic'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel',
            description='Velocity command output topic'),
        DeclareLaunchArgument(
            'auto_log_topic', default_value='/auto_log',
            description='Telemetry log output topic'),

        # Control parameters
        DeclareLaunchArgument('Kp', default_value='0.0'),
        DeclareLaunchArgument('Kcte', default_value='0.0'),
        DeclareLaunchArgument('look_ahead', default_value='0.0'),
        DeclareLaunchArgument('pivot_threshold', default_value='40.0'),
        DeclareLaunchArgument('cte_threshold', default_value='0.1'),
        DeclareLaunchArgument('wp_arrival_dist', default_value='0.1'),
        DeclareLaunchArgument('wp_skip_dist', default_value='0.8'),

        # Output parameters
        DeclareLaunchArgument('throttle_scale', default_value='0.5'),
        DeclareLaunchArgument('pivot_scale', default_value='0.5'),
        DeclareLaunchArgument('driver_mix', default_value='0.0'),
        DeclareLaunchArgument('pwm_center', default_value='1500.0'),
        DeclareLaunchArgument('pwm_range', default_value='500.0'),
        DeclareLaunchArgument('pwm_min', default_value='1000.0'),
        DeclareLaunchArgument('pwm_max', default_value='2000.0'),

        # Other parameters
        DeclareLaunchArgument('odom_source', default_value='odom'),

        Node(
            package='look_ahead_control',
            executable='look_ahead_following',
            name='look_ahead_following',
            remappings=[
                ('/gnss_odom', gnss_odom_topic),
                ('/gnss/solution', gnss_topic),
                ('/mav/mission', mission_topic),
                ('/mav/modes', modes_topic),
                ('/rc_pwm', rc_pwm_topic),
                ('/cmd_vel', cmd_vel_topic),
                ('/auto_log', auto_log_topic),
            ],
            parameters=[{
                'Kp': LaunchConfiguration('Kp'),
                'Kcte': LaunchConfiguration('Kcte'),
                'look_ahead': LaunchConfiguration('look_ahead'),
                'pivot_threshold': LaunchConfiguration('pivot_threshold'),
                'cte_threshold': LaunchConfiguration('cte_threshold'),
                'wp_arrival_dist': LaunchConfiguration('wp_arrival_dist'),
                'wp_skip_dist': LaunchConfiguration('wp_skip_dist'),
                'throttle_scale': LaunchConfiguration('throttle_scale'),
                'pivot_scale': LaunchConfiguration('pivot_scale'),
                'driver_mix': LaunchConfiguration('driver_mix'),
                'pwm_center': LaunchConfiguration('pwm_center'),
                'pwm_range': LaunchConfiguration('pwm_range'),
                'pwm_min': LaunchConfiguration('pwm_min'),
                'pwm_max': LaunchConfiguration('pwm_max'),
                'odom_source': LaunchConfiguration('odom_source'),
            }],
            output='screen',
        ),
    ])
