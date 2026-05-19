from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='minimal_lane_following',
            executable='lane_perception_node',
            name='lane_perception_node',
            output='screen',
            parameters=[{
                'image_topic': '/rm0/camera/image_color',
                'geometry_topic': '/lane_geometry',
                'debug_image_topic': '/lane_debug/image',
                'max_processing_fps': 10.0,
                'draw_debug_guides': True,
            }],
        ),
        Node(
            package='minimal_lane_following',
            executable='lane_controller_node',
            name='lane_controller_node',
            output='screen',
            parameters=[{
                'geometry_topic': '/lane_geometry',
                'cmd_vel_topic': '/rm0/cmd_vel',
                'control_rate_hz': 30.0,
                'linear_speed': 0.12,
                'kp_lateral': 1.3,
                'kp_heading': 1.0,
                'max_angular_speed': 0.8,
            }],
        ),
    ])
