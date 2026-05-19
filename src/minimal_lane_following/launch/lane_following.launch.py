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
                'hough_threshold': 24,
                'hough_min_line_length_px': 24.0,
                'hough_max_line_gap_px': 35.0,
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
                'control_rate_hz': 15.0,
                'linear_speed': 0.08,
                'kp_lateral': 1.0,
                'kp_heading': 0.8,
                'max_angular_speed': 0.5,
            }],
        ),
    ])
