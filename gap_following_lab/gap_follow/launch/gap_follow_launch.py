#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    wall_following_node = Node(
        package="gap_follow",
        executable="reactive_node",
        parameters=[
            {'avg_window_size': 1},
            {'high_dist_cutoff': 3.0},
            {'inflation_radius': 0.21},
            {'high_speed': 1.5},
            {'mid_speed': 1.0},
            {'low_speed': 1.0},
            {'disparity_thresh': 0.3}
        ]
    )

    ld.add_action(wall_following_node)

    return ld