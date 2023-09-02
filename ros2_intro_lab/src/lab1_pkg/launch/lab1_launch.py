#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    talker_node = Node(
        package="lab1_pkg",
        executable="talker",
        parameters=[
            {'v': 1.0},
            {'d': 3.0}
        ]
    )

    relay_node = Node(
        package="lab1_pkg",
        executable="relay"
    )

    ld.add_action(talker_node)
    ld.add_action(relay_node)

    return ld