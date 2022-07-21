from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    frame = 'enu'
    clock = 'steady-clock'
    msg_type = 'pose'
    # uav_prefix = 'fmu'
    uav_prefix = 'fmu_test'

    visual_node = Node(
        package='px4_visual',
        executable='px4_visual_node',
        name='px4_visual_node',
        output='screen',
        parameters=[
            {'frame': frame, 'clock': clock, 'msg_type': msg_type, 'uav_prefix': uav_prefix}
        ],
        remappings=[
            ('/visual/pose', '/vicon/tester/tester'),
        ]
    )

    return LaunchDescription([visual_node])