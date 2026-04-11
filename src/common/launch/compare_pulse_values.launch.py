import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic = LaunchConfiguration('topic')
    topic_to_compare = LaunchConfiguration('topic_to_compare')
    video_file = LaunchConfiguration('video_file')
    bdf_file = LaunchConfiguration('bdf_file')

    common_share = get_package_share_directory('common')
    phm_share = get_package_share_directory('pulse_head_movement')
    emm_share = get_package_share_directory('eulerian_motion_magnification')
    legacy_share = get_package_share_directory('legacy_measurement')

    compare_node = Node(
        package='common',
        executable='compare_pulse_values.py',
        name='compare',
        output='screen',
        parameters=[{
            'topic': topic,
            'topic_to_compare': topic_to_compare,
        }],
    )

    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='pulse_plot',
        arguments=['--layout', os.path.join(common_share, 'config', 'compare_config.xml')],
    )

    phm_chest_group = GroupAction(
        condition=LaunchConfigurationEquals('topic', '/pulse_chest_strap'),
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(phm_share, 'launch', 'industry_camera.launch')
                )
            ),
            Node(
                package='pulse_chest_strap',
                executable='pulse_chest_strap.py',
                name='pulse_chest_strap',
                arguments=['-m', '00:22:D0:84:1E:64'],
                output='screen',
            ),
        ],
    )

    phm_ecg_group = GroupAction(
        condition=LaunchConfigurationEquals('topic', '/ecg'),
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(phm_share, 'launch', 'pulse_head_movement.launch')
                ),
                launch_arguments={
                    'video_file': video_file,
                    'bdf_file': bdf_file,
                }.items(),
            ),
        ],
    )

    emm_chest_group = GroupAction(
        condition=LaunchConfigurationEquals('topic', '/pulse_chest_strap'),
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(emm_share, 'launch', 'industry_camera.launch')
                )
            ),
            Node(
                package='pulse_chest_strap',
                executable='pulse_chest_strap.py',
                name='pulse_chest_strap',
                arguments=['-m', '00:22:D0:84:1E:64'],
                output='screen',
            ),
        ],
    )

    emm_ecg_group = GroupAction(
        condition=LaunchConfigurationEquals('topic', '/ecg'),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(emm_share, 'launch', 'eulerian_motion_magnification.launch.py')
                ),
                launch_arguments={
                    'video_file': video_file,
                    'bdf_file': bdf_file,
                }.items(),
            ),
        ],
    )

    legacy_chest_group = GroupAction(
        condition=LaunchConfigurationEquals('topic', '/pulse_chest_strap'),
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(legacy_share, 'launch', 'webcam.launch')
                )
            ),
            Node(
                package='pulse_chest_strap',
                executable='pulse_chest_strap.py',
                name='pulse_chest_strap',
                arguments=['-m', '00:22:D0:84:1E:64'],
                output='screen',
            ),
        ],
    )

    legacy_ecg_group = GroupAction(
        condition=LaunchConfigurationEquals('topic', '/ecg'),
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(legacy_share, 'launch', 'legacy_measurement.launch')
                ),
                launch_arguments={
                    'video_file': video_file,
                    'bdf_file': bdf_file,
                }.items(),
            ),
        ],
    )

    phm_group = GroupAction(
        condition=LaunchConfigurationEquals('topic_to_compare', '/pulse_head_movement'),
        actions=[phm_chest_group, phm_ecg_group],
    )

    emm_group = GroupAction(
        condition=LaunchConfigurationEquals('topic_to_compare', '/eulerian_motion_magnification'),
        actions=[emm_chest_group, emm_ecg_group],
    )

    legacy_group = GroupAction(
        condition=LaunchConfigurationEquals('topic_to_compare', '/legacy_measurement'),
        actions=[legacy_chest_group, legacy_ecg_group],
    )

    return LaunchDescription([
        DeclareLaunchArgument('topic', default_value='/pulse_chest_strap'),
        DeclareLaunchArgument('topic_to_compare', default_value='/pulse_head_movement'),
        DeclareLaunchArgument('video_file', default_value=''),
        DeclareLaunchArgument('bdf_file', default_value=''),
        compare_node,
        plotjuggler_node,
        phm_group,
        emm_group,
        legacy_group,
    ])