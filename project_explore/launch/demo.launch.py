import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use robot clock.",
    )

    # ---------------------------------------------------------
    # Nav2 + SLAM Toolbox (TurtleBot3)
    # Equivalent to:
    #   ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    #       use_sim_time:=false slam:=True
    # ---------------------------------------------------------

    tb3_nav2_launch = os.path.join(
        get_package_share_directory("turtlebot3_navigation2"),
        "launch",
        "navigation2.launch.py",
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_nav2_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": "True",
        }.items(),
    )

    # Delays (s) — on hardware Nav2/SLAM take longer to come up than in sim
    NAV2_STARTUP_DELAY = 10.0

    # ---------------------------------------------------------
    # Utility Weights Node
    # ---------------------------------------------------------

    util_weights = Node(
        package="project_search",
        executable="util_weights",
        name="util_weights",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
    )

    # ---------------------------------------------------------
    # Frontier Search Node
    # ---------------------------------------------------------

    frontier_search = TimerAction(
        period=NAV2_STARTUP_DELAY,
        actions=[
            Node(
                package="project_search",
                executable="frontier_search",
                name="frontier_search",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                }],
            )
        ],
    )

    # ---------------------------------------------------------
    # Recovery Helper Node
    # ---------------------------------------------------------

    recovery_helper = Node(
        package="project_explore",
        executable="recovery_helper",
        name="recovery_helper",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
    )

    # ---------------------------------------------------------
    # Exploration Manager Node
    # ---------------------------------------------------------

    exploration_manager = TimerAction(
        period=NAV2_STARTUP_DELAY + 3.0,
        actions=[
            Node(
                package="project_explore",
                executable="exploration_manager",
                name="exploration_manager",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                }],
            )
        ],
    )

    # ---------------------------------------------------------
    # ArUco Target Localisation
    # ---------------------------------------------------------

    target_localizer = Node(
        package="project_localisation",
        executable="localizer",
        name="target_localizer",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "image_topic": "/camera/image_raw",
            "info_topic": "/camera/camera_info",
            "marker_size": 0.10,
        }],
    )

    # ---------------------------------------------------------
    # Launch
    # ---------------------------------------------------------

    return LaunchDescription([
        declare_use_sim_time,
        navigation,
        util_weights,
        frontier_search,
        recovery_helper,
        exploration_manager,
        target_localizer,
    ])