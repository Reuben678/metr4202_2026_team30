from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock.",
    )

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
        period=3.0,
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
        period=6.0,
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
    # Launch Project Nodes
    # ---------------------------------------------------------

    return LaunchDescription([
        declare_use_sim_time,
        util_weights,
        frontier_search,
        recovery_helper,
        exploration_manager,
        target_localizer,
    ])
