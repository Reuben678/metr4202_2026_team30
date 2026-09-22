import os

from ament_index_python.packages import (get_package_share_directory)

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction)

from launch.launch_description_sources import (PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    turtlebot3_model = LaunchConfiguration("turtlebot3_model")

    gazebo_launch_directory = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "launch",)

    nav2_launch_directory = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch")

    navigation_params_file = os.path.join(
        get_package_share_directory(
            "turtlebot3_navigation2"
        ),
        "param",
        "humble",
        "waffle_pi.yaml",
    )

    navigation_map_file = os.path.join(
        get_package_share_directory(
            "turtlebot3_navigation2"
        ),
        "map",
        "map.yaml",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use the Gazebo simulation clock.",
    )

    declare_turtlebot3_model = DeclareLaunchArgument(
        "turtlebot3_model",
        default_value="waffle_pi",
        description="TurtleBot3 model used in Gazebo.",
    )

    set_turtlebot3_model = SetEnvironmentVariable(
        name="TURTLEBOT3_MODEL",
        value=turtlebot3_model,
    )

    nav2_bringup_share = get_package_share_directory(
    "nav2_bringup"
    )

    rviz_config = os.path.join(
        nav2_bringup_share,
        "rviz",
        "nav2_default_view.rviz",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_launch_directory,
                "turtlebot3_world.launch.py",
            )
        )
    )


    rviz = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[
                    {"use_sim_time": use_sim_time}
                ],
                output="screen",
            )
        ],
    )
    
    # This launches both Nav2 and SLAM Toolbox because
    # the slam argument is set to true.
    navigation_and_slam = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        nav2_launch_directory,
                        "bringup_launch.py",
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map": navigation_map_file,
                    "slam": "True",
                    "autostart": "true",
                    "use_composition": "False",
                    "params_file": navigation_params_file,
                }.items(),
            )
        ],
    )

    util_weights = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="project_search",
                executable="util_weights",
                name="util_weights",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
            )
        ],
    )

    frontier_search = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="project_search",
                executable="frontier_search",
                name="frontier_search",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
            )
        ],
    )

    exploration_manager = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="project_explore",
                executable="exploration_manager",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_turtlebot3_model,
            set_turtlebot3_model,
            gazebo,
            navigation_and_slam,
            rviz,
            util_weights,
            frontier_search,
            exploration_manager,
        ]
    )