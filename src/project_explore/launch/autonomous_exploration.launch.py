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

    gazebo_ros_launch_directory = os.path.join(
    get_package_share_directory("gazebo_ros"), "launch")

    world = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")

    default_world = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "worlds",
        "turtlebot3_world.world",
    )

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Full path to the Gazebo world file.",
    )
    declare_x_pose = DeclareLaunchArgument("x_pose", default_value="-2.0")
    declare_y_pose = DeclareLaunchArgument("y_pose", default_value="-0.5")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_launch_directory, "gzserver.launch.py")),
        launch_arguments={"world": world}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_launch_directory, "gzclient.launch.py")),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_directory, "robot_state_publisher.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_directory, "spawn_turtlebot3.launch.py")),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
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
            declare_world,
            declare_x_pose,
            declare_y_pose,
            set_turtlebot3_model,
            gzserver,
            gzclient,
            robot_state_publisher,
            spawn_turtlebot,
            navigation_and_slam,
            rviz,
            util_weights,
            frontier_search,
            exploration_manager,
        ]
    )