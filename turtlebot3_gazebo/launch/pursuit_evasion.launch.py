import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_p_pose = LaunchConfiguration('x_p_pose', default='-2.0')
    y_p_pose = LaunchConfiguration('y_p_pose', default='-0.5')
    x_e1_pose = LaunchConfiguration('x_e1_pose', default='1.0')
    y_e1_pose = LaunchConfiguration('y_e1_pose', default='0.0')
    x_e2_pose = LaunchConfiguration('x_e2_pose', default='-1.0')
    y_e2_pose = LaunchConfiguration('y_e2_pose', default='0.0')
    pursuer = LaunchConfiguration('pursuer_tb3_name', default='pursuer' )
    evader1 = LaunchConfiguration('evader1_tb3_name', default='evader1' )
    evader2 = LaunchConfiguration('evader2_tb3_name', default='evader2' )

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_pursuer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_pursuer.launch.py')
        ),
        launch_arguments={
            'x_pose': x_p_pose,
            'y_pose': y_p_pose,
            'tb3_name': pursuer
        }.items()
    )

    spawn_evader1_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_evader1.launch.py')
        ),
        launch_arguments={
            'x_pose': x_e1_pose,
            'y_pose': y_e1_pose,
            'tb3_name': evader1
        }.items()
    )

    spawn_evader2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_evader2.launch.py')
        ),
        launch_arguments={
            'x_pose': x_e2_pose,
            'y_pose': y_e2_pose,
            'tb3_name': evader2
        }.items()
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_pursuer_cmd)
    ld.add_action(spawn_evader1_cmd)
    ld.add_action(spawn_evader2_cmd)

    return ld
