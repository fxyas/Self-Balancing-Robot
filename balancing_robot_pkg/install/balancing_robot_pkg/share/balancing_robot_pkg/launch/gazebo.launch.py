from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_name = 'balancing_robot_pkg'  
    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'robot.urdf'
    )

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'))) 
    Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
    parameters=[{'use_sim_time': True}],)

    # PID Controller Node
    pid_python_node = Node(
    package='balancing_robot_pkg',
    executable='pid_sim',
    name='pid_sim',
    output='screen')

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path).read()},{'use_sim_time': True}],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'amr',
            '-file', urdf_path,
            '-x', '0', '-y', '0', '-z', '0.2'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )


    return LaunchDescription([
        gazebo,
        rsp_node,
        spawn_entity,
        pid_python_node
    ])