from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import sys
import os

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    rosbag_record = sys.argv[-1]
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'turtlebot3_worlds/' + TURTLEBOT3_MODEL + '.model'
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ]
    actions.append(Node(
        package='ros2_turtlebot',
        executable='walker',
    ))
    if rosbag_record=='record:=True':
        actions.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", "ros2_turtlebot_bag", "-a"]
            )
        )
    return LaunchDescription(actions)
