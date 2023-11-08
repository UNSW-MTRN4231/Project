from launch import LaunchDescription
##from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
##from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ##position_goals = PathJoinSubstitution(
      ##  [FindPackageShare("bottle_flip"), "config", "goal_publishers_config.yaml"]
    ##)
    config = os.path.join(
        get_package_share_directory('bottle_flip'),
        'config',
        'goal_publishers_config.yaml'
        )

    return LaunchDescription(
        [
            Node(
                package="bottle_flip",
                executable="bottle_flip",
                name="publisher_joint_trajectory_controller",
                parameters=[config],
                output="screen",
            )
        ]
    )
