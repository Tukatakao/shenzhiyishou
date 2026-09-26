from launch import LaunchDescription
from launch_ros.actions import Node
# 注意！Humble 下 FindPackageShare 在 launch_ros.substitutions
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = FindPackageShare(package="shoubi_description").find("shoubi_description")
    xacro_file = pkg_share + "/src/shenzhiyishou.xacro"

    robot_description = Command(['xacro ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])
