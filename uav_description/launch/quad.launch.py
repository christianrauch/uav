from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("uav_description"), "urdf", "quad.urdf.xacro",]),
    ])

    robot_controllers = PathJoinSubstitution([
        FindPackageShare("uav_description"), "config", "quad_controllers.yaml",
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
        ],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='screen',
        remappings=[
            ('/proxy_imu/imu', '/mujoco_drone_simulator/imu'),
        ],
    )

    imu_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'imu_sensor_broadcaster',
        ],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        imu_sensor_broadcaster_spawner,
    ])
