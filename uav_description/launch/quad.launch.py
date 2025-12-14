from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('uav_description'), 'urdf', 'quad.urdf.xacro',]),
    ])

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('uav_description'), 'config', 'quad_controllers.yaml',
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
        arguments=[
            '--ros-args',
            # '--log-level', 'control_allocation:=debug',
            # '--log-level', 'angular_velocity:=debug',
            # '--log-level', 'angular_position:=debug',
        ],
        remappings=[
            ('/proxy_imu/imu', '/mujoco_drone_simulator/imu'),
            ('/proxy_actuator/actuators', '/mujoco_drone_simulator/actuators'),
        ],
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'imu_sensor_broadcaster',
            'control_allocation',
            'angular_velocity',
            'angular_position',
        ],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        controller_spawner,
    ])
