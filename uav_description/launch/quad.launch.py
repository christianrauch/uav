from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    model_name_arg = DeclareLaunchArgument(
        'model',
        default_value='quad',
    )

    model_name = LaunchConfiguration('model')

    xacro_filename = PythonExpression([
        "'", model_name, "' + '.urdf.xacro'"
    ])

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('uav_description'), 'urdf', xacro_filename,]),
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

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='screen',
        additional_env={'RCUTILS_COLORIZED_OUTPUT': '1'},
        arguments=[
            '--ros-args',
            # '--log-level', 'control_allocation:=debug',
            # '--log-level', 'angular_velocity:=debug',
            # '--log-level', 'angular_position:=debug',
            # '--log-level', 'attitude:=debug',
        ],
        remappings=[
            ('/proxy_imu/imu', '/mujoco_drone_simulator/imu'),
            ('/proxy_actuator/actuators', '/mujoco_drone_simulator/actuators'),
        ],
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        additional_env={'RCUTILS_COLORIZED_OUTPUT': '1'},
        arguments=[
            'imu_sensor_broadcaster',
            'magnetometer_broadcaster',
            'fusion',
            'fusion_sensor_broadcaster',
            'control_allocation',
            'angular_velocity',
            # 'angular_position',
            'attitude',
            'attitude_mode',
        ],
    )

    return LaunchDescription([
        model_name_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        ros2_control_node,
        controller_spawner,
    ])
