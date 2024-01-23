import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # cycle_move = LaunchConfiguration('cycle_move')
    cycle_move_launch_arg = DeclareLaunchArgument(
        'cycle_move',
        default_value='false'
    )
    # state_publisher = LaunchConfiguration('state_publisher')
    state_publisher_launch_arg = DeclareLaunchArgument(
        'state_publisher',
        default_value='true'
    )
    # rviz = LaunchConfiguration('rviz')
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true'
    )

    rd_file = DeclareLaunchArgument(
        'rd_file',
        default_value=os.path.join(get_package_share_directory('cartesian_control'), "urdf", "franka_urdf.xml")
    )


    load_gripper_parameter_name = 'load_gripper'
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    load_gripper_arg = DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as end-effector if true. Robot is loaded without '
                        'end-effector otherwise')
    
    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots',
                                     'panda_arm.urdf.xacro')

    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper])
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')

    return launch.LaunchDescription([
        cycle_move_launch_arg,
        state_publisher_launch_arg,
        rviz_arg,
        rd_file,
        load_gripper_arg,
        launch_ros.actions.Node(
            package='cartesian_control',
            executable='marker_control',
            name='marker_control',
            parameters=[{"rd_file": LaunchConfiguration('rd_file')}],
            output='screen'),
        launch_ros.actions.Node(
            package='robot_sim',
            executable='robot_sim_bringup',
            name='cycle_move',
            parameters=[
                {'cycle_move': LaunchConfiguration('cycle_move')},
                {"rd_file": LaunchConfiguration('rd_file')}
            ],
            output='screen'),      
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(LaunchConfiguration('state_publisher'))),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['--display-config', rviz_file]),
  ])