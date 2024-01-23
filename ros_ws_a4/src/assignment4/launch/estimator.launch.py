import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription(
        [
        launch_ros.actions.Node(
            package='assignment4',
            executable='gui',
            name='autograde_gui',
            output='screen'),
        launch_ros.actions.Node(
            package='assignment4',
            executable='estimator',
            name='estimate',
            output='screen'),
        launch_ros.actions.Node(
            package='assignment4',
            executable='est_gt',
            name='estimate_ground_truth',
            output='screen'),
        launch_ros.actions.Node(
            package='assignment4',
            executable='robot',
            name='robot',
            output='screen'),
        launch_ros.actions.Node(
            package='assignment4',
            executable='autograde',
            name='autograde',
            output='screen'),    ]    
    )
