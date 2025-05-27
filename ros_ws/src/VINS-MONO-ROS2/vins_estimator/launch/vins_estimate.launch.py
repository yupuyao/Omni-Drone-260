from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution 
from launch_ros.actions import Node

def generate_launch_description():

    config_pkg_path = get_package_share_directory('config_pkg')

    config_path = PathJoinSubstitution([
        config_pkg_path,
        'config/config.yaml'
    ])

    vins_path = PathJoinSubstitution([
        config_pkg_path,
        'config/../'
    ])

    support_path = PathJoinSubstitution([
        config_pkg_path,
        'support_files'
    ])
    
    # Declare launch arguments
    imu_frame_id_arg = DeclareLaunchArgument(
        'imu_frame_id',
        default_value=TextSubstitution(text='imu_link'),
        description='Frame ID for the IMU')
    
    base_frame_id_arg = DeclareLaunchArgument(
        'base_frame_id',
        default_value=TextSubstitution(text='base_link'),
        description='Frame ID for the base link')
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value=TextSubstitution(text='true'),
        description='Whether to publish TF transform for the IMU')
    
    # Configure IMU Transform Node
    imu_transform_node = Node(
        package='imu_transform',
        executable='imu_transform_node',
        name='imu_transform_node',
        output='screen',
        parameters=[{
            'imu_frame_id': LaunchConfiguration('imu_frame_id'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'publish_tf': LaunchConfiguration('publish_tf')
        }]
    )
    
    # Define the feature_tracker node
    feature_tracker_node = Node(
        package='feature_tracker',
        executable='feature_tracker',
        name='feature_tracker',
        namespace='feature_tracker',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'vins_folder': vins_path
        }]
    )
    
    # Define the vins_estimator node
    vins_estimator_node = Node(
        package='vins_estimator',
        executable='vins_estimator',
        name='vins_estimator',
        namespace='vins_estimator',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'vins_folder': vins_path
        }]
    )

    # Define the pose_graph node
    pose_graph_node = Node(
        package='pose_graph',
        executable='pose_graph',
        name='pose_graph',
        namespace='pose_graph',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'support_file': support_path,
            'visualization_shift_x': 0,
            'visualization_shift_y': 0,
            'skip_cnt': 0,
            'skip_dis': 0.0
        }]
    )

    return LaunchDescription([
        LogInfo(msg=['[vins estimator launch] config path: ', config_path]),
        LogInfo(msg=['[feature tracker launch] config path: ', config_path]),
        LogInfo(msg=['[vins estimator launch] vins path: ', vins_path]),
        LogInfo(msg=['[vins estimator launch] support path: ', support_path]),
        
        imu_frame_id_arg,
        base_frame_id_arg,
        publish_tf_arg,


        imu_transform_node,
        feature_tracker_node,
        vins_estimator_node,
        pose_graph_node
    ])