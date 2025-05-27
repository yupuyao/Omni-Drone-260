import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    obj_num = LaunchConfiguration('obj_num', default=5)
    drone_id = LaunchConfiguration('drone_id', default=0)
    
    map_size_x = LaunchConfiguration('map_size_x', default = 18.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 16.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 4.0)
    odom_topic = LaunchConfiguration('odom_topic', default = '/vins_estimator/odometry')
    
    obj_num_cmd = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')

    
    # Include advanced parameters
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'obj_num_set': obj_num,
            
            'camera_pose_topic': 'nouse0',
            'depth_topic': '/cam0/depth_image',
            'cloud_topic': 'nouse1',
            
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),
            'max_vel': str(0.5),
            'max_acc': str(6.0),
            'planning_horizon': str(6.0),
            'use_distinctive_trajs': 'False',
            'flight_type': str(1),
        }.items()
    )
    
     # waypoint_manager node

    pkg_dir = get_package_share_directory('waypoint_manager')
    waypoints_file = os.path.join(pkg_dir, 'config', 'waypoints.yaml')

    waypoints_file_arg = DeclareLaunchArgument(
    'waypoints_file',
    default_value=waypoints_file,
    description='Path to the waypoints configuration file'
    )

    waypoint_manager_node = Node(
        package='waypoint_manager',
        executable='waypoint_manager_node',
        name='waypoint_manager',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('waypoints_file')]
    )
    
    # Trajectory server node
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],
        parameters=[
            {'traj_server/time_forward': 1.0}
        ]
    )
    
    
    ld = LaunchDescription()
        
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(obj_num_cmd)
    ld.add_action(drone_id_cmd)

    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(waypoints_file_arg)
    ld.add_action(waypoint_manager_node)

    return ld
