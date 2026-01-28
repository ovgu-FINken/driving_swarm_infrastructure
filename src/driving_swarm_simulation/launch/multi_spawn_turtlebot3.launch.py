import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the urdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    theta = LaunchConfiguration('theta', default='0.0')
    robot_name = LaunchConfiguration('robot_name', default=TURTLEBOT3_MODEL)

    bridge_params = LaunchConfiguration('bridge_params', default=os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'params',
        model_folder+'_bridge.yaml'
    ))
    urdf_path = LaunchConfiguration('urdf_path', default=os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    ))
    
    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_theta_cmd = DeclareLaunchArgument(
        'theta', default_value='0.0',
        description='Specify yaw angle of the robot')
    
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-Y', theta
        ],
        output='screen',
    )

    
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            ['config_file:=', bridge_params], 
            
        ],
        remappings=[
            ('tf', '/tf'),             # Force TF to global
            ('tf_static', '/tf_static') # Force Static TF to global
        ],
        output='screen',
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[['/', robot_name, '/camera/image_raw']], 
        remappings=[
            (['/', robot_name, '/camera/image_raw'], 'camera/image_raw')
        ],
        output='screen',
    )
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_theta_cmd)
    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_image_bridge_cmd) if TURTLEBOT3_MODEL != 'burger' else None

    return ld
