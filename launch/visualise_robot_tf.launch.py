import os, sys
import yaml
import xacro
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode, ParameterValue
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource

def load_yaml_file_absolute_path(yaml_file):
    """Load a YAML file and return the parameters dictionary."""
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    home_dir = os.path.expanduser("~")
    sup_tms_params = os.path.join(home_dir, "sup_tms/tmr_ws/config/sup_tms_params.yaml")
    
    pose_f_tcp_params = load_yaml_file_absolute_path(sup_tms_params).get("pose_f_tcp_params", {})
    
    controllers_configs = PathJoinSubstitution(
        [FindPackageShare("hm_tm5-700_moveit_config"), "config", "ros2_controllers.yaml"]
    )

    # pkg_name = 'urdf_example'
    # file_subpath = 'description/example_robot.urdf.xacro'
    # xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    # robot_description = xacro.process_file(xacro_file).toxml()
    
    
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [FindPackageShare("hm_tm5-700_moveit_config"), "config", "tm5-700.urdf.xacro"]
    #         ),
    #     ]
    # )
    
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("hm_tm5-700_moveit_config"), "config", "tm5-700.urdf.xacro"]
            ),
            " ",
            "controller_configs:=",
            controllers_configs,
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
    
    visualise_robot_tf_node = Node(
        package="cga_ik",
        executable="visualise_robot_tf",
        name="visualise_robot_tf",
        output="screen",
        parameters=[pose_f_tcp_params],
    )
    
    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robot_description}] # add other parameters here if required
    # )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            robot_description, 
        ],
    )

    nodes_to_start = [
                      visualise_robot_tf_node,       
                      robot_state_publisher_node,
                      ]

    return LaunchDescription(nodes_to_start)