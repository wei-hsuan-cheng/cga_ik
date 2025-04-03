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
    spherical_robot_params = os.path.join(home_dir, "ros2_ws/test_ws/src/cga_ik/config/spherical_robot_params.yaml")
    
    geometric_params = load_yaml_file_absolute_path(spherical_robot_params).get("geometric_params", {})
    
    urdf_path = os.path.join(get_package_share_directory("cga_ik"), "config", "tm5-700.urdf.xacro")

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found at {urdf_path}")
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_path,
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    
    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="both",
    #     parameters=[
    #         robot_description, 
    #     ],
    # )
    
    visualise_spherical_robot_tf_node = Node(
        package="cga_ik",
        executable="visualise_spherical_robot_tf",
        name="visualise_spherical_robot_tf",
        output="screen",
        parameters=[geometric_params,
                    ],
    )
    
    

    nodes_to_start = [
                    #   robot_state_publisher_node,
                      visualise_spherical_robot_tf_node,       
                      ]

    return LaunchDescription(nodes_to_start)