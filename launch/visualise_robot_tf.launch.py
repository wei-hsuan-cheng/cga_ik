import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node

def load_yaml_file_absolute_path(yaml_file):
    """Load a YAML file and return the parameters dictionary."""
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    # home_dir = os.path.expanduser("~")
    # sup_tms_params = os.path.join(home_dir, "sup_tms/workspaces/isaac_ros-dev/config/sup_tms_params.yaml")
    
    # pose_f_tcp_params = load_yaml_file_absolute_path(sup_tms_params).get("pose_f_tcp_params", {})

    visualise_robot_tf_node = Node(
        package="cga_ik",
        executable="visualise_robot_tf",
        name="visualise_robot_tf",
        output="screen",
        parameters=[],
    )

    nodes_to_start = [
                      visualise_robot_tf_node,       
                      ]

    return LaunchDescription(nodes_to_start)