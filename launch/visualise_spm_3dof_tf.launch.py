import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def load_yaml_file_absolute_path(yaml_file):
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    spm_3dof_params = os.path.join(
        get_package_share_directory("cga_ik"),
        "config",
        "spm_3dof_params.yaml"
    )
    geometric_params = load_yaml_file_absolute_path(spm_3dof_params).get("geometric_params", {})

    r_b_str = str(geometric_params.get("r_b", 0.5))
    ratio_c_b_str = str(geometric_params.get("ratio_c_b", 0.3))
    ratio_e_b_str = str(geometric_params.get("ratio_e_b", 0.3))

    urdf_xacro_path = os.path.join(
        get_package_share_directory("cga_ik"),
        "config",
        "spm_3dof.urdf.xacro"
    )
    if not os.path.exists(urdf_xacro_path):
        raise FileNotFoundError(f"URDF file not found at {urdf_xacro_path}")

    # Construct the command to run xacro + pass arguments
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_xacro_path,
            " ",
            "r_b:=", r_b_str, " ",
            "ratio_c_b:=", ratio_c_b_str, " ",
            "ratio_e_b:=", ratio_e_b_str, " ",
        ]
    )
    
    # robot_description_content = xacro.process_file(urdf_xacro_path).toxml()

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str
        )
    }
    
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    visualise_spm_3dof_tf_node = Node(
        package="cga_ik",
        executable="visualise_spm_3dof_tf",
        name="visualise_spm_3dof_tf",
        output="screen",
        parameters=[geometric_params],
    )

    nodes_to_start = [
                      robot_state_publisher_node,
                      visualise_spm_3dof_tf_node,       
                      ]

    return LaunchDescription(nodes_to_start)