import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def load_yaml_file_absolute_path(yaml_file):
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file)

def launch_setup(context, *args, **kwargs):
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    
    # ROS 2 params
    spm_3dof_params = os.path.join(
        get_package_share_directory("cga_ik"),
        "config",
        "spm_3dof_params.yaml"
    )
    geometric_params = load_yaml_file_absolute_path(spm_3dof_params).get("geometric_params", {})
    motion_params = load_yaml_file_absolute_path(spm_3dof_params).get("motion_params", {})

    # For robot state publisher
    urdf_xacro_path = os.path.join(
        get_package_share_directory("cga_ik"),
        "config",
        "spm_3dof.urdf.xacro",
        # "spm_3dof_simplified.urdf.xacro",
    )
    
    # Lower parts
    r_c = str(geometric_params.get("r_c"))
    ang_b_m = str(geometric_params.get("ang_b_m"))
    r_b = str(geometric_params.get("r_b"))
    # Upper parts
    d = str(geometric_params.get("d"))
    r_e = str(geometric_params.get("r_e"))
    # Sphere radii
    r_s_piv = str(geometric_params.get("r_s_piv"))
    r_s_m = str(geometric_params.get("r_s_m"))
    r_s_elb = str(geometric_params.get("r_s_elb"))
    r_s_epl = str(geometric_params.get("r_s_epl"))
    # Elbow configuration
    krl = str(geometric_params.get("krl"))
    # Re-origining the coordinate system (mechanical origin -> control origin); set initial z-rotation
    th_z_ee_reset = str(geometric_params.get("th_z_ee_reset"))
    # SPM mode
    spm_mode = str(geometric_params.get("spm_mode"))
    
    if not os.path.exists(urdf_xacro_path):
        raise FileNotFoundError(f"URDF file not found at {urdf_xacro_path}")

    # Construct the command to run xacro + pass arguments
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_xacro_path,
            " ",
            # Lower parts
            "r_c:=", r_c, " ",
            "ang_b_m:=", ang_b_m, " ",
            "r_b:=", r_b, " ",
            # Upper parts
            "d:=", d, " ",
            "r_e:=", r_e, " ",
            # Sphere radii
            "r_s_piv:=", r_s_piv, " ",
            "r_s_m:=", r_s_m, " ",
            "r_s_elb:=", r_s_elb, " ",
            "r_s_epl:=", r_s_epl, " ",
            # Elbow configuration
            "krl:=", krl, " ",
            # Re-origining the coordinate system (mechanical origin -> control origin); set initial z-rotation
            "th_z_ee_reset:=", th_z_ee_reset, " ",
            # SPM mode
            "spm_mode:=", spm_mode, " ",
        ]
    )
    
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str
        )
    }
    
    # Rviz config
    rviz_base = os.path.join(
        get_package_share_directory("cga_ik"), 
        "launch"
    )
    rviz_config_file = os.path.join(rviz_base, "spm_3dof_rviz_config.rviz")

    
    # ROS 2 nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    spm_action_server_node = Node(
        package="cga_ik",
        executable="spm_action_server",
        name="spm_action_server",
        output="screen",
        parameters=[geometric_params,
                    motion_params,
                    ],
        condition=IfCondition(use_fake_hardware)
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[],
    )

    nodes_to_start = [
                      robot_state_publisher_node,
                      spm_action_server_node,
                      rviz_node,
                      ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    # Command-line arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)]) 