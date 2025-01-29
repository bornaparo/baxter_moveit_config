import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_param_builder import load_xacro
from pathlib import Path


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="If not using gazebo, then it is false"
        ),
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="True",
            description="xacro mapping wheter to use gazebo ign plugin for controllers or not"
        ),
        DeclareLaunchArgument(
            "is_robot_state_publisher_launched",
            default_value="True",
            description="If rsp is launched in other node, don't launch it again"
        ),
        DeclareLaunchArgument(
            "using_real_baxter",
            default_value="False",
            description="If the real baxter is used, then joint_state_broadcaster isn't used and /robot/joint_states is bridged to /joint_states topic"
        )
    ]
    
    baxter_urdf_path = os.path.join(get_package_share_directory("baxter_description"), "urdf", "baxter2_include.urdf.xacro")
    robot_description = {
        "robot_description": load_xacro(Path(baxter_urdf_path), mappings={"gazebo": "True"})
    }
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    
    baxter_share = get_package_share_directory("baxter_moveit_config")
    gui_config_path = os.path.join(baxter_share, "config", "gazebo_gui.config")
    # world_path = os.path.join(baxter_share, "worlds", "my_world.sdf")
    world_path = os.path.join(baxter_share, "worlds", "world_with_desk.sdf")
    gazebo = IncludeLaunchDescription( #ignition
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    # ("gz_args", [" -v 4 -r empty.sdf "] 
                    # ("gz_args", [f" -v 4 -r {world_path} "] 
                    # ("gz_args", [f" -v 4 empty.sdf --gui-config {gui_config_path} "] 
                    ("gz_args", [f" -v 4 -r empty.sdf --gui-config {gui_config_path} "] 
                    # ("gz_args", [f" -v 4 -r {world_path} --gui-config {gui_config_path} "] 
                    )
                ]
            )
    
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "baxter"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    
    gazebo_resource_path = AppendEnvironmentVariable( #bez ovog ti javlja "Unable to find file with URI ..."
        name="GZ_SIM_RESOURCE_PATH",
        value=[
                str(Path(get_package_share_directory("rethink_ee_description")).parent.resolve()), #za gripper
            ]
        )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    
    nodes_to_launch = [
        robot_state_publisher,
        gazebo,
        spawn_entity,
        gazebo_resource_path,
        gz_ros2_bridge,
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [baxter_share, "launch", "baxter_moveit_opaque.launch.py"]
                )
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "use_gazebo": LaunchConfiguration("use_gazebo"),
                "is_robot_state_publisher_launched": LaunchConfiguration("is_robot_state_publisher_launched"),
                "using_real_baxter": LaunchConfiguration("using_real_baxter"),
            }.items()
        ),        
    ]
    
    return LaunchDescription(
        declared_arguments + nodes_to_launch
    )
    