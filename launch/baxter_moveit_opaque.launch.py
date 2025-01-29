import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from launch_param_builder import load_yaml, load_xacro
from pathlib import Path

import xacro

def launch_setup(context, *args, **kwargs):
    print("[MOJE] rviz_config LaunchConfiguration:", LaunchConfiguration("rviz_config").perform(context))
    print("[MOJE] use_sim_time LaunchConfiguration:", LaunchConfiguration("use_sim_time").perform(context))
    print("[MOJE] use_gazebo LaunchConfiguration:", LaunchConfiguration("use_gazebo").perform(context))
    print("[MOJE] is_robot_state_publisher_launched LaunchConfiguration:", LaunchConfiguration("is_robot_state_publisher_launched").perform(context))
    print("[MOJE] using_real_baxter LaunchConfiguration:", LaunchConfiguration("using_real_baxter").perform(context))
    
    baxter_urdf_path = os.path.join(get_package_share_directory("baxter_description"), "urdf", "baxter2_include.urdf.xacro")
    baxter_srdf_path = os.path.join(get_package_share_directory("baxter_moveit_config"), "config", "baxter.srdf.xacro")
    baxter_share_config = os.path.join(get_package_share_directory("baxter_moveit_config"), "config")

    moveit_config = {
        "robot_description": {
            "robot_description": load_xacro(Path(baxter_urdf_path), mappings={"gazebo": LaunchConfiguration("use_gazebo").perform(context=context)}),
        },
        "robot_description_semantic": {
            "robot_description_semantic": xacro.process_file(baxter_srdf_path).toxml(),
        },
        "robot_description_kinematics": {
            "robot_description_kinematics": load_yaml(Path(os.path.join(baxter_share_config, "kinematics.yaml"))),
        },
        "planning_pipelines": {
            "planning_pipelines": ["ompl", "isaac_ros_cumotion"],
            "default_planning_pipeline": "ompl",
            "ompl": load_yaml(Path(os.path.join(baxter_share_config, "ompl_planning.yaml"))), #planning_pipelines["ompl"]
        },
        "trajectory_execution": {
            "moveit_manage_controllers": True,
        },
        "planning_scene_monitor": {
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
        },
        "joint_limits": {
            "robot_description_planning": load_yaml(Path(os.path.join(baxter_share_config, "joint_limits.yaml"))),
        }
    }
    
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config["robot_description"],
            moveit_config["robot_description_semantic"],
            moveit_config["robot_description_kinematics"],
            moveit_config["planning_pipelines"],
            moveit_config["trajectory_execution"],
            load_yaml(Path(os.path.join(baxter_share_config, "moveit_controllers.yaml"))), #trajectory execution params, moveit kontroleri
            moveit_config["planning_scene_monitor"],
            moveit_config["joint_limits"],
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            
            move_group_capabilities,
        ]
    )

    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("baxter_moveit_config"), "config", rviz_base]
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[ 
            moveit_config["robot_description"],
            moveit_config["robot_description_semantic"],
            moveit_config["robot_description_kinematics"],
            moveit_config["planning_pipelines"],
            moveit_config["joint_limits"],
            {
                "use_sim_time": LaunchConfiguration("use_sim_time")
            }
        ]
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config["robot_description"], {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=UnlessCondition(LaunchConfiguration("is_robot_state_publisher_launched")), #launchat ce se samo ako je ovo False
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("baxter_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node( #potreban kad koristis samo rviz
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config["robot_description"], ros2_controllers_path, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="both",
        condition=UnlessCondition(LaunchConfiguration("use_gazebo")) #zato jer ako koristis gazebo onda gazebo spawna controller_managera, loada pluginove itd
    )

    joint_state_broadcaster_spawner = Node( #da bi znao stanje (tj zakrete) zglobova
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "500",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=UnlessCondition(LaunchConfiguration("using_real_baxter")) #ako koristis pravog baxtera onda neces koristit ovaj joint_state_broadcaster koji cita ove iz simulacije nego ces koristit ove od pravog baxtera tako sta ces bridgeat /robot/joint_states u /joint_states        
    )

    right_arm_controller_spawner = Node( #kontroler koji upravlja rukom
        package="controller_manager",
        executable="spawner",
        arguments=["baxter_right_arm_controller", "--controller-manager-timeout", "500", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    
    left_arm_controller_spawner = Node( #kontroler koji upravlja rukom
        package="controller_manager",
        executable="spawner",
        arguments=["baxter_left_arm_controller", "--controller-manager-timeout", "500", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    right_hand_controller_spawner = Node( #kontroler koji upravlja gripperom
        package="controller_manager",
        executable="spawner",
        arguments=["baxter_right_hand_controller", "--controller-manager-timeout", "500", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    
    left_hand_controller_spawner = Node( #kontroler koji upravlja gripperom
        package="controller_manager",
        executable="spawner",
        arguments=["baxter_left_hand_controller", "--controller-manager-timeout", "500", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    
    nodes_to_start = [
        rviz_node,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        right_arm_controller_spawner,
        left_arm_controller_spawner,
        right_hand_controller_spawner,
        left_hand_controller_spawner,
    ]

    return nodes_to_start

def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument(
            "rviz_config",
            default_value="baxter_rviz_config.rviz",
            description="RViz configuration file",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="If not using gazebo, then it is false"
        ),
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="False",
            description="xacro mapping whether to use gazebo ign plugin for controllers or not"
        ),
        DeclareLaunchArgument(
            "is_robot_state_publisher_launched",
            default_value="False",
            description="If rsp is launched in other node, don't launch it again"
        ),
        DeclareLaunchArgument(
            "using_real_baxter",
            default_value="False",
            description="If the real baxter is used, then joint_state_broadcaster isn't used and /robot/joint_states is bridged to /joint_states topic"
        )
    ]
    
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
    