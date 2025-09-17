import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
from launch.actions import TimerAction


def generate_launch_description():

    ###############################################
    ###### Common configuration for dual arm ######
    ###############################################

    # MoveIt configuration for dual arm
    moveit_config = (
        MoveItConfigsBuilder("dual_arm_panda_gazebo")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # use Gazebo simulation time / clock for all nodes that read time
    use_sim_time = [{'use_sim_time': True}]

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[*use_sim_time],
    )

    # Publishes tf's for the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[*use_sim_time, moveit_config.robot_description],
    )

    ###############################################
    ###### Gazebo Fortress (ros_gz_sim) ###########
    ###############################################

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_servo")
        + "/config/demo_rviz_dual_hand_tracking.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[*use_sim_time, moveit_config.to_dict()],
    )

    # launch gazebo
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-v", "4", "-r", "empty.sdf"],
        output="screen"
    )

    # Spawn robot into Gazebo from robot_description
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "dual_panda"],
        output="screen",
        parameters=[*use_sim_time],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    ###############################################
    ###### Perception Nodes #######################
    ###############################################

    # Pose perception and gesture recognition nodes
    webcam_node = Node(
        package='pose_gesture_perception',
        executable='webcam_img_provider',
        name='webcam_img_provider',
        parameters=[*use_sim_time],
    )

    perception_dual_arm_node = Node(
        package='pose_gesture_perception',
        executable='pose_gesture_processer_dual_arm',
        name='pose_gesture_processer_dual_arm',
        parameters=[*use_sim_time],
    )

    ######################
    ###### Left arm ######
    ######################

    left_servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml("config/pose_tracking_settings.yaml")
        .yaml("config/panda_simulated_config_pose_tracking_left_arm.yaml")
        .to_dict()
    }

    dual_left_hand_tracking_node = Node(
        package="moveit_servo",
        executable="servo_dual_left_hand_tracking_demo",
        output="screen",
        remappings=[('target_pose', '/left_arm/target_pose')],
        parameters=[
            *use_sim_time,
            moveit_config.to_dict(),
            left_servo_params,
        ],
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
        parameters=[*use_sim_time],
    )

    left_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_hand_controller", "-c", "/controller_manager"],
        parameters=[*use_sim_time],
    )

    control_left_gripper_node = Node(
        package='control_gripper',
        executable='control_left_gripper',
        name='control_left_gripper',
        parameters=[*use_sim_time],
    )

    #######################
    ###### Right arm ######
    #######################

    right_servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml("config/pose_tracking_settings.yaml")
        .yaml("config/panda_simulated_config_pose_tracking_right_arm.yaml")
        .to_dict()
    }

    dual_right_hand_tracking_node = Node(
        package="moveit_servo",
        executable="servo_dual_right_hand_tracking_demo",
        output="screen",
        remappings=[('target_pose', '/right_arm/target_pose')],
        parameters=[
            *use_sim_time,
            moveit_config.to_dict(),
            right_servo_params,
        ],
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
        parameters=[*use_sim_time],
    )

    right_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_hand_controller", "-c", "/controller_manager"],
        parameters=[*use_sim_time],
    )

    control_right_gripper_node = Node(
        package='control_gripper',
        executable='control_right_gripper',
        name='control_right_gripper',
        parameters=[*use_sim_time],
    )

    ###############################################
    ###### Delay servo nodes until ready ##########
    ###############################################

    # Delay servo nodes until Gazebo + controllers are ready
    delayed_dual_hand_tracking_nodes = TimerAction(
        period=5.0,
        actions=[
            dual_left_hand_tracking_node,
            dual_right_hand_tracking_node,
        ],
    )


    ###############################################
    ###### Return launch description ##############
    ###############################################

    return LaunchDescription(
        [
            # Gazebo and robot spawn
            gazebo,
            spawn_robot,
            clock_bridge,

            # MoveIt / RViz
            rviz_node,
            robot_state_publisher,
            joint_state_broadcaster_spawner,

            # Perception
            webcam_node,
            perception_dual_arm_node,

            # Left arm
            left_arm_controller_spawner,
            left_hand_controller_spawner,
            control_left_gripper_node,

            # Right arm
            right_arm_controller_spawner,
            right_hand_controller_spawner,
            control_right_gripper_node,

            # Delayed servo nodes
            delayed_dual_hand_tracking_nodes,
        ]
    )
