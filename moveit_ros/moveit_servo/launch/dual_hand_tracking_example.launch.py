import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction


def generate_launch_description():

    ###############################################
    ###### Common configuration for dual arm ######
    ###############################################


    ### MoveIt configuration for dual arm
    moveit_config = (
        MoveItConfigsBuilder("dual_arm_panda") # automatically looking for a pacakage named as <whatever_given>_moveit_config
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    ### RViz
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
        parameters=[moveit_config.to_dict()],
    )

    ### Publishes tf's for the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

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
    )

    ### ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("dual_arm_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    ### pose percetion and gesture recognition nodes
    webcam_node = Node(
        package='pose_gesture_perception',
        executable='webcam_img_provider',
        name='webcam_img_provider',
    )

    perception_dual_arm_node = Node(
        package='pose_gesture_perception',
        executable='pose_gesture_processer_dual_arm',
        name='pose_gesture_processer_dual_arm',
    )


    ######################
    ###### Left arm ######
    ######################


    ### moveit servo parameters for left arm
    left_servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml("config/pose_tracking_settings.yaml")
        .yaml("config/panda_simulated_config_pose_tracking_left_arm.yaml")
        .to_dict()
    }

    ### left hand tracking
    dual_left_hand_tracking_node = Node(
        package="moveit_servo",
        executable="servo_dual_left_hand_tracking_demo",
        output="screen",
        remappings=[('target_pose', '/left_arm/target_pose')],
        parameters=[
            moveit_config.to_dict(),
            left_servo_params,
        ],
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
    )

    ### left gripper control
    left_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_hand_controller", "-c", "/controller_manager"],
    )

    control_left_gripper_node = Node(
        package='control_gripper',
        executable='control_left_gripper',
        name='control_left_gripper',
    )


    #######################
    ###### Right arm ######
    #######################


    ### moveit servo parameters for right arm
    right_servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml("config/pose_tracking_settings.yaml")
        .yaml("config/panda_simulated_config_pose_tracking_right_arm.yaml")
        .to_dict()
    }

    ### right hand tracking
    dual_right_hand_tracking_node = Node(
        package="moveit_servo",
        executable="servo_dual_right_hand_tracking_demo",
        output="screen",
        remappings=[('target_pose', '/right_arm/target_pose')],
        parameters=[
            moveit_config.to_dict(),
            right_servo_params,
        ],
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
    )

    ### right gripper control
    right_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_hand_controller", "-c", "/controller_manager"],
    )

    control_right_gripper_node = Node(
        package='control_gripper',
        executable='control_right_gripper',
        name='control_right_gripper',
    )

    ### Delay the start of some nodes to ensure that the system is ready
    delayed_dual_left_hand_tracking_nodes = TimerAction(
        period=3.0,
        actions=[
            dual_left_hand_tracking_node,
            dual_right_hand_tracking_node,
        ],
    )

    return LaunchDescription(
        [
            ### common moveit configuration
            rviz_node,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            ros2_control_node,
            webcam_node,
            perception_dual_arm_node,
            ### left arm configuration
            # dual_left_hand_tracking_node,
            left_arm_controller_spawner,
            left_hand_controller_spawner,
            control_left_gripper_node,
            ### right arm configuration
            # dual_right_hand_tracking_node,
            right_arm_controller_spawner,
            right_hand_controller_spawner,
            control_right_gripper_node,
            ### delayed nodes
            delayed_dual_left_hand_tracking_nodes,
        ]
    )
