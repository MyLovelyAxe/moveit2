import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    #####################################################
    # original
    # moveit_config = (
    #     MoveItConfigsBuilder("moveit_resources_panda")
    #     .robot_description(file_path="config/panda.urdf.xacro")
    #     .planning_scene_monitor(
    #         publish_robot_description=True, publish_robot_description_semantic=True
    #     )
    #     .to_moveit_configs()
    # )

    # test
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            # mappings={
            #     "ros2_control_hardware_type": LaunchConfiguration(
            #         "ros2_control_hardware_type"
            #     )
            # },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )
    #####################################################


    # Get parameters for the Pose Tracking node
    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml("config/pose_tracking_settings.yaml")
        .yaml("config/panda_simulated_config_pose_tracking.yaml")
        .to_dict()
    }

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_servo")
        + "/config/demo_rviz_hand_tracking.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # prefix=['xterm -e gdb -ex run --args'],
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.to_dict()],
    )

    # Publishes tf's for the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # A node to publish world -> panda_link0 transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    hand_tracking_node = Node(
        package="moveit_servo",
        executable="servo_hand_tracking_demo",
        # prefix=['xterm -e gdb -ex run --args'],
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
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

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller", "-c", "/controller_manager"],
    )

    webcam_node = Node(
        package='pose_gesture_perception',
        executable='webcam_img_provider',
        name='webcam_img_provider',
    )

    perception_node = Node(
        package='pose_gesture_perception',
        executable='pose_gesture_processer',
        name='pose_gesture_processer',
    )

    control_gripper_node = Node(
        package='control_gripper',
        executable='control_gripper',
        name='control_gripper',
    )

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            hand_tracking_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            panda_arm_controller_spawner,
            panda_hand_controller_spawner,
            robot_state_publisher,
            webcam_node,
            perception_node,
            control_gripper_node,
        ]
    )
