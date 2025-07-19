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
    #     MoveItConfigsBuilder("moveit_resources_panda") # automatically looking for a pacakage named as moveit_resources_panda_moveit_config
    #     .robot_description(
    #         file_path="config/panda.urdf.xacro",
    #         # mappings={
    #         #     "ros2_control_hardware_type": LaunchConfiguration(
    #         #         "ros2_control_hardware_type"
    #         #     )
    #         # },
    #     )
    #     .robot_description_semantic(file_path="config/panda.srdf")
    #     .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
    #     .planning_scene_monitor(
    #         publish_robot_description=True, publish_robot_description_semantic=True
    #     )
    #     .planning_pipelines(
    #         pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
    #     )
    #     .to_moveit_configs()
    # )
    # only left hand for dual arm
    moveit_config = (
        MoveItConfigsBuilder("dual_arm_panda") # automatically looking for a pacakage named as <whatever_given>_moveit_config
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # ############### test ##############
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        # ############### test ##############
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )


    #####################################################


    # Get parameters for the Pose Tracking node
    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml("config/pose_tracking_settings.yaml")
        .yaml("config/panda_simulated_config_pose_tracking_left_arm.yaml")
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

    dual_left_hand_tracking_node = Node(
        package="moveit_servo",
        executable="servo_dual_left_hand_tracking_demo",
        # prefix=['xterm -e gdb -ex run --args'],
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
    )

    # ros2_control using FakeSystem as hardware
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

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
    )

    left_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_hand_controller", "-c", "/controller_manager"],
    )

    right_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_hand_controller", "-c", "/controller_manager"],
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

    # control_gripper_node = Node(
    #     package='control_gripper',
    #     executable='control_gripper',
    #     name='control_gripper',
    # )

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            dual_left_hand_tracking_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            left_arm_controller_spawner,
            right_arm_controller_spawner,
            left_hand_controller_spawner,
            right_hand_controller_spawner,
            robot_state_publisher,
            webcam_node,
            perception_node,
            # control_gripper_node,
        ]
    )
