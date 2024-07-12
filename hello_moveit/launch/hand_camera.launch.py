# import os
from typing import Union

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    calibration_type = LaunchConfiguration('calibration_type')
    # marker_size = LaunchConfiguration("marker_size")
    # image_topic = LaunchConfiguration("image_topic")
    # camera_info_topic = LaunchConfiguration("camera_info_topic")
    # tracking_base_frame = LaunchConfiguration("tracking_base_frame")
    # marker_id_list = LaunchConfiguration("marker_id_list")

    nodes_to_start:list[Union[Node, IncludeLaunchDescription, GroupAction,
                              TimerAction]] = []

    ## include realsense2_camera
    launch_hand_realsense2_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ]),
        launch_arguments={
            'camera_name': 'hand_camera',
            'pointcloud.enable': 'true',
            'serial_no': '_128422272064',
            'depth_module.profile': '848x480x10',
            'align_depth.enable': 'true'
        }.items(),
        condition=IfCondition(
            PythonExpression([
                str(context.perform_substitution(calibration_type) == 'eye_in_hand'),
                ' or ',
                str(context.perform_substitution(use_fake_hardware) == 'false')
            ])
        ),
    )
    nodes_to_start.append(launch_hand_realsense2_camera)

    ## include aruco recognition
    # launch_aruco_recognition = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('ros2_aruco'), 'launch', 'aruco_recognition.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'marker_size': context.perform_substitution(marker_size),
    #         'aruco_dictionary_id': 'DICT_5X5_250',
    #         'image_topic': context.perform_substitution(image_topic),
    #         'camera_info_topic': context.perform_substitution(camera_info_topic),
    #         'tracking_base_frame': context.perform_substitution(tracking_base_frame),
    #         'marker_id_list': marker_id_list,
    #     }.items(),
    #     condition=IfCondition(
    #         PythonExpression([
    #             str(context.perform_substitution(calibration_type) != '')
    #         ])
    #     ),
    # )
    # nodes_to_start.append(launch_aruco_recognition)

    ## include easy_handeye2
    # use the marker at index 0 in the marker_id_list to perform handeye calibration
    # calibration_marker = ast.literal_eval(context.perform_substitution(marker_id_list))[0]
    # launch_easy_handeye2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('easy_handeye2'), 'launch', 'calibrate.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'calibration_type': context.perform_substitution(calibration_type),
    #         'tracking_base_frame': context.perform_substitution(tracking_base_frame),
    #         'tracking_marker_frame': f'aruco_marker_{calibration_marker}',
    #         'robot_base_frame': 'base_link',
    #         'robot_effector_frame': 'tool0',
    #         'name': f'easy_handeye2_{context.perform_substitution(calibration_type)}',
    #     }.items(),
    #     condition=IfCondition(
    #         PythonExpression([
    #             str(context.perform_substitution(calibration_type) != '')
    #         ])
    #     )
    # )
    # easy_handeye2_delay = TimerAction(period=5.0, actions=[launch_easy_handeye2])
    # nodes_to_start.append(easy_handeye2_delay)

    # static transform publisher
    calibration_yaml = load_yaml("hello_moveit", "config/easy_handeye2_eye_in_hand.calib")
    hand_camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hand_camera_tf_publisher',
        arguments=[
            "--x", str(calibration_yaml["transform"]["translation"]["x"]),
            "--y", str(calibration_yaml["transform"]["translation"]["y"]),
            "--z", str(calibration_yaml["transform"]["translation"]["z"]),
            "--qx", str(calibration_yaml["transform"]["rotation"]["x"]),
            "--qy", str(calibration_yaml["transform"]["rotation"]["y"]),
            "--qz", str(calibration_yaml["transform"]["rotation"]["z"]),
            "--qw", str(calibration_yaml["transform"]["rotation"]["w"]),
            "--frame-id", calibration_yaml["parameters"]["robot_effector_frame"],
            "--child-frame-id", calibration_yaml["parameters"]["tracking_base_frame"],
        ],
        condition=IfCondition(
            PythonExpression([
                str(context.perform_substitution(calibration_type) != 'eye_in_hand'),
            ])
        ),
    )
    nodes_to_start.append(hand_camera_tf_node)

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description=
            "Indicate whether robot is running with fake hardware mirroring command to its states.",
            choices=['true', 'false'],
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "calibration_type",
            default_value="",
            description="easy_handeye2_calibration calibration type"
        ))
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "marker_size",
    #         default_value='0.050',
    #         description="aruco marker size."
    #     ))
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "image_topic",
    #         description="image topic name.",
    #         default_value='/camera/color/image_rect_raw'
    #     ))
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "camera_info_topic",
    #         description="camera info topic name.",
    #         default_value='/camera/color/camera_info'
    #     ))
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "tracking_base_frame",
    #         description=".",
    #         default_value='camera_color_optical_frame'
    #     ))
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "marker_id_list",
    #         description="list of marker IDs to be detected.",
    #         default_value='[0, 1, 2, 3]'
    #     ))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
