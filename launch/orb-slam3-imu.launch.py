import os
import json
import yaml
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart

IS_MAPPING = os.getenv("IS_MAPPING") == "1"
STORAGE_PATH = os.getenv("STORAGE_PATH")
INPUT_IMU_BIAS_FILE = os.path.join("/", "calib", "imu.json")
IMU_TYPE = "vectornav"  # or 'xsens'

if IS_MAPPING is None:
    print("IS_MAPPING is not set")
    exit(1)
elif STORAGE_PATH is None:
    print("STORAGE_PATH is not set")
    exit(1)

map_name = os.path.join(STORAGE_PATH, "orb_slam3_atlas")


def generate_launch_description():
    ld = LaunchDescription()
    share_folder = get_package_share_directory("orbslam3")

    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation time"
        )
    )

    bias_x = 0.0
    bias_y = 0.0
    bias_z = 0.0

    if os.path.exists(INPUT_IMU_BIAS_FILE):
        with open(INPUT_IMU_BIAS_FILE, "r") as f:
            bias_data = json.load(f)
            bias_x = bias_data[IMU_TYPE]["angular_velocities"]["x"]
            bias_y = bias_data[IMU_TYPE]["angular_velocities"]["y"]
            bias_z = bias_data[IMU_TYPE]["angular_velocities"]["z"]
    else:
        print("No bias file found, using default values")

    if IMU_TYPE == "vectornav":
        namespace = LaunchConfiguration("vn100_ns")
        vectornav_namespace_launch_arg = DeclareLaunchArgument(
            "vn100_ns", default_value=IMU_TYPE
        )

        config_file = os.path.join(
            share_folder, "config", "stereo-inertial", "_vn100.yaml"
        )

        print(f"Biases:\n  x={bias_x}\n  y={bias_y}\n  z={bias_z}\n")
        bias_compensator_node = Node(
            package="norlab_imu_tools",
            executable="imu_bias_compensator_node",
            name="bias_compensator",
            namespace=namespace,
            output="both",
            parameters=[
                config_file,
                {"bias_x": bias_x, "bias_y": bias_y, "bias_z": bias_z},
            ],
            remappings=[
                ("imu_topic_in", "data_raw"),
                ("bias_topic_in", "bias"),
                ("imu_topic_out", "data_unbiased"),
            ],
            arguments=[
                "--ros-args",
                "--log-level",
                "warn",
            ],
        )
        ld.add_action(vectornav_namespace_launch_arg)
        ld.add_action(bias_compensator_node)
    elif IMU_TYPE == "xsens":
        raise NotImplementedError("xsens IMU is not yet supported")

    default_config_file = os.path.join(
        share_folder,
        "config",
        "stereo-inertial",
        "zedx.yaml",
    )
    temp_config_file = os.path.join(
        share_folder,
        "config",
        "stereo-inertial",
        "temp.yaml",
    )
    print("Updating configuration file")
    print(f"Copying default from {default_config_file} to {temp_config_file}")
    with open(default_config_file, "r") as fin, open(temp_config_file, "w") as fout:
        lines = fin.readlines()
        for line in lines:
            fout.write(line)
            if line.startswith("Atlas"):
                if IS_MAPPING:
                    print("Will save atlas to ", map_name)
                    fout.write(f"System.SaveAtlasToFile: {map_name}\n")
                else:
                    print("Will load atlas from ", map_name)
                    fout.write(f"System.LoadAtlasFromFile: {map_name}\n")
                    fout.write("System.LocalizationMode: True\n")

    orbslam3_node = Node(
        package="orbslam3",
        executable="stereo-inertial",
        name="orbslam3_stereo",
        output="screen",
        sigterm_timeout="30",  # Wait 30 seconds before escalating to SIGTERM
        sigkill_timeout="5",  # Wait 5 more seconds before SIGKILL
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
            "--log-level",
            "rcl:=INFO",
            "--log-level",
            "rmw_fastrtps_cpp:=INFO",
            "--log-level",
            "rclcpp:=INFO",
        ],
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "rectify": False,
                "output_folder": STORAGE_PATH,
                "vocabulary": os.path.join(
                    share_folder,
                    "vocabulary",
                    "ORBvoc.txt",
                ),
                "config": temp_config_file,
            }
        ],
        remappings=[
            ("camera_pose", "estimated_pose"),
            ("image_left", "zedx/left/image_rect"),
            ("image_right", "zedx/right/image_rect"),
            ("imu", "vectornav/data_unbiased"),
        ],
    )
    ld.add_action(orbslam3_node)
    return ld
