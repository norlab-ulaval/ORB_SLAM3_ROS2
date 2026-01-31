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

    default_config_file = os.path.join(
        share_folder,
        "config",
        "stereo",
        "zedx.yaml",
    )
    temp_config_file = os.path.join(
        share_folder,
        "config",
        "stereo",
        "temp.yaml",
    )
    print("Updating configuration file")
    print("Copying default from zedx.yaml to temp.yaml")
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
        executable="stereo",
        name="orbslam3_stereo",
        output="screen",
        sigterm_timeout="30",  # Wait 30 seconds before escalating to SIGTERM
        sigkill_timeout="5",  # Wait 5 more seconds before SIGKILL
        arguments=[
            "--ros-args",
            "--log-level",
            "debug",
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
        ],
    )
    ld.add_action(orbslam3_node)
    return ld
