import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    # --- Args ---
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='my_world.sdf',
        description='Name of the Gazebo world file to load'
    )

    # --- Paths ---
    pkg_bme = get_package_share_directory('bme_gazebo_sensors')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # --- Gazebo resources (safe append) ---
    gazebo_models_path = "/home/ubuntu/gazebo_models"
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if os.environ["GZ_SIM_RESOURCE_PATH"]:
        os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep
    os.environ["GZ_SIM_RESOURCE_PATH"] += gazebo_models_path

    # Build the world path as a substitution (FIX)
    world_path = PathJoinSubstitution([pkg_bme, 'worlds', LaunchConfiguration('world')])
    
    
    # ros_gz_sim's gz_sim.launch.py typically accepts 'gz_args' as a single string
    # Example: " -r -v 1 <world_path>"
    gz_args = [TextSubstitution(text=" -r -v 1 "), world_path]

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'true'
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(gazebo_launch)
    return ld

