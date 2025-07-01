from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arm1", package_name="arm1f_moveit_config").to_moveit_configs()
    
    # Ensure that the interactive marker server is enabled
    # This should be the default behavior, but making it explicit
    return generate_demo_launch(moveit_config)
