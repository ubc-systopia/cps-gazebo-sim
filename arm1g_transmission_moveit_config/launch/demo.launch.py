from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch, generate_move_group_launch

def generate_launch_description():
    # Build the MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "mutli_arm",
        package_name="arm1g_transmission_moveit_config"
    ).to_moveit_configs()
    
    # Launch essential MoveIt components (no RViz)
    rsp_launch = generate_rsp_launch(moveit_config)
    move_group_launch = generate_move_group_launch(moveit_config)

    return LaunchDescription([
        rsp_launch,
        move_group_launch,
    ])
