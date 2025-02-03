from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor

# TODO: check the limits
workspace_x_lims = [-1.0, -0.45]
workspace_y_lims = [-1.0, -0.45]


def create_cube_stack(cube_positions, target_location):
    """
    Create a stable stack of cubes at the specified target location.

    :param env: Simulation environment object
    :param cube_positions: List of cube positions
    :param target_location: Target location for stack placement
    """

    #Check the limits
    # Check if the target location is within the workspace limits
    if not (workspace_x_lims[0] <= target_location[0] <= workspace_x_lims[1] and workspace_y_lims[0] <= target_location[
        1] <= workspace_y_lims[1]):
        raise ValueError("Target location is out of workspace limits")

    # Calculate the height of each cube (assuming all cubes have the same height)
    cube_height = 0.05  # Example height, adjust as necessary

    # Place each cube at the target location, stacking them vertically
    for i, position in enumerate(cube_positions):
        # Initial position for the robot to be above the cube
        new_init_position = [
            position[0],
            position[1],
            position[2] +  0.1
        ]
        # New position for the cube in the stack
        new_target_position = [
            target_location[0],
            target_location[1],
            target_location[2] + i * cube_height
        ]

        executor.plan_and_move_to_xyz_facing_down("ur5e_2",new_init_position)
        executor.pick_up("ur5e_2",position[0],position[1],position[2]+0.12)
        executor.plan_and_move_to_xyz_facing_down("ur5e_2",new_target_position)
        executor.put_down("ur5e_2", new_target_position[0],new_target_position[1],new_target_position[2])
        executor.plan_and_move_to_xyz_facing_down("ur5e_2",new_target_position)
        executor.wait(4)

# block_position = [
#     [-0.7, -0.6, 0.03],
#     [-0.7, -0.7, 0.03],
#     [-0.7, -0.8, 0.03],
#     [-0.7, -0.9, 0.03]]

target_location = [-0.5, -0.5, 0.15]

env = SimEnv()
executor = MotionExecutor(env)
block_position_dict= env._object_manager.get_all_block_positions_dict()
block_positions = [list(map(float, pos)) for pos in env._object_manager.get_all_block_positions()]
print(block_positions)
create_cube_stack(block_positions, target_location)









