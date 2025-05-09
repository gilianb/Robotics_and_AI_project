from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor

# TODO: check the limits  
workspace_x_lims = [-1.0, -0.45]
workspace_y_lims = [-1.0, -0.45]


def create_cube_stack(cube_positions, target_location, x_lims = workspace_x_lims, y_lim = workspace_y_lims):
    """
    Create a stable stack of cubes at the specified target location.

    :param env: Simulation environment object
    :param cube_positions: List of cube positions
    :param target_location: Target location for stack placement
    """

    #Check the limits
    # # Check if the target location is within the workspace limits
    # if not (x_lims[0] <= target_location[0] and target_location[0] <= x_lims[1] and y_lim[0] <= target_location[1] and target_location[1]<= y_lim[1]):
    #     raise ValueError("Target location is out of workspace limits")

    # Calculate the height of each cube (assuming all cubes have the same height)
    cube_height = 0.05  # Example height, adjust as necessary

    # Place each cube at the target location, stacking them vertically
    for i, position in enumerate(cube_positions):
        # Calculate the new position for the cube in the stack
        new_init_position = [
            position[0],
            position[1],
            position[2] +  0.1
        ]
        new_target_position = [
            target_location[0],
            target_location[1],
            target_location[2] + i * cube_height
        ]

        executor.plan_and_move_to_xyz_facing_down("ur5e_2",new_init_position)
        executor.pick_up("ur5e_2",position[0],position[1],position[2]+0.12)
        executor.plan_and_move_to_xyz_facing_down("ur5e_2",new_target_position)
        executor.put_down("ur5e_2", target_location[0],target_location[1],target_location[2])
        executor.plan_and_move_to_xyz_facing_down("ur5e_2",new_target_position)
        executor.wait(4)

# Locate an area where both robots can reach
# You can choose any location in the area to place the blocks

# block_position = [
#     [-0.7, -0.6, 0.03],
#     [-0.7, -0.7, 0.03],
#     [-0.7, -0.8, 0.03],
#     [-0.7, -0.9, 0.03]]

# Create the simulation environment and the executor
env = SimEnv()
executor = MotionExecutor(env)

# Add blocks to the world by enabling the randomize argument and setting the block position in the reset function of the SimEnv class 
# env.reset(randomize=False, block_positions=block_position)

# # moveJ is utilized when the robot's joints are clear to you but use carefully because there is no planning here
# move_to = [1.305356658502026, -0.7908733209856437, 1.4010098471710881, 4.102251451313659, -1.5707962412281837, -0.26543967541515895]
# executor.moveJ("ur5e_2", move_to)
#
# executor.pick_up("ur5e_2", -0.7, -0.6, 0.15)
#
# executor.plan_and_move_to_xyz_facing_down("ur5e_2", [-0.7, -0.7, 0.15])
# executor.put_down("ur5e_2", -0.7, -0.7, 0.20)
#
# executor.wait(4)


block_position_dict= env.object_manager.get_all_block_positions_dict()
block_positions = [list(map(float, pos)) for pos in env.object_manager.get_all_block_positions()]


target_location = [-0.5, -0.5, 0.15]
create_cube_stack(block_positions, target_location)









