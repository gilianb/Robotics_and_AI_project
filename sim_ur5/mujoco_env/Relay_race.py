from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor

workspace_x_lims = [-1.0, -0.45]
workspace_y_lims = [-1.0, -0.45]


def relay_race_transfer(start_position, handover_position):
    """
    Manage the transfer of a cube between two robots.

    :param start_position: Starting position (cube pickup location)
    :param handover_position: Position where the handover occurs
    :param end_position: Final position where the cube is placed
    """

    # Check the limits
    # Check if the target location is within the workspace limits
    if not (workspace_x_lims[0] <= handover_position[0] <= workspace_x_lims[1] and workspace_y_lims[0] <= handover_position[
        1] <= workspace_y_lims[1]):
        raise ValueError("Target location is out of workspace limits")

    # Initial pickup by ur5e_2
    ur5e_2_initial_position = [start_position[0], start_position[1], start_position[2] + 0.1]
    executor.plan_and_move_to_xyz_facing_down("ur5e_2", ur5e_2_initial_position)
    executor.pick_up("ur5e_2", start_position[0], start_position[1], start_position[2] + 0.12)

    # Move ur5e_2 to handover position
    ur5e_2_handover_position = [handover_position[0], handover_position[1], handover_position[2]+0.3]
    executor.plan_and_move_to_xyz_facing_down("ur5e_2", ur5e_2_handover_position)

    # Handover to ur5e_1
    ur5e_1_handover_position = [handover_position[0], handover_position[1], handover_position[2]]
    executor.plan_and_move_to_xyz_facing_down("ur5e_1", ur5e_1_handover_position)

    # turn ur5e_1 head
    ur5e_1_position = list(env.get_agent_joint("ur5e_1"))
    ur5e_1_position[-2]= - ur5e_1_position[-2]
    executor.moveJ("ur5e_1", ur5e_1_position)

    # Movedown ur5e_2
    ur5e_2_handover_position = [handover_position[0], handover_position[1], handover_position[2]+0.25]
    executor.plan_and_move_to_xyz_facing_down("ur5e_2", ur5e_2_handover_position)


handover_position = [-0.6, -0.6, 0.15]
env = SimEnv()
executor = MotionExecutor(env)
start_position = env.object_manager.get_all_block_positions()[0]
relay_race_transfer(start_position, handover_position)