from motion_planning.motion_planner import MotionPlanner
from motion_planning.geometry_and_transforms import GeometryAndTransforms
from lab_ur5.manipulation.manipulation_controller import ManipulationController
from lab_ur5.robot_inteface.robots_metadata import ur5e_1, ur5e_2

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

    executor_1.move_home()
    executor_2.move_home()
    # Initial pickup by ur5e_2
    executor_2.pick_up( start_position[0], start_position[1],0, start_position[2] + 0.12)
    executor_2.move_home()

    # Handover to ur5e_1
    executor_1.plan_and_move_to_xyzrz(handover_position[0], handover_position[1], handover_position[2], 0)


    # turn ur5e_1 head
    ur5e_1_position = list(executor_1.getActualQ())
    ur5e_1_position[-2] = -ur5e_1_position[-2]
    executor_1.moveL_relative(ur5e_1_position)

    # Move ur5e_2 to handover position
    executor_2.plan_and_move_to_xyzrz(handover_position[0], handover_position[1], handover_position[2]+0.4, 0.78)


    # Movedown ur5e_2
    executor_2.plan_and_move_to_xyzrz(handover_position[0], handover_position[1], handover_position[2]+0.25, 0)

    executor_2.release_grasp()

    executor_1.grasp()

    executor_1.move_home()




block_positions = [
    [-0.7, -0.6, 0.03],
    [-0.7, -0.7, 0.03],
    [-0.7, -0.8, 0.03],
    [-0.7, -0.9, 0.03]]


handover_position=[-0.5, -0.5, 0.05]

start_position = [-0.5, -0.5, 0.03]

#create the robot
motion_planner = MotionPlanner()
gt = GeometryAndTransforms.from_motion_planner(motion_planner)
executor_1 = ManipulationController(ur5e_1["ip"], ur5e_1["name"], motion_planner, gt)
executor_2 = ManipulationController(ur5e_2["ip"], ur5e_2["name"], motion_planner, gt)

executor_1.speed = 2.
executor_1.acceleration = 1.
executor_2.speed = 2.
executor_2.acceleration = 1.
relay_race_transfer(start_position, handover_position)








