
from motion_planning.motion_planner import MotionPlanner
from motion_planning.geometry_and_transforms import GeometryAndTransforms
from lab_ur5.manipulation.manipulation_controller import ManipulationController
from lab_ur5.robot_inteface.robots_metadata import ur5e_1, ur5e_2


# check the limits
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
        # new_init_position = [
        #     position[0],
        #     position[1],
        #     position[2] +  0.1
        # ]
        # New position for the cube in the stack
        new_target_position = [
            target_location[0],
            target_location[1],
            target_location[2] + i * cube_height
        ]

        executor.move_home()
        executor.pick_up(position[0],position[1],0,position[2]+0.12)
        executor.plan_and_move_to_xyzrz(new_target_position[0],new_target_position[1],new_target_position[2],0)
        executor.put_down(new_target_position[0],new_target_position[1],0,new_target_position[2])
        executor.plan_and_move_to_xyzrz(new_target_position[0],new_target_position[1],new_target_position[2],0)
        # executor.wait(4)

block_positions = [
    [-0.7, -0.6, 0.03],
    [-0.7, -0.7, 0.03],
    [-0.7, -0.8, 0.03],
    [-0.7, -0.9, 0.03]]

target_location = [-0.5, -0.5, 0.15]

#create the robot
motion_planner = MotionPlanner()
gt = GeometryAndTransforms.from_motion_planner(motion_planner)
executor = ManipulationController(ur5e_2["ip"], ur5e_2["name"], motion_planner, gt)
executor.speed = 0.3
executor.acceleration = 0.3
create_cube_stack(block_positions,target_location)










