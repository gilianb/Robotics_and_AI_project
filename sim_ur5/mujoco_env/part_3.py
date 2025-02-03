# import time
#
# from sim_ur5.mujoco_env.sim_env import SimEnv
# from sim_ur5.motion_planning.motion_executor import MotionExecutor
#
# # TODO: check the limits
# workspace_x_lims = [-1.0, -0.45]
# workspace_y_lims = [-1.0, -0.45]
#
#
# def create_cube_stack(cube_positions, target_location, x_lims = workspace_x_lims, y_lim = workspace_y_lims):
#     """
#     Create a stable stack of cubes at the specified target location.
#
#     :param env: Simulation environment object
#     :param cube_positions: List of cube positions
#     :param target_location: Target location for stack placement
#     """
#
#     #Check the limits
#     # Check if the target location is within the workspace limits
#     # if not (x_lims[0] <= target_location[0] and target_location[0] <= x_lims[1] and y_lim[0] <= target_location[1] and target_location[1]<= y_lim[1]):
#     #     raise ValueError("Target location is out of workspace limits")
#
#     # Calculate the height of each cube (assuming all cubes have the same height)
#     cube_height = 0.05  # Example height, adjust as necessary
#
#
#     # Place each cube at the target location, stacking them vertically
#     for i, position in enumerate(cube_positions):
#         # Initial position for the robot to be above the cube
#         new_init_position = [
#             position[0],
#             position[1],
#             position[2] +  0.01
#         ]
#         # New position for the cube in the stack
#         new_target_position = [
#             target_location[0],
#             target_location[1],
#             target_location[2] + i * cube_height
#         ]
#
#         executor.plan_and_move_to_xyz_facing_down("ur5e_2",new_init_position)
#         executor.pick_up("ur5e_2",position[0],position[1],position[2]+0.02)
#         executor.plan_and_move_to_xyz_facing_down("ur5e_2",new_target_position)
#         executor.put_down("ur5e_2", new_target_position[0],new_target_position[1],new_target_position[2])
#         executor.plan_and_move_to_xyz_facing_down("ur5e_2",new_target_position)
#
# block_positions = [
#     [-0.7, -0.6, 0.03],
#     [-0.7, -0.7, 0.03],
#     [-0.7, -0.8, 0.03],
#     [-0.7, -0.9, 0.03]]
#
# target_location = [-0.5, -0.5, 0.15]
#
# env = SimEnv()
# executor = MotionExecutor(env)
# # block_position_dict= env._object_manager.get_all_block_positions_dict()
# # block_positions = [list(map(float, pos)) for pos in env._object_manager.get_all_block_positions()]
# # print(block_positions)
# create_cube_stack(block_positions, target_location)
#
#
#
import time
from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor

# Définition des limites de l'espace de travail
workspace_x_lims = [-1.0, -0.45]
workspace_y_lims = [-1.0, -0.45]

# Hauteur d'un Kapla
kapla_height = 0.015  # 1.5 cm

# Définition des nouvelles positions pour les trois piles
pile_positions = [
    [-0.3, -0.5, 0.03],  # Pile 1
    [-0.5, -0.5, 0.03],  # Pile 2
    [-0.7, -0.5, 0.03]   # Pile 3
]

def disassemble_kapla_tower(kapla_positions, pile_positions):
    """
    Désassemble une tour de Kaplas en trois piles distinctes.
    """
    # On part du sommet de la tour et on descend
    for i, position in enumerate(reversed(kapla_positions)):  # On prend du haut vers le bas
        # Choisir la pile en alternant
        pile_index = i % 3
        target_pile = pile_positions[pile_index]

        # Calcul de la nouvelle hauteur dans la pile choisie
        new_target_position = [
            target_pile[0],
            target_pile[1],
            target_pile[2] + (i // 3) * kapla_height  # On empile dans chaque pile
        ]

        # Position initiale pour la prise
        new_init_position = [
            position[0],
            position[1],
            position[2] + 0.06
        ]

        # Déplacement du bras pour saisir un Kapla
        executor.plan_and_move_to_xyz_facing_down("ur5e_2", new_init_position)
        executor.pick_up("ur5e_2", position[0], position[1], position[2] + 0.06)

        # Déplacement et positionnement dans la pile choisie
        executor.plan_and_move_to_xyz_facing_down("ur5e_2", new_target_position)
        executor.put_down("ur5e_2", new_target_position[0], new_target_position[1], new_target_position[2])

# Positions des 12 Kaplas empilés
kapla_positions = [
    [-0.8, -0.6, 0.03], [-0.8, -0.4, 0.03],
    [-0.7, -0.5, 0.05], [-0.9, -0.5, 0.05],
    [-0.8, -0.6, 0.07], [-0.8, -0.4, 0.07],
    [-0.7, -0.5, 0.09], [-0.9, -0.5, 0.09],
    [-0.8, -0.6, 0.11], [-0.8, -0.4, 0.11],
    [-0.7, -0.5, 0.13], [-0.9, -0.5, 0.13]
]

# Initialisation de l'environnement et du bras robotique
env = SimEnv()
executor = MotionExecutor(env)

# Désassemblage de la tour et formation de 3 piles
disassemble_kapla_tower(kapla_positions, pile_positions)
