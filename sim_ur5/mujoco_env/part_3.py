# from sim_ur5.mujoco_env.sim_env import SimEnv
# from sim_ur5.motion_planning.motion_executor import MotionExecutor
#
# # Définition des limites de l'espace de travail
# workspace_x_lims = [-1.0, -0.45]
# workspace_y_lims = [-1.0, -0.45]
#
# # Hauteur d'un bloc
# block_height = 0.015  # 1.5 cm
#
# # Définition des nouvelles positions pour les trois piles
# pile_positions = [
#     [-0.3, -0.5, 0.03],  # Pile 1
#     [-0.5, -0.5, 0.03],  # Pile 2
#     [-0.7, -0.5, 0.03]   # Pile 3
# ]
#
# # Définition explicite des positions des blocs dans la scène
# block_positions = [
#     [-0.8, -0.6, 0.03], [-0.8, -0.4, 0.03],  # Niveau 1
#     [-0.7, -0.5, 0.05], [-0.9, -0.5, 0.05],  # Niveau 2
#     [-0.8, -0.6, 0.07], [-0.8, -0.4, 0.07],  # Niveau 3
#     [-0.7, -0.5, 0.09], [-0.9, -0.5, 0.09],  # Niveau 4
#     [-0.8, -0.6, 0.11], [-0.8, -0.4, 0.11],  # Niveau 5
#     [-0.7, -0.5, 0.13], [-0.9, -0.5, 0.13]   # Niveau 6
# ]
#
# def disassemble_block_tower(pile_positions, block_positions):
#     """
#     Désassemble une tour de blocs en trois piles distinctes.
#     """
#
#     # Trier les blocs par hauteur (ordre décroissant) pour les retirer du haut vers le bas
#     block_positions.sort(key=lambda x: x[2], reverse=True)
#
#     for i, position in enumerate(block_positions):  # On prend du haut vers le bas
#         # Choisir la pile en alternant
#         pile_index = i % 3
#         target_pile = pile_positions[pile_index]
#
#         # Calcul de la nouvelle hauteur dans la pile choisie
#         new_target_position = [
#             target_pile[0],
#             target_pile[1],
#             target_pile[2] + (i // 3) * block_height
#         ]
#
#         # Position initiale pour arriver au-dessus du bloc
#         approach_position = [
#             position[0],
#             position[1],
#             position[2] + 0.03  # Légèrement au-dessus du bloc
#         ]
#
#         # Position finale pour attraper le bloc
#         grasp_position = [
#             position[0],
#             position[1],
#             position[2] + 0.005  # Juste au niveau du bloc
#         ]
#
#         # Déplacement du bras pour approcher du bloc
#         executor.plan_and_move_to_xyz_facing_down("ur5e_2", approach_position)
#
#         # Descendre pour attraper le bloc
#         executor.plan_and_move_to_xyz_facing_down("ur5e_2", grasp_position)
#         executor.pick_up("ur5e_2", grasp_position[0], grasp_position[1], grasp_position[2] + 0.008)
#
#         # Reculer légèrement après avoir attrapé le bloc
#         executor.plan_and_move_to_xyz_facing_down("ur5e_2", approach_position)
#
#         # Déplacement et positionnement dans la pile choisie
#         executor.plan_and_move_to_xyz_facing_down("ur5e_2", new_target_position)
#         executor.put_down("ur5e_2", new_target_position[0], new_target_position[1], new_target_position[2])
#
#
# # Initialisation de l'environnement et du bras robotique
# env = SimEnv()
# executor = MotionExecutor(env)
# env.reset(randomize=False, block_positions=block_positions)
#
# # Désassemblage de la tour et formation de 3 piles
# disassemble_block_tower(pile_positions, block_positions)

from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor
import logging
import numpy as np


def initialize_environment(block_positions):
    env = SimEnv()
    executor = MotionExecutor(env)
    env.reset(randomize=False, block_positions=block_positions)
    return env, executor

def detect_and_sort_blocks(block_positions):
    return sorted(block_positions, key=lambda x: x[2], reverse=True)


def pick_up_part3(executor, robot_name, x, y, start_height=0.25, descent_step=0.001, retract_height=0.002, speed=0.1):
    """
    Progressive descent until contact detection based on the robot's speed,
    then retrieve the actual end-effector position from the simulation.
    """
    logging.info(f"🔹 {robot_name} attempting to pick up at ({x}, {y}) from start height {start_height}")
    # Move on top of the target
    if not executor.plan_and_move_to_xyz_facing_down(robot_name, [x, y, start_height], speed=2.0, acceleration=2.0):
        logging.error(f"❌ {robot_name} unable to reach start position. Aborting pick-up.")
        return False

    above_pickup_config = executor.env.robots_joint_pos[robot_name]
    executor.deactivate_grasp()

    # Descend slowly until contact is detected
    logging.info(f"🔻 {robot_name} descending slowly until contact is detected...")
    current_z = start_height
    previous_velocity = np.inf
    detected_contact = False

    # loop for progressive descent and contact detection
    while current_z > 0.02:  # Stop before reaching the table
        current_z -= descent_step
        executor.moveL(robot_name, (x, y, current_z), speed=speed, tolerance=0.001)

        # retrieve the current velocity of the robot
        current_velocity = np.linalg.norm(executor.env.robots_joint_velocities[robot_name])
        if current_velocity < previous_velocity * 0.9: # Contact detection: drop in velocity
            logging.info(f"⚠️ {robot_name} detected contact at Z = {current_z} (velocity drop)")
            detected_contact = True
            break  # stop if contact is detected

        previous_velocity = current_velocity

    if not detected_contact:
        logging.warning(f"⚠️ {robot_name} did not detect contact. Retrying...")
        return False

    # Retrieve the exact position of the end effector in the simulation
    ee_position = executor.env.get_ee_pos()
    grasp_z = ee_position[2] + retract_height
    logging.info(f"📏 Adjusted grasp height based on simulation: {grasp_z}")

    # Fine adjustment before grasping
    executor.moveL(robot_name, (x, y, grasp_z), speed=1.0, tolerance=0.003)
    logging.info(f"🔄 Activating gripper for {robot_name}...")
    executor.activate_grasp()

    # # Check if the grasp is successful
    if not executor.env.is_object_grasped():
        logging.warning(f"⚠️ {robot_name} failed to grasp the object. Retrying...")
        executor.deactivate_grasp()
        return False

    logging.info(f"✅ {robot_name} successfully grasped the Kapla! Retracting now.")
    safe_retract_height = grasp_z + 0.05  # Safe height to retract
    executor.moveL(robot_name, (x, y, safe_retract_height), speed=2.0, tolerance=0.003)
    executor.moveJ(robot_name, above_pickup_config, speed=2.0, acceleration=2.0, tolerance=0.05)


def disassemble_block_tower(executor, env, block_positions, pile_positions, block_height=0.015):
    sorted_blocks = detect_and_sort_blocks(block_positions)
    for i, position in enumerate(sorted_blocks):
        pile_index = i % 4
        target_pile = pile_positions[pile_index]
        new_target_position_up = [target_pile[0], target_pile[1], 0.2 + target_pile[2] + (i // 4) * block_height]
        new_target_position = [target_pile[0], target_pile[1], target_pile[2] + (i // 3) * block_height]
        x, y = position[0], position[1]
        start_height = position[2] + 0.05  # initial estimation

        # Motion
        pick_up_part3(executor, "ur5e_2", x, y+0.01, start_height)
        executor.plan_and_move_to_xyz_facing_down("ur5e_2", new_target_position_up,speed=2, acceleration=2)
        executor.put_down("ur5e_2", new_target_position[0],new_target_position[1],new_target_position[2])
        executor.deactivate_grasp()
        executor.plan_and_move_to_xyz_facing_down("ur5e_2",  new_target_position_up, speed=2, acceleration=2)


def main():
    block_positions = [
        [-0.8, -0.7, 0.03], [-0.8, -0.5, 0.03],
        [-0.7, -0.6, 0.05], [-0.9, -0.6, 0.05],
        [-0.8, -0.7, 0.07], [-0.8, -0.5, 0.07],
        [-0.7, -0.6, 0.09], [-0.9, -0.6, 0.09],
        [-0.8, -0.7, 0.11], [-0.8, -0.5, 0.11],
        [-0.7, -0.6, 0.13], [-0.9, -0.6, 0.13]
    ]
    pile_positions = [[-0.5, -0.5, 0.02], [-0.5, -0.5, 0.02], [-0.5, -0.8, 0.02], [-0.5, -0.8, 0.02]]

    env, executor = initialize_environment(block_positions)
    disassemble_block_tower(executor, env, block_positions, pile_positions)
    logging.info("Disassembly completed successfully!")

if __name__ == "__main__":
    main()
