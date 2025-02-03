from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor

# Définition des limites de l'espace de travail
workspace_x_lims = [-1.0, -0.45]
workspace_y_lims = [-1.0, -0.45]

# Hauteur d'un bloc
block_height = 0.015  # 1.5 cm

# Définition des nouvelles positions pour les trois piles
pile_positions = [
    [-0.3, -0.5, 0.03],  # Pile 1
    [-0.5, -0.5, 0.03],  # Pile 2
    [-0.7, -0.5, 0.03]   # Pile 3
]

# Définition explicite des positions des blocs dans la scène
block_positions = [
    [-0.8, -0.6, 0.03], [-0.8, -0.4, 0.03],  # Niveau 1
    [-0.7, -0.5, 0.05], [-0.9, -0.5, 0.05],  # Niveau 2
    [-0.8, -0.6, 0.07], [-0.8, -0.4, 0.07],  # Niveau 3
    [-0.7, -0.5, 0.09], [-0.9, -0.5, 0.09],  # Niveau 4
    [-0.8, -0.6, 0.11], [-0.8, -0.4, 0.11],  # Niveau 5
    [-0.7, -0.5, 0.13], [-0.9, -0.5, 0.13]   # Niveau 6
]

def disassemble_block_tower(pile_positions, block_positions):
    """
    Désassemble une tour de blocs en trois piles distinctes.
    """

    # Trier les blocs par hauteur (ordre décroissant) pour les retirer du haut vers le bas
    block_positions.sort(key=lambda x: x[2], reverse=True)

    for i, position in enumerate(block_positions):  # On prend du haut vers le bas
        # Choisir la pile en alternant
        pile_index = i % 3
        target_pile = pile_positions[pile_index]

        # Calcul de la nouvelle hauteur dans la pile choisie
        new_target_position = [
            target_pile[0],
            target_pile[1],
            target_pile[2] + (i // 3) * block_height
        ]

        # Position initiale pour arriver au-dessus du bloc
        approach_position = [
            position[0],
            position[1],
            position[2] + 0.03  # Légèrement au-dessus du bloc
        ]

        # Position finale pour attraper le bloc
        grasp_position = [
            position[0],
            position[1],
            position[2] + 0.005  # Juste au niveau du bloc
        ]

        # Déplacement du bras pour approcher du bloc
        executor.plan_and_move_to_xyz_facing_down("ur5e_2", approach_position)

        # Descendre pour attraper le bloc
        executor.plan_and_move_to_xyz_facing_down("ur5e_2", grasp_position)
        executor.pick_up("ur5e_2", grasp_position[0], grasp_position[1], grasp_position[2] + 0.008)

        # Reculer légèrement après avoir attrapé le bloc
        executor.plan_and_move_to_xyz_facing_down("ur5e_2", approach_position)

        # Déplacement et positionnement dans la pile choisie
        executor.plan_and_move_to_xyz_facing_down("ur5e_2", new_target_position)
        executor.put_down("ur5e_2", new_target_position[0], new_target_position[1], new_target_position[2])


# Initialisation de l'environnement et du bras robotique
env = SimEnv()
executor = MotionExecutor(env)
env.reset(randomize=False, block_positions=block_positions)

# Désassemblage de la tour et formation de 3 piles
disassemble_block_tower(pile_positions, block_positions)
