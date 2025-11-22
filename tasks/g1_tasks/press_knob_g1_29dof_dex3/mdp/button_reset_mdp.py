# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
# Adapted for Random Target Selection

"""Custom MDP functions for button reset with collision avoidance and target selection."""

import random
import torch
from isaaclab.envs import ManagerBasedRLEnv

__all__ = [
    "reset_buttons_random",
    "reset_robot_only",
    "reset_robot_and_buttons",
]

def _get_button_rigid_object(env: "ManagerBasedRLEnv", button_idx: int):
    """Safely get button RigidObject from scene entities without using 'in env.scene'."""
    obj_name = f"object_{button_idx}"
    try:
        try:
            button = env.scene[obj_name]
        except KeyError:
            button = None
        if button is not None:
            return button
    except Exception as e:
        print(f"[Button Reset] Exception getting {obj_name}: {e}")
    return None

def reset_buttons_random(
    env: "ManagerBasedRLEnv",
    env_ids: torch.Tensor,
    min_buttons: int = 1,
    max_buttons: int = 4,
    base_pos: tuple = (-0.35, 0.40, 0.84),
    position_randomization: tuple = (0.15, 0.15, 0.0),
    min_distance: float = 0.12,
):
    """
    Resets buttons randomly and selects one ACTIVE button as the TARGET.
    Target index is stored in `env.target_button_indices`.
    """
    num_envs = len(env_ids)
    
    # 1. Initialize Target Storage if needed
    if not hasattr(env, "target_button_indices"):
        env.target_button_indices = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)

    # Color map for console logging (Visualization for Teleop)
    # Assuming object_0=Red, object_1=Green, object_2=Blue, object_3=Yellow
    color_map = {0: "RED", 1: "GREEN", 2: "BLUE", 3: "YELLOW"}

    # 2. Get button RigidObjects
    button_objects = []
    for i in range(max_buttons):
        button = _get_button_rigid_object(env, i)
        if button is None:
            print(f"[Button Reset] ERROR: object_{i} not found in scene")
            return
        button_objects.append(button)
    
    # 3. Prepare Tensors
    base_pos_tensor = torch.tensor(base_pos, device=env.device, dtype=torch.float32)
    pos_rand = torch.tensor(position_randomization, device=env.device, dtype=torch.float32)
    far_away_pos = torch.tensor([0.0, 0.0, -100.0], device=env.device, dtype=torch.float32)
    
    # Shape: (max_buttons, num_envs, 3)
    all_button_positions = torch.zeros((max_buttons, num_envs, 3), device=env.device)
    
    # 4. Logic per environment
    print(f"[Button Reset] Resetting {num_envs} environments...")
    
    for env_idx_local, env_id in enumerate(env_ids):
        # A. Determine how many and which buttons
        n_buttons = torch.randint(min_buttons, max_buttons + 1, (1,), device=env.device).item()
        all_indices = list(range(max_buttons))
        
        # Select active buttons
        selected_indices = random.sample(all_indices, n_buttons)
        selected_indices.sort()
        
        # B. SELECT TARGET (Must be one of the active buttons)
        target_idx = random.choice(selected_indices)
        env.target_button_indices[env_id] = target_idx
        
        # Log for Teleop Operator
        print(f"  >>> Env {env_id.item()} Target: {color_map.get(target_idx, f'Obj_{target_idx}')} (Active: {selected_indices})")
        
        # C. Generate Positions (Collision Free)
        positions = []
        max_attempts = 100
        
        for button_idx in selected_indices:
            placed = False
            for _ in range(max_attempts):
                # Random offset
                rand_offset = (torch.rand(3, device=env.device) * 2 - 1) * pos_rand
                new_pos = base_pos_tensor + rand_offset
                new_pos[2] = torch.clamp(new_pos[2], min=0.82, max=0.86) # Keep on table Z
                
                # Check collision (2D)
                collision = False
                for existing_pos in positions:
                    if torch.norm(new_pos[:2] - existing_pos[:2]) < min_distance:
                        collision = True
                        break
                
                if not collision:
                    positions.append(new_pos)
                    placed = True
                    all_button_positions[button_idx, env_idx_local] = new_pos
                    break
            
            if not placed:
                # Fallback
                fallback = base_pos_tensor.clone()
                fallback[0] += button_idx * 0.15
                all_button_positions[button_idx, env_idx_local] = fallback
        
        # D. Move inactive buttons far away
        for button_idx in range(max_buttons):
            if button_idx not in selected_indices:
                all_button_positions[button_idx, env_idx_local] = far_away_pos

    # 5. Write Physics States
    identity_quat = torch.tensor([1.0, 0.0, 0.0, 0.0], device=env.device)
    zero_vel = torch.zeros(6, device=env.device)
    
    for button_idx, button in enumerate(button_objects):
        positions = all_button_positions[button_idx] # (num_envs, 3)
        orientations = identity_quat.unsqueeze(0).repeat(num_envs, 1)
        poses = torch.cat([positions, orientations], dim=-1)
        velocities = zero_vel.unsqueeze(0).repeat(num_envs, 1)
        full_root_state = torch.cat([poses, velocities], dim=-1)
        
        button.write_root_state_to_sim(full_root_state, env_ids=env_ids)

    print(f"[Button Reset] ✓ Complete. Targets stored in env.target_button_indices.")

def reset_robot_only(env: "ManagerBasedRLEnv", env_ids: torch.Tensor):
    """Reset only the robot to default pose."""
    robot = env.scene["robot"]
    joint_pos = robot.data.default_joint_pos[env_ids].clone()
    joint_vel = robot.data.default_joint_vel[env_ids].clone()
    robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
    
    if hasattr(robot.data, 'default_root_state'):
        try:
            root_states = robot.data.default_root_state[env_ids].clone()
            robot.write_root_pose_to_sim(root_states[:, :7], env_ids=env_ids)
            robot.write_root_velocity_to_sim(root_states[:, 7:13], env_ids=env_ids)
        except Exception:
            pass

def reset_robot_and_buttons(
    env: "ManagerBasedRLEnv",
    env_ids: torch.Tensor,
    min_buttons: int = 1,
    max_buttons: int = 4,
    base_pos: tuple = (-0.35, 0.40, 0.84),
    position_randomization: tuple = (0.15, 0.15, 0.0),
    min_distance: float = 0.12,
):
    """Full reset."""
    reset_robot_only(env, env_ids)
    reset_buttons_random(env, env_ids, min_buttons, max_buttons, base_pos, position_randomization, min_distance)