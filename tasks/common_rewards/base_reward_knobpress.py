# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

from __future__ import annotations
import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

def compute_reward(
    env: ManagerBasedRLEnv,
    object_name: str = "object_0",   # Argument kept for config compatibility, but effectively ignored
    hand_side: str = "left",         # "left" or "right"
    touch_tolerance: float = 0.01,   # 1cm tolerance
    max_buttons: int = 4,            # Must match max buttons in scene
) -> torch.Tensor:
    """
    Compute reward for touching the randomly selected TARGET button.
    
    This function:
    1. Finds the correct index finger (Dex3 supported).
    2. Retrieves the current target index from env.target_button_indices.
    3. Looks up the position of that specific target button.
    4. Returns 1.0 if touching, 0.0 otherwise.
    """
    
    # --- 1. Get Robot and Cache Finger Index ---
    try:
        robot = env.scene["robot"]
    except KeyError:
        return torch.zeros(env.num_envs, device=env.device)

    # Lazy Index Lookup (Runs once per session)
    cache_key = f"_cache_idx_{hand_side}_dex3_finger"
    body_names = robot.data.body_names
    
    # Candidates for G1 / Dex3 finger tip
    
    name = f"{hand_side}_hand_index_1_link"
    finger_idx = None
    finger_idx = body_names.index(name)
    print(f"[Rewards] Found finger link: '{name}' at index {finger_idx}")
    
    if finger_idx is None:
        print(f"[Rewards] WARNING: Finger link not found in {body_names}. Reward will be 0.")
        return torch.zeros(env.num_envs, device=env.device)
        
    setattr(env, cache_key, finger_idx)

    # Get Finger Position
    finger_idx = getattr(env, cache_key)
    # shape: (num_envs, 3)
    finger_pos = robot.data.body_link_pose_w[:, finger_idx, :3]


    # --- 2. Get Target Button Position ---
    
    # If targets haven't been set yet (e.g. before first reset), return 0
    if not hasattr(env, "target_button_indices"):
        return torch.zeros(env.num_envs, device=env.device)
    
    target_indices = env.target_button_indices  # Shape: (num_envs,)

    # Gather positions of all objects (object_0 to object_N)
    # We create a stack of shape: (num_envs, max_buttons, 3)
    all_obj_positions_list = []
    
    for i in range(max_buttons):
        obj_key = f"object_{i}"
        try:
            if obj_key in env.scene.keys():
                # Get real position
                pos = env.scene[obj_key].data.root_pos_w[:, :3]
                all_obj_positions_list.append(pos)
            else:
                # Object defined in logic but not in scene? Use dummy far pos
                dummy = torch.zeros_like(finger_pos) + 100.0
                all_obj_positions_list.append(dummy)
        except Exception:
            dummy = torch.zeros_like(finger_pos) + 100.0
            all_obj_positions_list.append(dummy)

    # Stack: (num_envs, max_buttons, 3)
    all_obj_positions_stack = torch.stack(all_obj_positions_list, dim=1)

    # Select the specific target position for each environment
    # We need to expand indices to match dimensions for torch.gather
    # target_indices: (num_envs,) -> (num_envs, 1, 3)
    target_indices_expanded = target_indices.view(-1, 1, 1).expand(-1, 1, 3)
    
    # Gather: result is (num_envs, 1, 3)
    target_button_pos_grouped = torch.gather(all_obj_positions_stack, 1, target_indices_expanded)
    
    # Squeeze to (num_envs, 3)
    target_button_pos = target_button_pos_grouped.squeeze(1)


    # --- 3. Calculate Distance and Reward ---
    
    distance = torch.norm(finger_pos - target_button_pos, dim=1)
    
    # Binary reward: 1.0 if within tolerance, else 0.0
    reward = (distance < touch_tolerance).float()
    
    return reward