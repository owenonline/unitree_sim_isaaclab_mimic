# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0      
from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def reset_object_estimate(
    env: ManagerBasedRLEnv,
    min_x: float = -0.42,
    max_x: float = 1.0,
    min_y: float = 0.2,
    max_y: float = 0.7,
    min_height: float = 0.5,
    max_buttons: int = 4,
) -> torch.Tensor:
    """Check if ANY active button is out of bounds.
    
    Returns True (done) if any button that's in the active zone (z > 0.5) 
    has moved outside the valid workspace.
    """
    # Initialize done tensor (False = continue, True = reset)
    done = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    
    # Check all possible buttons
    for i in range(max_buttons):
        obj_name = f"object_{i}"
        
        # Skip if button doesn't exist
        if obj_name not in env.scene:
            continue
            
        try:
            object: RigidObject = env.scene[obj_name]
            
            # Get button position
            wheel_x = object.data.root_pos_w[:, 0]
            wheel_y = object.data.root_pos_w[:, 1]
            wheel_height = object.data.root_pos_w[:, 2]
            
            # Only check buttons in the active zone (not the ones at z=-10)
            active_buttons = wheel_height > 0.5
            
            # Check if active buttons are out of bounds
            out_of_bounds_x = (wheel_x < min_x) | (wheel_x > max_x)
            out_of_bounds_y = (wheel_y < min_y) | (wheel_y > max_y)
            out_of_bounds_z = wheel_height < min_height
            
            # Mark as done if any active button is out of bounds
            button_out = active_buttons & (out_of_bounds_x | out_of_bounds_y | out_of_bounds_z)
            done = done | button_out
            
        except Exception as e:
            # Skip buttons that cause errors
            pass
    
    return done

