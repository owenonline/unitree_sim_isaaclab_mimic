# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
from action_provider.action_base import ActionProvider
from typing import Optional
import torch
import numpy as np

class FileActionProvider(ActionProvider):
    """Action provider based on file, this file is for testing, not for actual use"""
    
    def __init__(self, file_path: str, loop: bool = True):
        super().__init__("FileActionProvider")
        self.file_path = file_path
        self.loop = loop
        self.action_data = None
        self.current_step = 0
        self._load_data()
    
    def _load_data(self):
        """Load file data"""
        try:
            import re
            with open(self.file_path, 'r') as f:
                lines = f.readlines()
            
            data = []
            for line in lines:
                numbers = re.findall(r"[-+]?\d*\.\d+|\d+", line)
                if numbers:
                    float_list = list(map(float, numbers))
                    data.append(float_list)
            
            self.action_data = np.array(data)
            print(f"[{self.name}] Loaded {len(self.action_data)} steps of action data")
        except Exception as e:
            print(f"[{self.name}] Load file failed: {e}")
            self.action_data = None
    
    def get_action(self, env) -> Optional[torch.Tensor]:
        """Get action from file"""
        if self.action_data is None:
            return None
        
        try:
            all_joint_names = env.scene["robot"].data.joint_names
            full_action = torch.zeros(len(all_joint_names), device=env.device)
            
            # Get current step data
            if self.current_step >= len(self.action_data):
                if self.loop:
                    self.current_step = 0
                else:
                    return None
            
            current_action = self.action_data[self.current_step]
            self.current_step += 1
            
            # Map to robot joints
            arm_joint_mapping = {
                "left_shoulder_pitch_joint": 0,
                "left_shoulder_roll_joint": 1,
                "left_shoulder_yaw_joint": 2,
                "left_elbow_joint": 3,
                "left_wrist_roll_joint": 4,
                "left_wrist_pitch_joint": 5,
                "left_wrist_yaw_joint": 6,
                "right_shoulder_pitch_joint": 7,
                "right_shoulder_roll_joint": 8,
                "right_shoulder_yaw_joint": 9,
                "right_elbow_joint": 10,
                "right_wrist_roll_joint": 11,
                "right_wrist_pitch_joint": 12,
                "right_wrist_yaw_joint": 13
            }
            
            for i, name in enumerate(all_joint_names):
                if name in arm_joint_mapping and arm_joint_mapping[name] < len(current_action):
                    full_action[i] = current_action[arm_joint_mapping[name]]
            
            return full_action.unsqueeze(0)
            
        except Exception as e:
            print(f"[{self.name}] Get file action failed: {e}")
            return None
