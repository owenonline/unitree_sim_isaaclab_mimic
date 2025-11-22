# Copyright (c) 2025
# License: Apache-2.0

#adapted from Owen's
from isaaclab.envs.mimic_env_cfg import MimicEnvCfg, SubTaskConfig
from isaaclab.utils import configclass
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs import ManagerBasedRLEnv

import torch

# Import your IsaacLab task config
from tasks.g1_tasks.press_knob_g1_29dof_dex3.press_knob_g1_29dof_dex3_joint_env_cfg import (
    PressKnobG129DEX3JointEnvCfg,
)

def check_knob_pressed_success(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    initial_height: float = 0.84,
    target_press_depth: float = 0.015,
    press_tolerance: float = 0.005,
    position_tolerance: float = 0.05,
    initial_x: float = -0.35,
    initial_y: float = 0.40,
) -> torch.Tensor:
    """Check if knob has been successfully pressed to target depth."""
    object: RigidObject = env.scene[object_cfg.name]
    
    # Get current position
    knob_x = object.data.root_pos_w[:, 0]
    knob_y = object.data.root_pos_w[:, 1]
    knob_height = object.data.root_pos_w[:, 2]
    
    # Calculate press depth
    press_depth = initial_height - knob_height
    
    # Check if in valid XY position
    x_ok = torch.abs(knob_x - initial_x) <= position_tolerance
    y_ok = torch.abs(knob_y - initial_y) <= position_tolerance
    
    # Check if pressed to target depth
    press_ok = torch.abs(press_depth - target_press_depth) <= press_tolerance
    
    # Success = pressed correctly AND still in position
    success = x_ok & y_ok & press_ok
    
    return success.to(dtype=torch.bool, device=env.device)

@configclass
class MimicTerminationsCfg:
    # check if the object is out of the working range
    success = DoneTerm(func=check_knob_pressed_success)# func that checks if knob is pressed.


@configclass
class PressKnobG129DEX3JointMimicEnvCfg(PressKnobG129DEX3JointEnvCfg, MimicEnvCfg):
    """
    Mimic wrapper config for the G1 press knob task (29DoF + Dex3, joint control).

    Inherits your task config, then augments it with Mimic data-gen settings and subtask layout.
    """

    terminations: MimicTerminationsCfg = MimicTerminationsCfg()

    def __post_init__(self):
        # Call parents first
        super().__post_init__()

        # ---- Datagen knobs (tuned for joint-space control) ----
        self.datagen_config.name = "g1_press_knob_dex3_joint_D0"
        self.datagen_config.generation_guarantee = True
        self.datagen_config.generation_keep_failed = False
        self.datagen_config.generation_num_trials = 1000

        # Joint-space controller: do not require per-arm source selection
        self.datagen_config.generation_select_src_per_subtask = False
        self.datagen_config.generation_select_src_per_arm = False

        # We are *not* producing targets in a relative eef space
        self.datagen_config.generation_relative = False
        # Joint-space: don't attempt to synthesize joint targets from "joint_pos" values here
        self.datagen_config.generation_joint_pos = False

        # For joint-space, "interpolate_from_last_target_pose" isn’t critical,
        # but leaving it True helps smooth subtask stitching when available.
        self.datagen_config.generation_transform_first_robot_pose = False
        self.datagen_config.generation_interpolate_from_last_target_pose = True

        self.datagen_config.max_num_failures = 25
        self.datagen_config.num_demo_to_render = 10
        self.datagen_config.num_fail_demo_to_render = 25
        self.datagen_config.seed = 1

        subtask_configs = []
        subtask_configs.append(
            SubTaskConfig( # Entire task: approach + press - follow the tele op demo.
                # Each subtask involves manipulation with respect to a single object frame.
                object_ref="object", # the knob
                # No termination signal - treat entire demo as one continuous task
                subtask_term_signal=None,
                first_subtask_start_offset_range=(0, 0),
                # Randomization range for starting index of the first subtask
                subtask_term_offset_range=(0, 0),
                # Selection strategy for the source subtask segment during data generation
                selection_strategy="nearest_neighbor_object",
                # Optional parameters for the selection strategy function
                selection_strategy_kwargs={"nn_k": 3},
                # Amount of action noise to apply during this subtask
                action_noise=0.003,
                # Number of interpolation steps to bridge to this subtask segment
                num_interpolation_steps=0,
                # Additional fixed steps for the robot to reach the necessary pose
                num_fixed_steps=0,
                # If True, apply action noise during the interpolation phase and execution
                apply_noise_during_interpolation=False,
            )
        )
        self.subtask_configs["right"] = subtask_configs
    

        # Or Approach + press separated for more control over each phase.
        # TO BE FINALIZED LATER, as it requires annotation and mimic schema. 
    
        subtask_configs = []
        subtask_configs.append( #letting left handle remain still.
            SubTaskConfig(
                object_ref="object",
                subtask_term_signal=None,  # Runs entire demo
                selection_strategy="random",  # Left arm doesn't need NN matching
                action_noise=0.001,  # Less noise for stabilizing arm
                num_interpolation_steps=0,
            )
        )
        # comment out if not using left hand at all.
        self.subtask_configs["left"] = subtask_configs

    
    