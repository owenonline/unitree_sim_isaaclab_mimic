# Copyright (c) 2025
# License: Apache-2.0

from isaaclab.envs.mimic_env_cfg import MimicEnvCfg, SubTaskConfig
from isaaclab.utils import configclass

# Import your IsaacLab task config
from tasks.g1_tasks.pick_place_cylinder_g1_29dof_dex3.pickplace_cylinder_g1_29dof_dex3_joint_env_cfg import (
    PickPlaceG129DEX3JointEnvCfg,
)


@configclass
class PickPlaceG129DEX3JointMimicEnvCfg(PickPlaceG129DEX3JointEnvCfg, MimicEnvCfg):
    """
    Mimic wrapper config for the G1 pick-place cylinder task (29DoF + Dex3, joint control).

    Inherits your task config, then augments it with Mimic data-gen settings and subtask layout.
    """

    def __post_init__(self):
        # Call parents first
        super().__post_init__()

        # ---- Datagen knobs (tuned for joint-space control) ----
        self.datagen_config.name = "g1_pick_place_cylinder_dex3_joint_D0"
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
            SubTaskConfig(
                # Each subtask involves manipulation with respect to a single object frame.
                object_ref="object",
                # This key corresponds to the binary indicator in "datagen_info" that signals
                # when this subtask is finished (e.g., on a 0 to 1 edge).
                subtask_term_signal="idle_right",
                first_subtask_start_offset_range=(0, 0),
                # Randomization range for starting index of the first subtask
                subtask_term_offset_range=(0, 0),
                # Selection strategy for the source subtask segment during data generation
                # selection_strategy="nearest_neighbor_object",
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
        subtask_configs.append(
            SubTaskConfig(
                # Each subtask involves manipulation with respect to a single object frame.
                object_ref="object",
                # Corresponding key for the binary indicator in "datagen_info" for completion
                subtask_term_signal=None,
                # Time offsets for data generation when splitting a trajectory
                subtask_term_offset_range=(0, 0),
                # Selection strategy for source subtask segment
                selection_strategy="nearest_neighbor_object",
                # Optional parameters for the selection strategy function
                selection_strategy_kwargs={"nn_k": 3},
                # Amount of action noise to apply during this subtask
                action_noise=0.003,
                # Number of interpolation steps to bridge to this subtask segment
                num_interpolation_steps=3,
                # Additional fixed steps for the robot to reach the necessary pose
                num_fixed_steps=0,
                # If True, apply action noise during the interpolation phase and execution
                apply_noise_during_interpolation=False,
            )
        )
        self.subtask_configs["right"] = subtask_configs

        subtask_configs = []
        subtask_configs.append(
            SubTaskConfig(
                # Each subtask involves manipulation with respect to a single object frame.
                object_ref="object",
                # Corresponding key for the binary indicator in "datagen_info" for completion
                subtask_term_signal=None,
                # Time offsets for data generation when splitting a trajectory
                subtask_term_offset_range=(0, 0),
                # Selection strategy for source subtask segment
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
        self.subtask_configs["left"] = subtask_configs
