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

        # ---- Subtasks (placeholders for your future annotations) ----
        # You described 4 subtasks:
        # 1) left eef grabs and picks up cylinder
        # 2) right eef grabs other end while left maintains grip
        # 3) left eef lets go (right holds)
        # 4) right hand drops cylinder into right bin
        #
        # We split them per arm so Mimic can mix/match demonstrations cleanly.
        left_subtasks = []
        right_subtasks = []

        # (1) Left grabs & picks up
        left_subtasks.append(
            SubTaskConfig(
                object_ref="object",
                subtask_term_signal="left_grab_pick",  # <-- you’ll annotate this in datagen later
                first_subtask_start_offset_range=(0, 0),
                subtask_term_offset_range=(0, 0),
                selection_strategy="nearest_neighbor_object",
                selection_strategy_kwargs={"nn_k": 3},
                action_noise=0.003,
                num_interpolation_steps=0,
                num_fixed_steps=0,
                apply_noise_during_interpolation=False,
            )
        )

        # (2) Right grabs other end (left still holding)
        right_subtasks.append(
            SubTaskConfig(
                object_ref="object",
                subtask_term_signal="right_grab_hold",  # <-- annotate later
                subtask_term_offset_range=(0, 0),
                selection_strategy="nearest_neighbor_object",
                selection_strategy_kwargs={"nn_k": 3},
                action_noise=0.003,
                num_interpolation_steps=3,   # small bridge to help right hand approach
                num_fixed_steps=0,
                apply_noise_during_interpolation=False,
            )
        )

        # (3) Left releases (right holds)
        left_subtasks.append(
            SubTaskConfig(
                object_ref="object",
                subtask_term_signal="left_release",  # <-- annotate later
                subtask_term_offset_range=(0, 0),
                selection_strategy="nearest_neighbor_object",
                selection_strategy_kwargs={"nn_k": 3},
                action_noise=0.003,
                num_interpolation_steps=0,
                num_fixed_steps=0,
                apply_noise_during_interpolation=False,
            )
        )

        # (4) Right drops into right bin
        right_subtasks.append(
            SubTaskConfig(
                object_ref="object",
                subtask_term_signal="right_drop_bin",  # <-- annotate later
                subtask_term_offset_range=(0, 0),
                selection_strategy="nearest_neighbor_object",
                selection_strategy_kwargs={"nn_k": 3},
                action_noise=0.003,
                num_interpolation_steps=3,  # small bridge into the bin
                num_fixed_steps=0,
                apply_noise_during_interpolation=False,
            )
        )

        # Assign per end-effector
        self.subtask_configs["left"] = left_subtasks
        self.subtask_configs["right"] = right_subtasks
