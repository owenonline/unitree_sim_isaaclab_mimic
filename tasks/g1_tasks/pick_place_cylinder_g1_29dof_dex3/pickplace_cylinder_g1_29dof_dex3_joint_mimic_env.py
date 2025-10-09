# Copyright (c) 2025
# License: Apache-2.0

from __future__ import annotations

from collections.abc import Sequence
import torch

import isaaclab.utils.math as PoseUtils
from isaaclab.envs import ManagerBasedRLMimicEnv


class PickPlaceG129DEX3JointMimicEnv(ManagerBasedRLMimicEnv):
    """
    Mimic wrapper for the G1 29-DoF (Dex3) joint-control pick-place cylinder task.

    Notes:
    - The underlying controller takes *joint positions* for all joints.
    - We therefore do *not* compute IK inside this wrapper.
    - get_robot_eef_pose() reads EEF link pose directly from the sim.
    - target_eef_pose_to_action() preserves current joint positions and only applies gripper commands.
    - action_to_target_eef_pose() is not supported for joint-space actions (explicit error).
    """

    # Reasonable link-name candidates for this model; we’ll auto-detect the first that exists.
    LEFT_EEF_LINK_CANDIDATES = [
        "left_hand_base_link",
        "left_hand_camera_base_link",
        "left_wrist_link",
        "left_hand_link",
    ]
    RIGHT_EEF_LINK_CANDIDATES = [
        "right_hand_base_link",
        "right_hand_camera_base_link",
        "right_wrist_link",
        "right_hand_link",
    ]

    # Dex3 hand joint indices (matching your observations util)
    # Order is [L(7), R(7)] after we slice with this set and split.
    _DEX3_HAND_JOINT_INDICES = [31, 37, 41, 30, 36, 29, 35, 34, 40, 42, 33, 39, 32, 38]

    def _resolve_body_index(self, link_name_candidates: list[str]) -> int:
        body_names = self.scene["robot"].data.body_names
        for name in link_name_candidates:
            if name in body_names:
                return body_names.index(name)
        raise RuntimeError(
            f"None of the candidate EEF link names were found in robot.body_names. "
            f"Tried: {link_name_candidates}. Available: {body_names}"
        )

    def _eef_link_index(self, eef_name: str) -> int:
        if eef_name.lower().startswith("left"):
            return self._resolve_body_index(self.LEFT_EEF_LINK_CANDIDATES)
        elif eef_name.lower().startswith("right"):
            return self._resolve_body_index(self.RIGHT_EEF_LINK_CANDIDATES)
        else:
            raise ValueError(f"eef_name must start with 'left' or 'right'; got '{eef_name}'")

    # ------------------------------
    # End-effector pose (world frame)
    # ------------------------------
    def get_robot_eef_pose(self, eef_name: str, env_ids: Sequence[int] | None = None) -> torch.Tensor:
        """
        Get current robot EEF pose (4x4) from sim for each env_id.

        Args:
            eef_name: "left" or "right" (prefix, e.g., "left", "right")
            env_ids: iterable of env indices; if None, uses all envs

        Returns:
            Tensor of shape (len(env_ids), 4, 4)
        """
        if env_ids is None:
            env_ids = slice(None)

        idx = self._eef_link_index(eef_name)
        body_pos_w = self.scene["robot"].data.body_pos_w  # (N, n_bodies, 3)
        body_quat_w = self.scene["robot"].data.body_quat_w  # (N, n_bodies, 4)

        pos = body_pos_w[env_ids, idx] - self.scene.env_origins[env_ids]
        quat = body_quat_w[env_ids, idx]
        rot = PoseUtils.matrix_from_quat(quat)
        return PoseUtils.make_pose(pos, rot)

    # ---------------------------------------------------------
    # Map a target EEF pose + gripper commands -> env action
    # ---------------------------------------------------------
    def target_eef_pose_to_action(
        self,
        target_eef_pose_dict: dict,           # {"left": (4x4), "right": (4x4)} — ignored in joint-space mapping
        gripper_action_dict: dict,            # {"left": (7,), "right": (7,)} Dex3 commands
        action_noise_dict: dict | None = None,
        env_id: int = 0,                      # kept for API consistency
    ) -> torch.Tensor:
        """
        Since this env uses *joint position* control, we do not compute IK here.
        We simply:
          - take the robot's current joint positions
          - overwrite the Dex3 hand joints with the provided gripper targets (left/right, 7 each)
          - optionally add *small* noise to those gripper targets

        Returns:
            Tensor (action_dim,) suitable for env.step(action)
        """
        # Current joint positions for that env
        q = self.scene["robot"].data.joint_pos[env_id].clone()

        # Prepare gripper slices
        device = q.device
        idx_t = torch.tensor(self._DEX3_HAND_JOINT_INDICES, dtype=torch.long, device=device)

        # Build [L(7), R(7)] vector from dict
        left_cmd = gripper_action_dict["left"]
        right_cmd = gripper_action_dict["right"]

        # Ensure shapes are (7,)
        left_cmd = left_cmd.reshape(-1)
        right_cmd = right_cmd.reshape(-1)

        if action_noise_dict is not None:
            # If provided, we only apply noise to the gripper targets (not to whole-body joints)
            if "left" in action_noise_dict and action_noise_dict["left"] is not None:
                left_cmd = left_cmd + action_noise_dict["left"] * torch.randn_like(left_cmd)
            if "right" in action_noise_dict and action_noise_dict["right"] is not None:
                right_cmd = right_cmd + action_noise_dict["right"] * torch.randn_like(right_cmd)

        # Concatenate [left, right] -> (14,)
        hand_target = torch.cat([left_cmd, right_cmd], dim=0).to(device=device, dtype=q.dtype)

        # Overwrite only the hand joints in the action vector
        q.index_copy_(0, idx_t, hand_target)

        return q

    # ---------------------------------------------------------
    # Infer a target EEF pose from a *joint* action (not possible)
    # ---------------------------------------------------------
    def action_to_target_eef_pose(self, action: torch.Tensor) -> dict[str, torch.Tensor]:
        """
        Not supported for joint-space actions. We can’t derive a unique 6-DoF target EEF pose
        from a full joint vector without simulating forward kinematics under the env’s constraints
        and controller. If you need this, compute FK externally (e.g., with your kinematics model)
        and call get_robot_eef_pose() while stepping through the trajectory.
        """
        raise NotImplementedError(
            "action_to_target_eef_pose is undefined for joint-space actions. "
            "Use get_robot_eef_pose() to read the EEF pose from sim while executing actions."
        )

    # ---------------------------------------------------------
    # Extract gripper actuation (Dex3: 7 + 7) from a batch of actions
    # ---------------------------------------------------------
    def actions_to_gripper_actions(self, actions: torch.Tensor) -> dict[str, torch.Tensor]:
        """
        Extract Dex3 gripper joints from action sequences.

        Args:
            actions: (num_envs, T, action_dim) or (num_envs, action_dim)

        Returns:
            {
              "left":  (num_envs, T, 7) or (num_envs, 7),
              "right": (num_envs, T, 7) or (num_envs, 7),
            }
        """
        idx = torch.tensor(self._DEX3_HAND_JOINT_INDICES, dtype=torch.long, device=actions.device)

        # Gather along the last dimension
        hand = torch.index_select(actions, dim=-1, index=idx)

        # Split into left/right (7 each)
        left = hand[..., :7]
        right = hand[..., 7:]

        return {"left": left, "right": right}
