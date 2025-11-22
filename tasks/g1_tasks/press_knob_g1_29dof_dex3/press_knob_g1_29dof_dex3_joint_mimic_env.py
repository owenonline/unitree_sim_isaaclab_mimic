# Copyright (c) 2025
# License: Apache-2.0

#adapted from Owen's
from __future__ import annotations

from collections.abc import Sequence
import torch
import numpy as np

import isaaclab.utils.math as PoseUtils
from isaaclab.envs import ManagerBasedRLMimicEnv
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from isaacsim.core.prims import SingleArticulation

# This is a bit tougher than the example because they include eef pose in their observation, but our actions are just absolute joint angles
# https://github.com/isaac-sim/IsaacLab/blob/3aafdf075d630907065d654450c51914e0ffe0a0/source/isaaclab_mimic/isaaclab_mimic/envs/pinocchio_envs/pickplace_gr1t2_mimic_env.py

# note: we only support one env for this for now. This may make isaac mimic challenging.

LEFT_HAND_DEX3_ORDER = [
    "left_hand_thumb_0_joint","left_hand_thumb_1_joint","left_hand_thumb_2_joint",
    "left_hand_middle_0_joint","left_hand_middle_1_joint",
    "left_hand_index_0_joint","left_hand_index_1_joint",
]
RIGHT_HAND_DEX3_ORDER = [
    "right_hand_thumb_0_joint","right_hand_thumb_1_joint","right_hand_thumb_2_joint",
    "right_hand_middle_0_joint","right_hand_middle_1_joint",
    "right_hand_index_0_joint","right_hand_index_1_joint",
]
JOINT_TO_IDX = {'left_hip_pitch_joint': 0, 'right_hip_pitch_joint': 1, 'waist_yaw_joint': 2, 'left_hip_roll_joint': 3, 'right_hip_roll_joint': 4, 'waist_roll_joint': 5, 'left_hip_yaw_joint': 6, 'right_hip_yaw_joint': 7, 'waist_pitch_joint': 8, 'left_knee_joint': 9, 'right_knee_joint': 10, 'left_shoulder_pitch_joint': 11, 'right_shoulder_pitch_joint': 12, 'left_ankle_pitch_joint': 13, 'right_ankle_pitch_joint': 14, 'left_shoulder_roll_joint': 15, 'right_shoulder_roll_joint': 16, 'left_ankle_roll_joint': 17, 'right_ankle_roll_joint': 18, 'left_shoulder_yaw_joint': 19, 'right_shoulder_yaw_joint': 20, 'left_elbow_joint': 21, 'right_elbow_joint': 22, 'left_wrist_roll_joint': 23, 'right_wrist_roll_joint': 24, 'left_wrist_pitch_joint': 25, 'right_wrist_pitch_joint': 26, 'left_wrist_yaw_joint': 27, 'right_wrist_yaw_joint': 28, 'left_hand_index_0_joint': 29, 'left_hand_middle_0_joint': 30, 'left_hand_thumb_0_joint': 31, 'right_hand_index_0_joint': 32, 'right_hand_middle_0_joint': 33, 'right_hand_thumb_0_joint': 34, 'left_hand_index_1_joint': 35, 'left_hand_middle_1_joint': 36, 'left_hand_thumb_1_joint': 37, 'right_hand_index_1_joint': 38, 'right_hand_middle_1_joint': 39, 'right_hand_thumb_1_joint': 40, 'left_hand_thumb_2_joint': 41, 'right_hand_thumb_2_joint': 42}

class PressKnobG129DEX3JointMimicEnv(ManagerBasedRLMimicEnv):
    """
    Mimic wrapper for the G1 29-DoF (Dex3) joint-control knob pressing task.

    Notes:
    - The underlying controller takes *joint positions* for all joints.
    - Uses inverse kinematics (IK) to convert target EEF poses to joint positions.
    - get_robot_eef_pose() reads EEF link pose directly from the sim.
    - target_eef_pose_to_action() computes IK for both arms and applies gripper commands.
    - action_to_target_eef_pose() uses forward kinematics to convert joint positions to EEF poses.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        self.lula_solver = LulaKinematicsSolver(
            robot_description_path="/home/code/unitree_sim_isaaclab/lula_config.yaml",
            urdf_path="/home/code/unitree_sim_isaaclab/g1_29dof_with_hand_rev_1_0.urdf"
        )

        # print("DEBUGGING MIMIC")
        # print(f"robot type: {type(self.scene['robot'])}")
        found_prims = str([x for x in self.scene['robot'].stage.Traverse()])
        with open("/workspace/found_prims.txt", "w") as f:
            f.write(found_prims)
        
        self.robot = None

    def lazy_load_solvers(self):

        if self.robot is not None:
            return

        self.robot = SingleArticulation("/World/envs/env_0/Robot", name="roboto")
        self.robot.initialize()
        assert self.robot.is_valid()

        self.left_eef_solver = ArticulationKinematicsSolver(self.robot, self.lula_solver, "left_hand_palm_link")
        self.right_eef_solver = ArticulationKinematicsSolver(self.robot, self.lula_solver, "right_hand_palm_link")

        right_solver_joint_names = getattr(self.right_eef_solver._joints_view, "joint_names", None)
        left_solver_joint_names = getattr(self.left_eef_solver._joints_view, "joint_names", None)
        assert right_solver_joint_names == left_solver_joint_names
        self.solver_to_action_mapping = [JOINT_TO_IDX[name] for name in right_solver_joint_names] # indices into the 43 dim action vector for each joint computed by the kinematic solver
        self.left_mask = [right_solver_joint_names.index(name) for name in right_solver_joint_names if "left" in name]
        self.right_mask = [right_solver_joint_names.index(name) for name in right_solver_joint_names if "right" in name]
        self.left_hand_indices = [JOINT_TO_IDX[name] for name in LEFT_HAND_DEX3_ORDER]
        self.right_hand_indices = [JOINT_TO_IDX[name] for name in RIGHT_HAND_DEX3_ORDER]

    def get_robot_eef_pose(self, eef_name: str, env_ids: Sequence[int] | None = None) -> torch.Tensor:
        """
        Get current robot end effector pose. Should be the same frame as used by the robot end-effector controller.

        Args:
            eef_name: Name of the end effector.
            env_ids: Environment indices to get the pose for. If None, all envs are considered.

        Returns:
            A torch.Tensor eef pose matrix. Shape is (len(env_ids), 4, 4)
        """

        try:

            self.lazy_load_solvers()

            if eef_name == "left":
                eef_pos, rot_mat = self.left_eef_solver.compute_end_effector_pose()
            else:
                eef_pos, rot_mat = self.right_eef_solver.compute_end_effector_pose()

            eef_pos_torch = torch.from_numpy(eef_pos)
            rot_mat_torch = torch.from_numpy(rot_mat)

            return torch.unsqueeze(PoseUtils.make_pose(eef_pos_torch, rot_mat_torch), 0) # unsqueeze to get the env dimension

        except Exception as e:
            self.robot = None
            return self.get_robot_eef_pose(eef_name, env_ids)

    def target_eef_pose_to_action(
        self,
        target_eef_pose_dict: dict,
        gripper_action_dict: dict,
        action_noise_dict: dict | None = None,
        env_id: int = 0,  # Unused, but required to conform to interface
    ) -> torch.Tensor:
        """
        Takes a target pose and gripper action for the end effector controller and returns an action
        (usually a normalized delta pose action) to try and achieve that target pose.
        Noise is added to the target pose action if specified.

        Args:
            target_eef_pose_dict: Dictionary of 4x4 target eef pose for each end-effector.
            gripper_action_dict: Dictionary of gripper actions for each end-effector.
            action_noise_dict: Noise to add to the action. If None, no noise is added.
            env_id: Environment index to get the action for.

        Returns:
            An action torch.Tensor that's compatible with env.step().
        """

        try:

            self.lazy_load_solvers()

            # set the root pose of the lula solver, to ensure correct inverse kinematics
            robot_base_translation, robot_base_orientation = self.robot.get_world_pose()
            robot_base_translation = robot_base_translation.cpu().detach().numpy()
            robot_base_orientation = robot_base_orientation.cpu().detach().numpy()
            self.lula_solver.set_robot_base_pose(robot_base_translation, robot_base_orientation)

            # split the target pose into translation and orientation, and add action noise if specified by mimic
            target_left_eef_pos, left_target_rot = PoseUtils.unmake_pose(target_eef_pose_dict["left"])
            target_right_eef_pos, right_target_rot = PoseUtils.unmake_pose(target_eef_pose_dict["right"])
            target_left_eef_rot_quat = PoseUtils.quat_from_matrix(left_target_rot)
            target_right_eef_rot_quat = PoseUtils.quat_from_matrix(right_target_rot)

            if action_noise_dict is not None:
                pos_noise_left = action_noise_dict["left"] * torch.randn_like(target_left_eef_pos)
                pos_noise_right = action_noise_dict["right"] * torch.randn_like(target_right_eef_pos)
                quat_noise_left = action_noise_dict["left"] * torch.randn_like(target_left_eef_rot_quat)
                quat_noise_right = action_noise_dict["right"] * torch.randn_like(target_right_eef_rot_quat)

                target_left_eef_pos += pos_noise_left
                target_right_eef_pos += pos_noise_right
                target_left_eef_rot_quat += quat_noise_left
                target_right_eef_rot_quat += quat_noise_right

            # use inverse kinematics to compute the joint angles required to achieve the target pose for each end effector
            left_action, left_success = self.left_eef_solver.compute_inverse_kinematics(target_left_eef_pos, target_left_eef_rot_quat)
            right_action, right_success = self.right_eef_solver.compute_inverse_kinematics(target_right_eef_pos, target_right_eef_rot_quat)

            if not left_success or not right_success:
                raise ValueError("Failed to compute inverse kinematics for one or both end effectors")

            # the left solver returns zeros for all right arm joints and vice versa, 
            # so we add the two vectors to get the joint angles for all arm joints
            left_action_array = left_action.joint_positions
            right_action_array = right_action.joint_positions
            combined_arm_action = left_action_array + right_action_array

            # now we use the mapping between indices in the list of joint actions returned by the solver
            # and the full action vector required by the environment
            full = np.zeros(len(JOINT_TO_IDX), dtype=np.float32)
            full[self.solver_to_action_mapping] = combined_arm_action

            # the next step is to add the gripper actions
            # FOR THIS TO WORK THE GRIPPER ACTIONS MUST BE JOINT ACTUATIONS. I'M STILL NOT SURE IF THIS IS THE CASE.
            full[self.left_hand_indices] = gripper_action_dict["left"]
            full[self.right_hand_indices] = gripper_action_dict["right"]

            return torch.from_numpy(np.expand_dims(full, axis=0)) # unsqueeze to get the env dimension

        except Exception as e:
            self.robot = None
            return self.target_eef_pose_to_action(target_eef_pose_dict, gripper_action_dict, action_noise_dict, env_id)

    def action_to_target_eef_pose(self, action: torch.Tensor) -> dict[str, torch.Tensor]:
        """
        Converts action (compatible with env.step) to a target pose for the end effector controller.
        Inverse of @target_eef_pose_to_action. Usually used to infer a sequence of target controller poses
        from a demonstration trajectory using the recorded actions.

        Args:
            action: Environment action. Shape is (num_envs, action_dim).

        Returns:
            A dictionary of eef pose torch.Tensor that @action corresponds to.
        """

        try:

            self.lazy_load_solvers()

            # set the root pose of the lula solver, to ensure correct inverse kinematics
            robot_base_translation, robot_base_orientation = self.robot.get_world_pose()
            robot_base_translation = robot_base_translation.cpu().detach().numpy()
            robot_base_orientation = robot_base_orientation.cpu().detach().numpy()
            self.lula_solver.set_robot_base_pose(robot_base_translation, robot_base_orientation)

            combined_arm_action = action[0][self.solver_to_action_mapping] # index into the env dimension

            left_pos, left_rot_mat = self.lula_solver.compute_forward_kinematics("left_hand_palm_link", combined_arm_action)
            right_pos, right_rot_mat = self.lula_solver.compute_forward_kinematics("right_hand_palm_link", combined_arm_action)

            left_pos = torch.from_numpy(left_pos)
            right_pos = torch.from_numpy(right_pos)
            left_rot_mat = torch.from_numpy(left_rot_mat)
            right_rot_mat = torch.from_numpy(right_rot_mat)

            left_target_eef_pose = PoseUtils.make_pose(left_pos, left_rot_mat)
            right_target_eef_pose = PoseUtils.make_pose(right_pos, right_rot_mat)

            return {"left": left_target_eef_pose, "right": right_target_eef_pose}
        
        except Exception as e:
            self.robot = None
            return self.action_to_target_eef_pose(action)

    def actions_to_gripper_actions(self, actions: torch.Tensor) -> dict[str, torch.Tensor]:
        """
        Extracts the gripper actuation part from a sequence of env actions (compatible with env.step).

        Args:
            actions: environment actions. The shape is (num_envs, num steps in a demo, action_dim).

        Returns:
            A dictionary of torch.Tensor gripper actions. Key to each dict is an eef_name.
        """

        try:

            self.lazy_load_solvers()

            return {
                "left": actions[:,:,self.left_hand_indices],
                "right": actions[:,:,self.right_hand_indices]
            }

        except Exception as e:
            self.robot = None
            return self.actions_to_gripper_actions(actions)