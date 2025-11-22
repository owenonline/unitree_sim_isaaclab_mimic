# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
Button Press Task Environment Configuration with Random Multi-Button Spawning

RANDOM BUTTON SPAWNING:
- On each reset, 1-4 buttons spawn randomly with collision avoidance
- Scene objects: object_0 (red), object_1 (green), object_2 (blue), object_3 (yellow)
- Inactive buttons are moved to [100, 100, -10] (far away from workspace)

IMPORTANT - Current Limitations:
- Reward function (compute_reward) expects single "object" - may need updates for multi-button
- Termination function (reset_object_estimate) expects single "object" - may need updates
- You may need to modify mdp/rewards.py and mdp/terminations.py to handle multiple buttons

To customize button spawning, modify the reset_buttons_random parameters in __post_init__:
- min_buttons/max_buttons: Control how many buttons spawn (1-4)
- base_pos: Center of spawning area
- position_randomization: Size of random spawn area (x, y, z offsets)
- min_distance: Minimum spacing between buttons (default 0.12m)

See mdp/button_reset_mdp.py for implementation details.
"""

import tempfile
import torch
from dataclasses import MISSING



import isaaclab.envs.mdp as base_mdp
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.utils import configclass

from . import mdp
# use Isaac Lab native event system

from tasks.common_config import  G1RobotPresets, CameraPresets  # isort: skip
from tasks.common_event.event_manager import SimpleEvent, SimpleEventManager

# import public scene configuration
from tasks.common_scene.adapted_press_knobcfg import TableKnobSceneCfg

##
# Scene definition
##

@configclass
class ObjectTableSceneCfg(TableKnobSceneCfg):
    """object table scene configuration class
    
    inherits from G1SingleObjectSceneCfg, gets the complete G1 robot scene configuration
    can add task-specific scene elements or override default configurations here
    """
    
    # Humanoid robot w/ arms higher
    # 5. humanoid robot configuration 
    robot: ArticulationCfg = G1RobotPresets.g1_29dof_dex3_base_fix()
    # 6. add camera configuration 
    front_camera = CameraPresets.g1_front_camera()
    left_wrist_camera = CameraPresets.left_dex3_wrist_camera()
    right_wrist_camera = CameraPresets.right_dex3_wrist_camera()

##
# MDP settings
##
@configclass
class ActionsCfg:
    """defines the action configuration related to robot control, using direct joint angle control
    """
    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=1.0, use_default_offset=True)



@configclass
class ObservationsCfg:
    """defines all available observation information
    """
    @configclass
    class PolicyCfg(ObsGroup):
        """policy group observation configuration class
        defines all state observation values for policy decision
        inherit from ObsGroup base class 
        """

        # 1. robot joint state observation
        robot_joint_state = ObsTerm(func=mdp.get_robot_boy_joint_states)
        # 2. gripper joint state observation 
        robot_gipper_state = ObsTerm(func=mdp.get_robot_dex3_joint_states)

        # 3. camera image observation
        camera_image = ObsTerm(func=mdp.get_camera_image)

        def __post_init__(self):
            """post initialization function
            set the basic attributes of the observation group
            """
            self.enable_corruption = False  # disable observation value corruption
            self.concatenate_terms = False  # disable observation item connection
    # observation groups
    # create policy observation group instance
    policy: PolicyCfg = PolicyCfg()

# @configclass
# class TerminationsCfg:
#     # check if the object is out of the working range
#     # success = DoneTerm(func=mdp.reset_object_estimate)# use task completion check function
#     success = DoneTerm(func=lambda env: False)
@configclass
class TerminationsCfg:
    # Disable termination check for now since you have multiple buttons
    success = DoneTerm(func=lambda env: torch.zeros(env.num_envs, dtype=torch.bool, device=env.device))

    
@configclass
class RewardsCfg:
    # We explicitly pass params here so we don't rely on defaults in the function
    reward = RewTerm(
        func=mdp.compute_reward,
        weight=1.0,
        params={
            "hand_side": "left",     # Change to "right" if using right hand
            "touch_tolerance": 0.05,
            "max_buttons": 4         # MUST match the number of object_X prims in your scene
        }
    )
    
@configclass
class EventCfg:
    # NOTE: EventTermCfg with mode="reset" causes physics tensor invalidation during teleop
    # Button randomization is handled via custom event_manager in __post_init__
    # and triggered explicitly via DDS commands from sim_main.py (not via Isaac Lab's native reset)
    pass


@configclass
class PressKnobG129DEX3JointEnvCfg(ManagerBasedRLEnvCfg):
    """uNITREE G1 robot press knob environment configuration class
    inherits from ManagerBasedRLEnvCfg, defines all configuration parameters for the entire environment
    """

    # 1. scene settings
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=1, # environment number: 1
                                                     env_spacing=2.5, # environment spacing: 2.5 meter
                                                     replicate_physics=True # enable physics replication
                                                     )
    # basic settings
    observations: ObservationsCfg = ObservationsCfg()   # observation configuration
    actions: ActionsCfg = ActionsCfg()                  # action configuration
    # MDP settings
    terminations: TerminationsCfg = TerminationsCfg()    # termination configuration
    events = EventCfg()                                  # event configuration
    commands = None # command manager
    rewards: RewardsCfg = RewardsCfg()  # reward manager
    curriculum = None # curriculum manager
    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 20.0
        # simulation settings
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625

        # Create event manager for external DDS-triggered resets
        self.event_manager = SimpleEventManager()

        # Register "reset_object_self" - Only randomize buttons (robot stays in place)
        # Triggered by DDS reset_category='1'
        self.event_manager.register("reset_object_self", SimpleEvent(
            func=lambda env: mdp.reset_buttons_random(
                env,
                torch.arange(env.num_envs, device=env.device),
                min_buttons=1,
                max_buttons=4,
                base_pos=(-0.35, 0.40, 0.84),
                position_randomization=(0.15, 0.15, 0.0),
                min_distance=0.12
            )
        ))
        
        # Register "reset_all_self" - Reset robot AND randomize buttons (full reset)
        # Triggered by DDS reset_category='2'
        self.event_manager.register("reset_all_self", SimpleEvent(
            func=lambda env: mdp.reset_robot_and_buttons(
                env,
                torch.arange(env.num_envs, device=env.device),
                min_buttons=1,
                max_buttons=4,
                base_pos=(-0.35, 0.40, 0.84),
                position_randomization=(0.15, 0.15, 0.0),
                min_distance=0.12
            )
        ))