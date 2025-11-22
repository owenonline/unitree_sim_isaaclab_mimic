# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0      
# Adapted by Hanyang Gu for pressing knob task
"""
Public base scene configuration module for button pressing task.

SCENE OBJECTS:
- object_0 (Red), object_1 (Green), object_2 (Blue), object_3 (Yellow)
- All buttons: radius=0.045m, height=0.025m, mass=0.1kg

This scene supports random button spawning (1-4 buttons) with collision avoidance.
Button reset logic is implemented in task-specific mdp folder:
    tasks/g1_tasks/press_knob_g1_29dof_dex3/mdp/button_reset_mdp.py
"""
import isaaclab.sim as sim_utils
from isaaclab.assets import  AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from tasks.common_config import   CameraBaseCfg  # isort: skip
import os
project_root = os.environ.get("PROJECT_ROOT")
@configclass
class TableKnobSceneCfg(InteractiveSceneCfg): # inherit from the interactive scene configuration class
    """object table scene configuration class
    defines a complete scene containing robot, object, table, etc.
    """
      # 1. room wall configuration - simplified configuration to avoid rigid body property conflicts
    room_walls = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Room",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[0.0, 0.0, 0],  # 房间中心点
            rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse.usd",  # use simple room model
        ),
    )
    # print(f"ISAAC_NUCLEUS_DIR: {ISAAC_NUCLEUS_DIR}")
    #ISAAC_NUCLEUS_DIR: http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac
        # 1. table configuration
    packing_table = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable",    # table in the scene
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.0, 0.55, -0.2],   # initial position [x, y, z]
                                                rot=[1.0, 0.0, 0.0, 0.0]), # initial rotation [x, y, z, w]
        spawn=UsdFileCfg(
            # usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/PackingTable/packing_table.usd",    # table model file
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",    # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),    # set to kinematic object
        ),
    )

    packing_table_2 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_2",   
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-3.5, 0.55, -0.2],  
                                                rot=[1.0, 0.0, 0.0, 0.0]), 
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",    # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),   
        ),
    )
    packing_table_3 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_3",   
        init_state=AssetBaseCfg.InitialStateCfg(pos=[3.5, 0.55, -0.2],  
                                                rot=[1.0, 0.0, 0.0, 0.0]), 
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",    # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),   
        ),
    )
    packing_table_4 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_4",   
        init_state=AssetBaseCfg.InitialStateCfg(pos=[3.5, -5, -0.2],  
                                                rot=[1.0, 0.0, 0.0, 0.0]), 
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",    # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),   
        ),
    )
    packing_table_5 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_5",   
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-3.5, -5, -0.2],  
                                                rot=[1.0, 0.0, 0.0, 0.0]), 
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",    # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),   
        ),
    )
    packing_table_6 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_6",   
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.0, -5, -0.2],  
                                                rot=[1.0, 0.0, 0.0, 0.0]), 
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",    # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),   
        ),
    )
    # Objects - Define 4 buttons (maximum, will be randomly enabled 1-4 on reset)
    # 2. button configurations (pressable buttons/knobs - simple height-based detection)
    object_0 = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Object_0",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[-0.35, 0.40, 0.84],
                                                  rot=[1, 0, 0, 0]),
        spawn=sim_utils.CylinderCfg(
            radius=0.045,
            height=0.025,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,
                rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.2, 0.2), metallic=0.4),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="max",
                restitution_combine_mode="min",
                static_friction=0.3,
                dynamic_friction=0.3,
                restitution=0.1,
            ),
        ),
    )
    
    object_1 = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Object_1",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[-0.20, 0.40, 0.84],
                                                  rot=[1, 0, 0, 0]),
        spawn=sim_utils.CylinderCfg(
            radius=0.045,
            height=0.025,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,
                rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.9, 0.2), metallic=0.4),  # green
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="max",
                restitution_combine_mode="min",
                static_friction=0.3,
                dynamic_friction=0.3,
                restitution=0.1,
            ),
        ),
    )
    
    object_2 = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Object_2",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[-0.05, 0.40, 0.84],
                                                  rot=[1, 0, 0, 0]),
        spawn=sim_utils.CylinderCfg(
            radius=0.045,
            height=0.025,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,
                rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.2, 0.9), metallic=0.4),  # blue
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="max",
                restitution_combine_mode="min",
                static_friction=0.3,
                dynamic_friction=0.3,
                restitution=0.1,
            ),
        ),
    )
    
    object_3 = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Object_3",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[0.10, 0.40, 0.84],
                                                  rot=[1, 0, 0, 0]),
        spawn=sim_utils.CylinderCfg(
            radius=0.045,
            height=0.025,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,
                rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.9, 0.2), metallic=0.4),  # yellow
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="max",
                restitution_combine_mode="min",
                static_friction=0.3,
                dynamic_friction=0.3,
                restitution=0.1,
            ),
        ),
    )
    # Ground plane
    # 3. ground configuration
    # ground = AssetBaseCfg(
    #     prim_path="/World/GroundPlane",    # ground in the scene
    #     spawn=GroundPlaneCfg( ),    # ground configuration
    # )

    # Lights
    # 4. light configuration
    light = AssetBaseCfg(
        prim_path="/World/light",   # light in the scene
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), # light color (white)
                                     intensity=3000.0),    # light intensity
    )

    world_camera = CameraBaseCfg.get_camera_config(prim_path="/World/PerspectiveCamera",
                                                    pos_offset=(-0.1, 3.6, 1.6),
                                                    rot_offset=( -0.00617,0.00617, 0.70708, -0.70708),
                                                    focal_length = 16.5)