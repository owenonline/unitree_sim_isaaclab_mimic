# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
from action_provider.action_base import ActionProvider
from typing import Optional
import torch
from tools.data_json_load import load_robot_data
from image_server.shared_memory_utils import MultiImageReader
from tools.episode_writer import EpisodeWriter
import json
from typing import List, Optional
import numpy as np
import time
class FileActionProviderReplay(ActionProvider):
    """Action provider based on DDS"""
    
    def __init__(self,env, robot_type="g129", enable_gripper=False, enable_dex3=False,generate_data=False,generate_data_dir="",frequency=30,rerun_log=True):
        super().__init__("FileActionProviderReplay")
        self.env = env
        self.enable_robot = robot_type
        self.enable_gripper = enable_gripper
        self.enable_dex3 = enable_dex3
        self.generate_data = generate_data
        self.generate_data_dir = generate_data_dir
        self.action_index = 10**1000
        self.total_step_num =0
        self.start_loop = True
        self.saved_data = True
        self.all_joint_names = env.scene["robot"].data.joint_names
        self.joint_to_index = {name: i for i, name in enumerate(self.all_joint_names)}
        self._setup_joint_mapping()
        self.multi_image_reader=None
        self.recorder=None
        if generate_data:
            # 只有在保存数据且需要图像时才创建MultiImageReader
            # 对于replay场景，通常不需要实时图像数据
            try:
                self.multi_image_reader = MultiImageReader()
                print(f"[{self.name}] MultiImageReader 创建成功")
            except Exception as e:
                print(f"[{self.name}] MultiImageReader 创建失败: {e}")
                print(f"[{self.name}] 将禁用图像数据保存功能")
                self.multi_image_reader = None
            
            self.recorder = EpisodeWriter(task_dir = generate_data_dir, frequency = frequency, rerun_log = rerun_log)
        print(f"FileActionProviderReplay init ok")
    def load_data(self, file_path):
        """Setup DDS communication"""
        self.robot_action, self.hand_action, self.sim_state_list,self.task_name_list,self.sim_state_json_list = load_robot_data(file_path)
        
        self.total_step_num = len(self.robot_action)
        self.total_hand_step_num = len(self.hand_action)
        if self.total_hand_step_num != self.total_hand_step_num:
            raise ValueError("total_hand_step_num is NaN. Please check your data or initialization.")
        if self.generate_data:
            # tem_sim_state  = self.sim_state_to_json(self.sim_state_json_list[0])
            self.recorder.create_episode()
            self.saved_data = False
        
        self.start_loop = False
        
        return self.sim_state_list[0],self.task_name_list[0]
    def start_replay(self):
        self.action_index=0
    def get_start_loop(self):
        return self.start_loop
    def _setup_joint_mapping(self):
        """Setup joint mapping"""
        if self.enable_robot == "g129":
            self.arm_joint_mapping = {
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
            self.left_arm_joint = [        
                "left_shoulder_pitch_joint",
                "left_shoulder_roll_joint",
                "left_shoulder_yaw_joint",
                "left_elbow_joint",
                "left_wrist_roll_joint",
                "left_wrist_pitch_joint",
                "left_wrist_yaw_joint"]
            self.right_arm_joint = [        
                "right_shoulder_pitch_joint",
                "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint",
                "right_elbow_joint",
                "right_wrist_roll_joint",
                "right_wrist_pitch_joint",
                "right_wrist_yaw_joint"]
            self.left_arm_joint_indices = [self.joint_to_index[name] for name in self.left_arm_joint]
            self.right_arm_joint_indices = [self.joint_to_index[name] for name in self.right_arm_joint]
            
        if self.enable_gripper:
            self.gripper_joint_mapping = {
                "left_hand_Joint1_1": 1,
                "left_hand_Joint2_1": 1,
                "right_hand_Joint1_1": 0,
                "right_hand_Joint2_1": 0,
            }
            self.left_hand_joint = ["left_hand_Joint1_1"]
            self.right_hand_joint = ["right_hand_Joint1_1"]
        if self.enable_dex3:
            self.left_hand_joint_mapping = {
                "left_hand_thumb_0_joint":0,
                "left_hand_thumb_1_joint":1,
                "left_hand_thumb_2_joint":2,
                "left_hand_middle_0_joint":3,
                "left_hand_middle_1_joint":4,
                "left_hand_index_0_joint":5,
                "left_hand_index_1_joint":6}
            self.right_hand_joint_mapping = {
                "right_hand_thumb_0_joint":0,     
                "right_hand_thumb_1_joint":1,
                "right_hand_thumb_2_joint":2,
                "right_hand_middle_0_joint":3,
                "right_hand_middle_1_joint":4,
                "right_hand_index_0_joint":5,
                "right_hand_index_1_joint":6}
            self.left_hand_joint = [
                # hand joints (14)
                # left hand (7)
                "left_hand_thumb_0_joint",
                "left_hand_thumb_1_joint",
                "left_hand_thumb_2_joint",
                "left_hand_middle_0_joint",
                "left_hand_middle_1_joint",
                "left_hand_index_0_joint",
                "left_hand_index_1_joint"]
            self.right_hand_joint = [
                    "right_hand_thumb_0_joint",
                    "right_hand_thumb_1_joint",
                    "right_hand_thumb_2_joint",
                    "right_hand_middle_0_joint",
                    "right_hand_middle_1_joint",
                    "right_hand_index_0_joint",
                    "right_hand_index_1_joint",
                ]
        self.left_hand_joint_indices = [self.joint_to_index[name] for name in self.left_hand_joint]
        self.right_hand_joint_indices = [self.joint_to_index[name] for name in self.right_hand_joint]
        self.all_joint_indices = self.left_arm_joint_indices + self.right_arm_joint_indices #+ self.left_hand_joint_indices + self.right_hand_joint_indices
    def compare_states(self,state_from_dataset,runtime_state,runtime_env_index,diff_threshold_robot=0.001,diff_threshold_object=0.001,compare_joint_indices: Optional[List[int]] = None
    ) -> (bool, str):
        """Compare states from dataset and runtime.

        Args:
            state_from_dataset: 数据集中的状态
            runtime_state: 运行时状态
            runtime_env_index: 要比较的环境索引
            compare_joint_indices: 可选，指定要比较的 ["articulation"]["robot"]["joint_position"] 的索引列表

        Returns:
            bool: 状态是否匹配
            str: 不匹配时的日志信息
        """
        states_matched = True
        output_log = ""

        for asset_type in ["articulation", "rigid_object"]:
            for asset_name in runtime_state[asset_type].keys():
                          # 仅针对 rigid_object 类型：如果物体未静止，则跳过整个物体
                if asset_type == "rigid_object":
                    root_velocity = runtime_state[asset_type][asset_name].get("root_velocity", None)
                    if root_velocity is not None:
                        if root_velocity.ndim == 2:
                            velocity = root_velocity[runtime_env_index]
                        else:
                            velocity = root_velocity
                        # 如果不为全 0，跳过该刚体的所有状态比较
                        if not torch.allclose(velocity, torch.zeros_like(velocity), atol=1e-5):
                            continue
                for state_name in runtime_state[asset_type][asset_name].keys():
                    runtime_asset_state = runtime_state[asset_type][asset_name][state_name]
                    dataset_asset_state = state_from_dataset[asset_type][asset_name][state_name]

                    # 若是 batched 环境，则索引出目标环境
                    if runtime_asset_state.ndim == 2:
                        runtime_asset_state = runtime_asset_state[runtime_env_index]
                    if dataset_asset_state.ndim == 2:
                        dataset_asset_state = dataset_asset_state[runtime_env_index]

                    if runtime_asset_state.shape != dataset_asset_state.shape:
                        raise ValueError(f"State shape of {state_name} for asset {asset_name} don't match")

                    # 判断是否为 joint_position 并且需要索引比较
                    if (
                        asset_type == "articulation"
                        and asset_name == "robot"
                        and state_name == "joint_position"
                        and compare_joint_indices is not None
                    ):
                        indices_to_compare = compare_joint_indices
                    else:
                        indices_to_compare = range(len(dataset_asset_state))

                    for i in indices_to_compare:
                        if asset_name == "robot":
                            if "velocity" not in state_name and abs(dataset_asset_state[i] - runtime_asset_state[i]) > diff_threshold_robot:
                                states_matched = False
                                output_log += f'\tState ["{asset_type}"]["{asset_name}"]["{state_name}"][{i}] don\'t match\r\n'
                                output_log += f"\t  Dataset:\t{dataset_asset_state[i]}\r\n"
                                output_log += f"\t  Runtime: \t{runtime_asset_state[i]}\r\n"
                        else:
                            if "velocity" not in state_name and abs(dataset_asset_state[i] - runtime_asset_state[i]) > diff_threshold_object:
                                states_matched = False
                                output_log += f'\tState ["{asset_type}"]["{asset_name}"]["{state_name}"][{i}] don\'t match\r\n'
                                output_log += f"\t  Dataset:\t{dataset_asset_state[i]}\r\n"
                                output_log += f"\t  Runtime: \t{runtime_asset_state[i]}\r\n"
        return states_matched, output_log


    def get_action(self, env) -> Optional[torch.Tensor]:
        """Get action from DDS"""
        try:
            # Get robot command
            if self.action_index < self.total_step_num:
                if self.enable_robot == "g129":
                    arm_cmd_data = self.robot_action[self.action_index]
                    # for joint_name, arm_idx in self.arm_joint_mapping.items():
                    #     if joint_name in self.joint_to_index:
                    #         full_action[self.joint_to_index[joint_name]] = arm_cmd_data[arm_idx]
                
                # Get gripper command
                if self.enable_gripper:
                    hand_cmd_data = self.hand_action[self.action_index]
                    # for joint_name, gripper_idx in self.gripper_joint_mapping.items():
                    #     if joint_name in self.joint_to_index:
                    #         full_action[self.joint_to_index[joint_name]] = self._convert_to_joint_range(hand_cmd_data[gripper_idx])

                
                # Get hand command
                if self.enable_dex3:
                    hand_cmd_data = self.hand_action[self.action_index]
                    # for joint_name, hand_idx in self.left_hand_joint_mapping.items():
                    #     if joint_name in self.joint_to_index:
                    #         full_action[self.joint_to_index[joint_name]] = hand_cmd_data[hand_idx]
                    # for joint_name, hand_idx in self.right_hand_joint_mapping.items():
                    #     if joint_name in self.joint_to_index:
                    #         full_action[self.joint_to_index[joint_name]] = hand_cmd_data[hand_idx]
                # 使用简单可靠的reset_to方法
                flag ,log = self.compare_states(self.sim_state_list[self.action_index],self.env.scene.get_state(),0,compare_joint_indices=self.all_joint_indices)
                # if not flag:
                    # print(f"[{self.name}] State mismatch: {log}")
                env.reset_to(self.sim_state_list[self.action_index], torch.tensor([0], device=env.device), is_relative=True)
                
                if self.generate_data:
                    self.save_date(env,arm_cmd_data,hand_cmd_data,self.sim_state_json_list[self.action_index])
                
                # 顺序播放每一帧，不跳帧
                self.action_index += 1
            else:
                self.action_index = 10**1000
                if self.generate_data: 
                    if not self.saved_data:
                        self.recorder.save_episode()
                        self.saved_data = True
                    if self.recorder.is_available:
                        self.start_loop=True
                else:
                    self.start_loop=True
                
            # print(f"full_action: {full_action}")
            return None
            
        except Exception as e:
            print(f"[{self.name}] Get DDS action failed: {e}")
            return None
    
    def _convert_to_joint_range(self, value):
        """Convert gripper control value to joint angle"""
        input_min, input_max = 0.0, 5.6
        output_min, output_max = 0.03, -0.02
        value = max(input_min, min(input_max, value))
        return output_min + (output_max - output_min) * (value - input_min) / (input_max - input_min)
    def _convert_to_gripper_range(self, value):
        """Convert the Isaac Lab joint angle to the gripper control value [-0.02, 0.03] -> [5.6, 0]
        
        Args:
            value: the input value, range in [-0.02, 0.03]
                  -0.02: fully open
                  0.03: fully closed
            
        Returns:
            float: the converted value, range in [5.6, 0]
                  5.6: fully open
                  0.0: fully closed
        """
        # input range (joint angle)
        input_min = 0.03   # fully closed
        input_max = -0.02  # fully open
        
        # output range (gripper control value)
        output_min = 0.0   # fully closed
        output_max = 5.6   # fully open
        
        # ensure the input value is in the valid range
        value = max(input_max, min(input_min, value))
        
        # linear mapping conversion
        converted_value = output_min + (output_max - output_min) * (input_min - value) / (input_min - input_max)
        
        return converted_value
    def cleanup(self):
        """Clean up DDS resources"""
        if self.multi_image_reader:
            self.multi_image_reader.close()
        if self.recorder:
            self.recorder.close()
        self.is_running = False
        print(f"[{self.name}] 资源清理完成 - Resource cleanup completed")
    def get_state(self,env):

        joint_pos = env.scene["robot"].data.joint_pos
        left_arm_joint_pose = joint_pos[:,self.left_arm_joint_indices][0].detach().cpu().numpy().tolist()
        right_arm_joint_pose = joint_pos[:,self.right_arm_joint_indices][0].detach().cpu().numpy().tolist()
        left_hand_joint_pose = joint_pos[:,self.left_hand_joint_indices][0].detach().cpu().numpy().tolist()
        right_hand_joint_pose = joint_pos[:,self.right_hand_joint_indices][0].detach().cpu().numpy().tolist()

        return left_arm_joint_pose,right_arm_joint_pose,left_hand_joint_pose,right_hand_joint_pose

    def get_images(self,image_count=3):
        concatenated_image = self.multi_image_reader.read_concatenated_image()
        height, total_width, channels = concatenated_image.shape
        single_width = total_width // image_count
        if total_width % image_count != 0:
            raise ValueError("Total width is not divisible by image_count. Cannot split cleanly.")

        # 依次切分图像
        images = {}
        names = ['head', 'left', 'right']
        for i, name in enumerate(names[:image_count]):
            x_start = i * single_width
            x_end = x_start + single_width
            images[name] = concatenated_image[:, x_start:x_end, :]
        return images
    def save_date(self,env,arm_action,hand_action,sim_state=None):
        left_arm_state,right_arm_state,left_ee_state,right_ee_state = self.get_state(env)
        images = self.get_images()
        colors = {}
        depths = {}
        left_arm_action = arm_action[:7].tolist()
        right_arm_action = arm_action[7:].tolist()
        if self.enable_gripper:
            left_hand_action = np.array(hand_action[0]).tolist()
            right_hand_action = np.array(hand_action[1]).tolist()
        elif self.enable_dex3:
            left_hand_action = hand_action[:7].tolist()
            right_hand_action = hand_action[7:].tolist()
        colors[f"color_{0}"] = images["head"]
        colors[f"color_{1}"] = images["left"]
        colors[f"color_{2}"] = images["right"]
        states = {
            "left_arm": {                                                                    
                "qpos":   left_arm_state,    # numpy.array -> list
                "qvel":   [],                          
                "torque": [],                        
            }, 
            "right_arm": {                                                                    
                "qpos":   right_arm_state,       
                "qvel":   [],                          
                "torque": [],                         
            },                        
            "left_ee": {                                                                    
                "qpos":   left_ee_state,           
                "qvel":   [],                           
                "torque": [],                          
            }, 
            "right_ee": {                                                                    
                "qpos":   right_ee_state,       
                "qvel":   [],                           
                "torque": [],  
            }, 
            "body": {
                "qpos": [],
            }, 
        }
        actions = {
            "left_arm": {                                   
                "qpos":   left_arm_action,       
                "qvel":   [],       
                "torque": [],      
            }, 
            "right_arm": {                                   
                "qpos":   right_arm_action,       
                "qvel":   [],       
                "torque": [],       
            },                         
            "left_ee": {                                   
                "qpos":   left_hand_action,       
                "qvel":   [],       
                "torque": [],       
            }, 
            "right_ee": {                                   
                "qpos":   right_hand_action,       
                "qvel":   [],       
                "torque": [], 
            }, 
            "body": {
                "qpos": [],
            }, 
        }
        self.recorder.add_item(colors=colors, depths=depths, states=states, actions=actions,sim_state=sim_state)
    def sim_state_to_json(self,data):
        data_serializable = self.tensors_to_list(data)
        json_str = json.dumps(data_serializable)
        return json_str
    def tensors_to_list(self, obj):
        if isinstance(obj, torch.Tensor):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {k: self.tensors_to_list(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [self.tensors_to_list(i) for i in obj]
        return obj