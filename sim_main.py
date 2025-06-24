
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
#!/usr/bin/env python3
import argparse
import contextlib
import time
import sys
import signal
import torch
import gymnasium as gym
from pathlib import Path

# Isaac Lab AppLauncher
from isaaclab.app import AppLauncher

from image_server.image_server import ImageServer

# add command line arguments
parser = argparse.ArgumentParser(description="Unitree Simulation")
parser.add_argument("--task", type=str, default="Isaac-PickPlace-G129-Head-Waist-Fix", help="task name")
parser.add_argument("--action_source", type=str, default="dds", 
                   choices=["dds", "file", "trajectory", "policy"], 
                   help="Action source")
parser.add_argument("--file_path", type=str, default="t.py", help="file path (when action_source=file)")
parser.add_argument("--step_hz", type=int, default=500, help="control frequency")
parser.add_argument("--robot_type", type=str, default="g129", help="robot type")
parser.add_argument("--enable_gripper_dds", action="store_true", help="enable gripper DDS")
parser.add_argument("--enable_dex3_dds", action="store_true", help="enable dexterous hand DDS")
parser.add_argument("--stats_interval", type=float, default=10.0, help="statistics print interval (seconds)")

# performance analysis parameters
parser.add_argument("--enable_profiling", action="store_true", default=True, help="enable performance analysis")
parser.add_argument("--profile_interval", type=int, default=500, help="performance analysis report interval (steps)")


# add AppLauncher parameters
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()


import pinocchio 
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# import after simulator starts
import omni.log
import isaaclab_mimic.envs  # noqa: F401
# import optimized layered control system
from layeredcontrol.robot_control_system import (
    RobotController, 
    ControlConfig,
)
from action_provider.action_provider_dds import DDSActionProvider
from action_provider.action_provider_file import FileActionProvider
from action_provider.action_provider_trajectory import TrajectoryActionProvider,create_trajectory_generator
from action_provider.action_provider_policy import PolicyActionProvider
from dds.reset_pose_dds import get_reset_pose_dds
# import existing modules
import tasks
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg


def create_action_provider(args):
    """create action provider based on parameters"""
    if args.action_source == "dds":
        return DDSActionProvider(
            robot_type=args.robot_type,
            enable_gripper=args.enable_gripper_dds, 
            enable_dex3=args.enable_dex3_dds
        )
    
    elif args.action_source == "file":
        if not Path(args.file_path).exists():
            print(f"error: file {args.file_path} does not exist")
            return None
        return FileActionProvider(args.file_path, loop=True)
    
    elif args.action_source == "trajectory":
        trajectory_gen = create_trajectory_generator()
        return TrajectoryActionProvider(trajectory_gen)
    
    elif args.action_source == "policy":
        # here can load the trained policy model
        print("policy mode not implemented")
        return None
    
    else:
        print(f"unknown action source: {args.action_source}")
        return None



def setup_signal_handlers(controller):
    """set signal handlers"""
    def signal_handler(signum, frame):
        print(f"\nreceived signal {signum}, stopping controller...")
        controller.stop()
        simulation_app.close()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)


def main():
    """main function"""
    print("=" * 60)
    print("robot control system started")
    print(f"任务: {args_cli.task}")
    print(f"Action源: {args_cli.action_source}")
    print("=" * 60)

    # parse environment configuration
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
    env_cfg.env_name = args_cli.task
    
    # remove timeout termination condition
    env_cfg.terminations.time_out = None
    
    # performance optimization configuration
    if hasattr(env_cfg, 'sim'):
        # optimize simulation settings
        env_cfg.sim.dt = 0.01  # normal mode
        if hasattr(env_cfg.sim, 'substeps'):
            env_cfg.sim.substeps = 1
        
        if hasattr(env_cfg.sim, 'use_gpu_pipeline'):
            env_cfg.sim.use_gpu_pipeline = True  # enable GPU pipeline
    
    # # optimize observation frequency
    if hasattr(env_cfg, 'decimation'):
            env_cfg.decimation = 2
    
    # disable unnecessary calculations
    if hasattr(env_cfg, 'enable_render'):
        env_cfg.enable_render = True  # keep rendering but optimize frequency
    
    # create environment
    print("\ncreate environment...")
    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
    # reset environment
    env.sim.reset()
    env.reset()
    
    
    config = {
        'fps': 30,
        'head_camera_type': 'opencv',
        'head_camera_image_shape': [480, 640],  # Head camera resolution
        # 'head_camera_id_numbers': [0],
        # 'wrist_camera_type': 'opencv',
        # 'wrist_camera_image_shape': [480, 640],  # Wrist camera resolution
        # 'wrist_camera_id_numbers': [2, 4],
    }
    server = ImageServer(config, Unit_Test=False)
    
    # create simplified control configuration
    control_config = ControlConfig(
        step_hz=args_cli.step_hz
    )
    
    # create controller
    print("\ncreate controller...")
    controller = RobotController(env, control_config)
    
    # create action provider
    print(f"\ncreate action provider: {args_cli.action_source}...")
    action_provider = create_action_provider(args_cli)
    
    if action_provider is None:
        print("action provider creation failed, exiting")
        return
    
    # set action provider
    controller.set_action_provider(action_provider)
    
    # configure performance analysis
    if args_cli.enable_profiling:
        controller.set_profiling(True, args_cli.profile_interval)
        print(f"performance analysis enabled, report every {args_cli.profile_interval} steps")
    else:
        controller.set_profiling(False)
        print("performance analysis disabled")

    
    # set signal handlers
    setup_signal_handlers(controller)

    reset_pose_dds = get_reset_pose_dds()
    reset_pose_dds.start_communication(enable_publish=False, enable_subscribe=True)
    try:
        # start controller - start asynchronous components
        print("\nstart controller...")
        controller.start()
        print("controller started, start main loop...")
        
        # main loop - execute in main thread to support rendering
        last_stats_time = time.time()
        loop_start_time = time.time()
        loop_count = 0
        last_loop_time = time.time()
        recent_loop_times = []  # for calculating moving average frequency
        
        # use torch.inference_mode() and exception suppression
        with contextlib.suppress(KeyboardInterrupt), torch.inference_mode():
            while simulation_app.is_running() and controller.is_running:
                current_time = time.time()
                loop_count += 1
                
                # calculate instantaneous loop time
                loop_dt = current_time - last_loop_time
                last_loop_time = current_time
                recent_loop_times.append(loop_dt)
                
                # keep recent 100 loop times
                if len(recent_loop_times) > 100:
                    recent_loop_times.pop(0)
                
                # execute control step (in main thread, support rendering)
                controller.step()
                # print(f"reset_pose_dds: {reset_pose_dds}")
                reset_pose_cmd = reset_pose_dds.get_reset_pose_command()
                # # print(f"reset_pose_cmd: {reset_pose_cmd}")
                if reset_pose_cmd is not None:
                    reset_category = reset_pose_cmd.get("reset_category")
                    # print(f"reset_category: {reset_category}")
                    if reset_category == '1':
                        print("reset object")
                        env_cfg.event_manager.trigger("reset_object_self", env)
                        reset_pose_dds.write_reset_pose_command(-1)
                    elif reset_category == '2':
                        print("reset all")
                        env_cfg.event_manager.trigger("reset_all_self", env)
                        reset_pose_dds.write_reset_pose_command(-1)
                # print statistics and loop frequency periodically
                if current_time - last_stats_time >= args_cli.stats_interval:
                    # calculate while loop execution frequency
                    elapsed_time = current_time - loop_start_time
                    loop_frequency = loop_count / elapsed_time if elapsed_time > 0 else 0
                    
                    # calculate moving average frequency (based on recent loop times)
                    if recent_loop_times:
                        avg_loop_time = sum(recent_loop_times) / len(recent_loop_times)
                        moving_avg_frequency = 1.0 / avg_loop_time if avg_loop_time > 0 else 0
                        min_loop_time = min(recent_loop_times)
                        max_loop_time = max(recent_loop_times)
                        max_freq = 1.0 / min_loop_time if min_loop_time > 0 else 0
                        min_freq = 1.0 / max_loop_time if max_loop_time > 0 else 0
                    else:
                        moving_avg_frequency = 0
                        min_freq = max_freq = 0
                    
                    print(f"\n=== While loop execution frequency statistics ===")
                    print(f"loop execution count: {loop_count}")
                    print(f"running time: {elapsed_time:.2f} seconds")
                    print(f"overall average frequency: {loop_frequency:.2f} Hz")
                    print(f"moving average frequency: {moving_avg_frequency:.2f} Hz (last {len(recent_loop_times)} times)")
                    print(f"frequency range: {min_freq:.2f} - {max_freq:.2f} Hz")
                    print(f"average loop time: {(elapsed_time/loop_count*1000):.2f} ms")
                    if recent_loop_times:
                        print(f"recent loop time: {(avg_loop_time*1000):.2f} ms")
                    print(f"=============================")
                    
                    # print_stats(controller)
                    last_stats_time = current_time
                
                # check environment state
                if env.sim.is_stopped():
                    print("\nenvironment stopped")
                    break
                # rate_limiter.sleep(env)
    except KeyboardInterrupt:
        print("\nuser interrupted program")
    
    except Exception as e:
        print(f"\nprogram exception: {e}")
    
    finally:
        # clean up resources
        print("\nclean up resources...")
        controller.cleanup()
        
        env.close()
        print("cleanup completed")


if __name__ == "__main__":
    try:
        main()
    finally:
        # ensure simulator is closed
        simulation_app.close() 

# python sim_main.py --device cpu  --enable_cameras  --task  Isaac-PickPlace-Cylinder-G129-Dex1-Joint    --enable_gripper_dds --robot_type g129

#  python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint    --enable_dex3_dds --robot_type g129
#python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-G129-Dex1-Joint     --enable_gripper_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-G129-Dex3-Joint    --enable_dex3_dds --robot_type g129