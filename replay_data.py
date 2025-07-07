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

# add command line arguments
parser = argparse.ArgumentParser(description="Unitree Simulation")
parser.add_argument("--task", type=str, default="Isaac-PickPlace-G129-Head-Waist-Fix", help="task name")
parser.add_argument("--action_source", type=str, default="replay", 
                   choices=["dds", "file", "trajectory", "policy", "replay"], 
                   help="Action source")
parser.add_argument("--file_path", type=str, default="/home/unitree/Code/avp_teleoperate/teleop/utils/data", help="file path (when action_source=file)")

parser.add_argument("--robot_type", type=str, default="g129", help="robot type")
parser.add_argument("--enable_gripper_dds", action="store_true", help="enable gripper DDS")
parser.add_argument("--enable_dex3_dds", action="store_true", help="enable dexterous hand DDS")
parser.add_argument("--stats_interval", type=float, default=10.0, help="statistics print interval (seconds)")

parser.add_argument("--generate_data_dir", type=str, default="./data", help="save data dir")
parser.add_argument("--generate_data", type=bool, default=True, help="generate data")

# performance analysis parameters
parser.add_argument("--step_hz", type=int, default=500, help="control frequency")
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
import isaaclab_mimic.envs  
# import optimized layered control system
from layeredcontrol.robot_control_system import (
    RobotController, 
    ControlConfig,
)

# import existing modules
import tasks
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

import omni.usd
from tools.augmentation_utils import (
    update_light,
    batch_augment_cameras_by_name,
)
from tools.data_json_load import get_data_json_list

from pathlib import Path
from action_provider.create_action_provider import create_action_provider
reset_pose_dds = None
controller = None
action_provider = None
shutdown_requested = False  # 添加全局退出标志




def setup_signal_handlers(controller,env):
    """set signal handlers"""
    def signal_handler(signum, frame):
        print(f"\nreceived signal {signum}, stopping controller...")
        
        # 设置退出标志
        global shutdown_requested
        shutdown_requested = True
        
        try:
            controller.stop()
        except Exception as e:
            print(f"停止controller失败: {e}")

    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

def main():
    """main function"""
    global reset_pose_dds, controller, action_provider
    
    # 设置进程组，确保能够管理所有子进程
    import os
    import atexit
    
    # 创建新的进程组
    try:
        os.setpgrp()
        current_pgid = os.getpgrp()
        print(f"设置进程组: {current_pgid}")
        
        # 注册退出时的清理函数
        def cleanup_process_group():
            try:
                print(f"清理进程组: {current_pgid}")
                import signal
                os.killpg(current_pgid, signal.SIGTERM)
            except Exception as e:
                print(f"清理进程组失败: {e}")
        
        atexit.register(cleanup_process_group)
        
    except Exception as e:
        print(f"设置进程组失败: {e}")
    
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
    # update_light(
    #     prim_path="/World/light",
    #     color=(1.0, 0.8, 0.6),
    #     intensity=20000.0,
    #     position=(1.0, 2.0, 3.0),
    #     radius=0.2,
    #     enabled=True,
    #     cast_shadows=True
    # )
    # reset environment
    env.sim.reset()
    env.reset()
    
    # create simplified control configuration
    control_config = ControlConfig(
        step_hz=args_cli.step_hz,
        replay_mode=True
    )
    
    # create controller
    print("\ncreate controller...")
    controller = RobotController(env, control_config)
    
    # create action provider
    print(f"\ncreate action provider: {args_cli.action_source}...")
    action_provider = create_action_provider(env=env,args=args_cli)
    
    if action_provider is None:
        print("action provider creation failed, exiting")
        return
    
    # set action provider
    print(f"\nset action provider.....")
    controller.set_action_provider(action_provider)
   
    # configure performance analysis
    if args_cli.enable_profiling:
        controller.set_profiling(True, args_cli.profile_interval)
        print(f"performance analysis enabled, report every {args_cli.profile_interval} steps")
    else:
        controller.set_profiling(False)
        print("performance analysis disabled")

    # 设置信号处理器 - Set up signal handlers
    print(f"\nset signal handlers.....")
    setup_signal_handlers(controller,env)
    data_json_list = get_data_json_list(args_cli.file_path)
    # time.sleep(10)
    try:
        print(f"action_provider:=================")
        print("\nstart controller...")
        controller.start()
        print("controller started, start main loop...")
        
        # main loop - execute in main thread to support rendering
        last_stats_time = time.time()
        loop_start_time = time.time()
        loop_count = 0
        last_loop_time = time.time()
        recent_loop_times = []  # for calculating moving average frequency
        data_idx=0
        with contextlib.suppress(KeyboardInterrupt), torch.inference_mode():
            while simulation_app.is_running() and controller.is_running:
                
                if action_provider.get_start_loop() and data_idx<len(data_json_list):
                    print(f"data_idx: {data_idx}")
                    sim_state,task_name = action_provider.load_data(data_json_list[data_idx])
                    if task_name!=args_cli.task:
                        raise ValueError(f" The {task_name} in the dataset is different from the {args_cli.task} being executed .")
                    env.reset_to(sim_state, torch.tensor([0], device=env.device), is_relative=True)
                    env.sim.reset()
                    time.sleep(0.2)
                    action_provider.start_replay()
                    data_idx+=1
                    
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
                    
                    last_stats_time = current_time
                
                # check environment state
                if env.sim.is_stopped():
                    print("\nenvironment stopped")
                    break
                    
        # 检查退出条件
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
    except KeyboardInterrupt:
        print("\n主程序收到中断信号")
    finally:
        print("执行最终清理...")
        
        # 获取当前进程信息
        import os
        import subprocess
        import signal
        import time
        
        current_pid = os.getpid()
        print(f"当前主进程PID: {current_pid}")
        
        try:
            # 查找所有相关的Python进程
            result = subprocess.run(['pgrep', '-f', 'replay_data.py'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                pids = result.stdout.strip().split('\n')
                print(f"找到相关进程: {pids}")
                
                for pid in pids:
                    if pid and pid != str(current_pid):
                        try:
                            print(f"终止子进程: {pid}")
                            os.kill(int(pid), signal.SIGTERM)
                        except ProcessLookupError:
                            print(f"进程 {pid} 已经不存在")
                        except Exception as e:
                            print(f"无法终止进程 {pid}: {e}")
                
                # 等待进程退出
                time.sleep(2)
                
                # 检查是否还有残留进程，强制杀死
                result2 = subprocess.run(['pgrep', '-f', 'replay_data.py'], 
                                       capture_output=True, text=True)
                if result2.returncode == 0:
                    remaining_pids = result2.stdout.strip().split('\n')
                    for pid in remaining_pids:
                        if pid and pid != str(current_pid):
                            try:
                                print(f"强制杀死进程: {pid}")
                                os.kill(int(pid), signal.SIGKILL)
                            except Exception as e:
                                print(f"无法强制杀死进程 {pid}: {e}")
                                
        except Exception as e:
            print(f"进程清理过程中出错: {e}")
        
        try:
            simulation_app.close()
        except Exception as e:
            print(f"关闭仿真应用失败: {e}")
            
        print("程序退出完成")
        
        # 最终强制退出
        os._exit(0)
        


# python sim_main.py --device cpu  --enable_cameras  --task  Isaac-PickPlace-Cylinder-G129-Dex1-Joint    --enable_gripper_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint    --enable_dex3_dds --robot_type g129

# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-G129-Dex1-Joint     --enable_gripper_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-G129-Dex3-Joint    --enable_dex3_dds --robot_type g129

# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-G129-Dex1-Joint     --enable_gripper_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-G129-Dex3-Joint     --enable_dex3_dds --robot_type g129


# python replay_data.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-G129-Dex1-Joint     --enable_gripper_dds --robot_type g129

#  ['/home/unitree/Code/avp_teleoperate_sim/teleop/utils/data/episode_0007/data.json', '/home/unitree/Code/avp_teleoperate_sim/teleop/utils/data/episode_0004/data.json', '/home/unitree/Code/avp_teleoperate_sim/teleop/utils/data/episode_0001/data.json', '/home/unitree/Code/avp_teleoperate_sim/teleop/utils/data/episode_0002/data.json', '/home/unitree/Code/avp_teleoperate_sim/teleop/utils/data/episode_0003/data.json', '/home/unitree/Code/avp_teleoperate_sim/teleop/utils/data/episode_0006/data.json', '/home/unitree/Code/avp_teleoperate_sim/teleop/utils/data/episode_0008/data.json']
