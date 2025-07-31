from action_provider.action_provider_dds import DDSActionProvider
from action_provider.action_provider_trajectory import TrajectoryActionProvider,create_trajectory_generator
from action_provider.action_provider_policy import PolicyActionProvider
from action_provider.action_provider_replay import FileActionProviderReplay
from action_provider.action_provider_wholebody_dds import DDSWholebodyActionProvider
from pathlib import Path


def create_action_provider(env,args):
    """create action provider based on parameters"""
    if args.action_source == "dds":
        return DDSActionProvider(
            env=env,
            args_cli=args
        )
    elif args.action_source == "dds_wholebody":
        return DDSWholebodyActionProvider(
            env=env,
            args_cli=args
        )
    elif args.action_source == "trajectory":
        trajectory_gen = create_trajectory_generator()
        return TrajectoryActionProvider(trajectory_gen)
    
    elif args.action_source == "policy":
        # here can load the trained policy model
        print("policy mode not implemented")
        return None
    elif args.action_source == "replay":
        return FileActionProviderReplay(env=env,args_cli=args)
    else:
        print(f"unknown action source: {args.action_source}")
        return None