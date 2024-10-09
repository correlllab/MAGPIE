import argparse
import json
import h5py
import imageio
import numpy as np
import os
from copy import deepcopy

import torch

import robomimic
import robomimic.utils.file_utils as FileUtils
import robomimic.utils.torch_utils as TorchUtils
import robomimic.utils.tensor_utils as TensorUtils
import robomimic.utils.obs_utils as ObsUtils
from robomimic.envs.env_base import EnvBase
from robomimic.algo import RolloutPolicy
import urllib.request

def reset_policy(policy):
    policy.start_episode()
    policy.goal_mode = None
    policy.action_queue = None
    policy.eval_mode = True


def create_policy(ckpt_path):
    device = TorchUtils.get_torch_device(try_to_use_cuda=True)

    # restore policy
    policy, ckpt_dict = FileUtils.policy_from_checkpoint(ckpt_path=ckpt_path, device=device, verbose=True)
    reset_policy(policy)
    return policy, ckpt_dict

def run_action(policy, obs):
    # run policy
    action = policy(obs)
    return action