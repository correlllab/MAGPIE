import argparse
import json
import h5py
import imageio
import numpy as np
import os
from copy import deepcopy
from datetime import datetime
from functools import partial
import time
from absl import app, flags, logging
import click
import cv2
import torch

### robomimic imports
### for DROID
import robomimic
import robomimic.utils.file_utils as FileUtils
import robomimic.utils.torch_utils as TorchUtils
import robomimic.utils.tensor_utils as TensorUtils
import robomimic.utils.obs_utils as ObsUtils
from robomimic.envs.env_base import EnvBase
from robomimic.algo import RolloutPolicy
import urllib.request

### Octo/Jax imports
import jax
import jax.numpy as jnp
from octo.model.octo_model import OctoModel
from octo.utils.gym_wrappers import HistoryWrapper, TemporalEnsembleWrapper
from octo.utils.train_callbacks import supply_rng

def reset_policy(policy):
    policy.start_episode()
    policy.goal_mode = None
    policy.action_queue = None
    policy.eval_mode = True

def create_policy(ckpt_path, cfg="dp", task=None):
    ckpt_dict = None
    if "dp" in cfg:
        device = TorchUtils.get_torch_device(try_to_use_cuda=True)
        # restore policy
        policy, ckpt_dict = FileUtils.policy_from_checkpoint(ckpt_path=ckpt_path, device=device, verbose=True)
        reset_policy(policy)
        return policy, ckpt_dict
    elif "octo" in cfg:
        if task is None:
            print("No task provided for Octo policy, passing")
            pass
        else:
            model = OctoModel.load_pretrained(ckpt_path)
            pass
        #     policy = supply_rng(
        #         partial(
        #             model.sample_actions,
        #             unnormalization_statistics=model.dataset_statistics["action"],
        #         ),
        #     )
        #     task = model.create_tasks(texts=)
        # return policy, ckpt_dict


def run_action(policy, obs):
    # run policy
    action = policy(obs)
    return action