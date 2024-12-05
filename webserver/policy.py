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
def reset_policy(policy):
    policy.start_episode()
    policy.goal_mode = None
    policy.action_queue = None
    policy.eval_mode = True

def create_policy(ckpt_path, cfg="dp", task=None):
    ckpt_dict, model = None, None
    if "dp" in cfg:
        device = TorchUtils.get_torch_device(try_to_use_cuda=True)
        # restore policy
        policy, ckpt_dict = FileUtils.policy_from_checkpoint(ckpt_path=ckpt_path, device=device, verbose=True)
        reset_policy(policy)
        return policy, model, ckpt_dict
    elif "octo" in cfg:
        import jax
        import jax.numpy as jnp
        from octo.model.octo_model import OctoModel
        from octo.utils.gym_wrappers import HistoryWrapper, TemporalEnsembleWrapper
        from octo.utils.train_callbacks import supply_rng

        model = OctoModel.load_pretrained(ckpt_path)
        unnorm_stats = None
        if "ft" in cfg:
            unnorm_stats = model.dataset_statistics["action"]
        else:
            unnorm_stats = model.dataset_statistics["berkeley_autolab_ur5"]["action"]
        policy = supply_rng(
            partial(
                model.sample_actions,
                unnormalization_statistics=unnorm_stats,
            ),
        )
        return policy, model, ckpt_dict

def get_action(policy, *args):
    # run policy
    action = policy(*args)
    return action

def tree_map(obs):
    import jax
    return jax.tree_map(lambda x: x[None], obs)