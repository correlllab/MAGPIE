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
import pandas as pd
from PIL import Image

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

def dp_log(obs, act, obj, cfg, pth=""):
    dir = f"{pth}/cfg-{cfg}_obj-{obj}_{time.time()}"
    wrist_dir = f"{dir}/wrist_img"
    workspace_dir = f"{dir}/workspace_img"
    # make dir if does not exist
    if not os.path.exists(dir):
        os.makedirs(dir)
        os.makedirs(wrist_dir)
        os.makedirs(workspace_dir)
    fn = f"{dir}/obsact.csv"
    history = min(len(obs), len(act))
    ikeys = ["camera/image/varied_camera_1_left_image", 
             "camera/image/varied_camera_2_left_image"]
    okeys = ["robot_state/gripper_position"]
    akeys = ["gripper_position"]
    if "octo" in cfg:
        okeys.append("robot_state/joint_positions")
    if "nf" not in cfg:
        okeys.append("robot_state/applied_force")
        okeys.append("robot_state/contact_force")
        akeys.append("gripper_force")
    keys = okeys + akeys
    data = []
    for i in range(history):
        d = []
        for ok in okeys:
            scale = 100.0 if "contact" not in ok else 1.0
            d.append(np.round(obs[i][ok][0]*scale, 3))
        for ak in akeys:
            d.append(np.round(act[i][ak], 3))
        workspace_img = Image.fromarray(obs[i][ikeys[0]].transpose(1, 2, 0))
        workspace_img.save(f"{workspace_dir}/{i}.jpeg")
        wrist_img = Image.fromarray(obs[i][ikeys[1]].transpose(1, 2, 0))
        wrist_img.save(f"{wrist_dir}/{i}.jpeg")
        data.append(d)
    df = pd.DataFrame(data, columns=keys)
    df.to_csv(fn, index=False)
    return df

def get_observation(sensors={}, obs_queue=[], last_obs={}, cfg="dp"):
    # get observation from sensors. this will be abstracted later
    # for now, we explicitly transform
    first_obs = len(obs_queue) == 0
    obs = {}
    wksp_size = (256, 256) if "octo" in cfg else (128, 128)
    def process_image(image, size=(128, 128), order=(2, 0, 1)):
        # reshape image from 640x480x3 to 3x480x640 (H, W, C) --> (C, H, W)
        image = np.array(Image.fromarray(image).resize(size))
        image = np.transpose(image, order)
        return image
    if "dp" in cfg:
        obs["robot_state/gripper_position"]   = np.array([sensors["gripper"].get_aperture()])/100.0
        if "go" not in cfg:
            obs["robot_state/cartesian_position"] = np.array(sensors["robot"].recv.getActualTCPPose())
        # scale to mm/100 and N/100
        if "nf" not in cfg:
            obs["robot_state/applied_force"]      = np.array([sensors["gripper"].applied_force])/100.0
            obs["robot_state/contact_force"]      = np.array([sensors["gripper"].recorded_contact_force]) # I forgot to scale this in training, so wont scale here xd
                # obs["camera/image/varied_camera_1_left_image"] = process_image(await sensors["workspace_camera"].take_image())
        obs["camera/image/varied_camera_1_left_image"] = process_image(sensors["workspace_camera"].take_image_blocking(), size=wksp_size)
        obs["camera/image/varied_camera_2_left_image"] = process_image(sensors["wrist_camera"].take_image_blocking())

    elif "octo" in cfg:
        joints = sensors["robot"].get_joint_angles()
        tcp = np.array(sensors["robot"].recv.getActualTCPPose())
        gripper_pos = np.array([sensors["gripper"].get_aperture()])/100.0
        applied_force = np.array([sensors["gripper"].applied_force])/100.0
        contact_force = np.array([sensors["gripper"].recorded_contact_force])
        action_blocked = np.array([False])
        obs["image_primary"] = process_image(sensors["workspace_camera"].take_image_blocking(), size=wksp_size, order=(0, 1, 2))
        obs["image_wrist"] = process_image(sensors["wrist_camera"].take_image_blocking(), order=(0, 1, 2))
        obs["timestep_pad_mask"] = False if first_obs else True
        if "ft" in cfg:
            obs["proprio"] = np.concatenate((joints, tcp, gripper_pos, applied_force, contact_force, action_blocked))


    # window=2 so observations with shape (N, ...) become (2, N)
    if first_obs:
        # double the observation
        obs_queue.append({k: np.array([v, v]) for k, v in obs.items()})
    else:
        # take the last_obs and append the new observation to it
        obs_queue.append({k: np.array([last_obs[k], v]) for k, v in obs.items()})


    # create a lang_command.txt if it does not exist and write the observation to it
    # delete lang_command.txt if it exists
    os.remove("eval_params/lang_command.txt") if os.path.exists("eval_params/lang_command.txt") else None
    with open("eval_params/lang_command.txt", "w") as f:
        f.write(sensors["language_instruction"])

    return obs_queue, obs

def parse_dp_action(actions, action_flag="dp"):
    '''
    @param actions: list of actions, containing up [dx, dy, dz, drx, dry, drz, d_aperture, d_force]
    '''
    ad = {}
    actions = np.array(actions)
    print(f"{len(actions)=}")
    scale = 1000 # hack for grasp only
    if "go" not in action_flag:
        scale = 100 # need to re-scale the actions
        ad['rel_pos'] = actions[:3]
        ad['rel_rot'] = actions[3:6] # not gonna use rotation for now
    if "octo" in action_flag and "ft" not in action_flag:
        ad['close_gripper'] = actions[-1]
        return ad

    if "nf" not in action_flag:
        ad['gripper_force'] = max(actions[-1]*100.0, 0)
        ad['gripper_position'] = min(actions[-2]*scale, 0)
    else:
        ad['gripper_position'] = min(actions[-1]*scale, 0)
    
    return ad

def apply_action(actions=[], actuators={}, action_flag="dp", nograsp=False, record_load=False):
    # apply action to actuators
    # actions is a dictionary of action objects
    actions = np.array(actions)
    scale = 1000 # hack for grasp only
    if "go" not in action_flag:
        scale = 100 # need to re-scale the actions
        delta_pos = actions[:3]
        delta_rot = actions[3:6] # not gonna use rotation for now
        actuators["robot"].move_tcp_cartesian_delta(delta_pos, z_offset=0.0)
    
    if nograsp: return

    if "octo" in action_flag and "ft" not in action_flag:
        close_gripper = actions[-1]
        if close_gripper:
            actuators["gripper"].close_gripper()
        else:
            actuators["gripper"].open_gripper()
        return

    curr_aperture = actuators["gripper"].get_aperture()
    if "nf" not in action_flag: # force only or force + position action
        curr_force = actuators["gripper"].applied_force
        print(f"curr_force: {curr_force}")
        actuators["gripper"].set_force(curr_force + max(actions[-1]*100.0, 0))
        print(f"action: {max(actions[-1]*100.0, 0)}")
        print(f"curr_force after set: {actuators['gripper'].applied_force}")
        if "fo" not in action_flag: # not force only
            actuators["gripper"].set_goal_aperture(curr_aperture + min(actions[-2]*scale, 0), record_load=record_load)
        else:
            # try increasing goal aperture by hard coded value rather than only force control
            fo_dx = 3 # force only aperture decrease constant (mm)
            actuators["gripper"].set_goal_aperture(curr_aperture + fo_dx, record_load=record_load)
    else:
        actuators["gripper"].set_goal_aperture(curr_aperture + min(actions[-1]*scale, 0), record_load=record_load)
