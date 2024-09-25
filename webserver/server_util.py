import ast
import asyncio
import base64
import codecs
from concurrent.futures import ThreadPoolExecutor
import dataclasses
import io
import json
import logging
import numpy as np
import openai
import os
import pandas as pd
from PIL import Image
import platform
import rlds # GDM Reinforcement Learning Dataset
import spacy
import sys
sys.path.append("../")
from magpie import poses
import time
import tensorflow as tf
import tensorflow_datasets as tfds
from typing import Any
sys.path.append("../")
SLEEP_RATE = 0.5

def log_grasp(grasp_log, path="robot_logs/grasp_log.json"):
    # list of dictionaries to json
    # get content after last newline of stdout
    sep = grasp_log.split('\n')
    gl = np.array(ast.literal_eval(sep[-2]))
    # write list of dicts to json
    with open(path, 'w') as f:
        f.write(json.dumps(gl.tolist()))

async def move_robot_and_record_images(robot, pose, cp_dict, index=0, move_type="linear"):
    # configure move type
    print(cp_dict)
    robot.start()
    robot_motion = None
    move_type = move_type.lower()
    if move_type == "linear":
        robot_motion = robot.moveL 
    elif move_type == "cartesian":
        robot_motion = robot.move_tcp_cartesian 
    
    for camera in cp_dict:
        camera.begin_record(filepath=f"{cp_dict[camera]}/{index}_")
    time.sleep(SLEEP_RATE*1)

    loop = asyncio.get_event_loop()
    with ThreadPoolExecutor() as pool:
        await loop.run_in_executor(pool, robot_motion, pose)
        await asyncio.sleep(SLEEP_RATE * 3)
    
    for camera in cp_dict:
        await camera.stop_record()
    # need to stop robot inside async function
    # if not, i could have generic method
    # execute_function_and_record_images. oh well
    robot.stop_recording()
    robot.stop()

async def execute_grasp_and_record_images(code_executor, code, cp_dict, index=0):
    print(cp_dict)
    for camera in cp_dict:
        camera.begin_record(filepath=f"{cp_dict[camera]}/{index}_")
    time.sleep(SLEEP_RATE*1)

    loop = asyncio.get_event_loop()
    with ThreadPoolExecutor() as pool:
        grasp_log = await loop.run_in_executor(pool, code_executor, code)
    for camera in cp_dict:
        await camera.stop_record()
    return grasp_log

def encode_image(pil_img, decoder='ascii'):
    img_io = io.BytesIO()
    pil_img.save(img_io, 'jpeg', quality=100)
    img_io.seek(0)
    img = base64.b64encode(img_io.getvalue()).decode(decoder)
    img_tag = f'<img src="data:image/jpg;base64,{img}" class="img-fluid"/>'
    # return img_tag
    return img

def get_adjective_noun_phrase(token):
    # Get all tokens that depend on the token
    dependents = [child for child in token.children]
    
    # Get adjectives that modify the token
    adjectives = [child.text for child in dependents if child.dep_ == "amod"]
    
    # Get compound nouns (like 'circuit' in 'circuit board')
    compounds = [child.text for child in dependents if child.dep_ == "compound"]
    
    # Combine compounds, adjectives, and the noun
    phrase = " ".join(adjectives + compounds + [token.text])
    
    return phrase

def parse_object_description(user_input):
    # Process the sentence using spaCy
    nlp = spacy.load("en_core_web_sm")
    doc = nlp(user_input)
    query, abbrevq = user_input, user_input
    
    # Extract the adjective and noun
    for token in doc:
        if token.dep_ == "dobj":  # Direct object dependency
            phrase = get_adjective_noun_phrase(token)
            print(f"Adjective-Noun phrase for '{token.text}': {phrase}")
            query = phrase
            abbrevq = token.text
    return query, abbrevq

def log_to_df(path="robot_logs/", timestamp=0, obj=""):
    '''
    combine 3 logs:
    1. move (csv)
    2. grasp_log (json)
    3. home (csv)
    '''
    global IMAGE
    # required format
    # observation: 6x joint velocities, gripper velocity, gripper force
    # camera: 640x480x3 RGB image
    # alternative: depth image too?

    # list of dictionaries to tfds
    # https://stackoverflow.com/questions/68567630/converting-a-list-of-dictionaries-to-a-tf-dataset

    # parse out object from path
    obj = path.split('_id-')[0].split('_')[1:]
    obj = ' '.join(obj)

    # parse out timestamp from path
    if timestamp == 0:
        timestamp = path.split('_')[0].split('/')[-1]
    
    # load csv to pd df
    home = pd.read_csv(f"{path}/home.csv")
    move = pd.read_csv(f"{path}/move.csv")
    
    # extract 6d poses, x, y, z, rx, ry, rz from actual_TCP_pose_0 to actual_TCP_pose_5
    home_tcp = home[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2', 'actual_TCP_pose_3', 'actual_TCP_pose_4', 'actual_TCP_pose_5']]
    move_tcp = move[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2', 'actual_TCP_pose_3', 'actual_TCP_pose_4', 'actual_TCP_pose_5']]
    # calculate delta TCP pose from 'target_TCP_pose_0' to 'target_TCP_pose_5' vs 'actual_TCP_pose_0' to 'actual_TCP_pose_5'
    home_tcp_target = home[['target_TCP_pose_0', 'target_TCP_pose_1', 'target_TCP_pose_2', 'target_TCP_pose_3', 'target_TCP_pose_4', 'target_TCP_pose_5']]
    move_tcp_target = move[['target_TCP_pose_0', 'target_TCP_pose_1', 'target_TCP_pose_2', 'target_TCP_pose_3', 'target_TCP_pose_4', 'target_TCP_pose_5']]
    
    def axis_angle_to_quat(pose):
        # convert 6d pose to matrix
        tmat = poses.pose_vec_to_mtrx(pose)
        # convert rot to roll, pitch, yaw
        quat = poses.rotMatx_2_quat(tmat[:3, :3])
        quat = np.array([quat.scalar, quat.vector[0], quat.vector[1], quat.vector[2]])
        p = np.concatenate((pose[:3], quat))
        return p
    
    # very annoyed. bridge uses rpy/delta rpy
    # autolab uses quat for state, delta rpy for action. freaks!
    def axis_angle_to_rpy(pose):
        # convert 6d pose to matrix
        tmat = poses.pose_vec_to_mtrx(pose)
        # convert rot to roll, pitch, yaw
        rpy = poses.rotation_mtrx_to_rpy(tmat[:3, :3])
        # combine with translation, the first 3 elements of pose
        rpy = np.concatenate((pose[:3], rpy))
        return rpy

    # convert 6d axis-angle pose to 6d rpy pose for home_tcp and move_tcp
    home_tcp = home_tcp.apply(axis_angle_to_rpy, axis=1)
    move_tcp = move_tcp.apply(axis_angle_to_rpy, axis=1)
    home_tcp_target = home_tcp_target.apply(axis_angle_to_rpy, axis=1)
    move_tcp_target = move_tcp_target.apply(axis_angle_to_rpy, axis=1)

    home_tcp_delta = home_tcp_target - home_tcp
    move_tcp_delta = move_tcp_target - move_tcp

    # extract timestamp columns and actual_qd columns from actual_q_0 to actual_q_5
    home = home[['timestamp', 'actual_q_0', 'actual_q_1', 'actual_q_2', 'actual_q_3', 'actual_q_4', 'actual_q_5']]
    move = move[['timestamp', 'actual_q_0', 'actual_q_1', 'actual_q_2', 'actual_q_3', 'actual_q_4', 'actual_q_5']]
    # rename actual_q_0 to actual_q_5 to q0 to q5
    home.rename(columns={'actual_q_0': 'q0', 'actual_q_1': 'q1', 'actual_q_2': 'q2', 'actual_q_3': 'q3', 'actual_q_4': 'q4', 'actual_q_5': 'q5'}, inplace=True)
    move.rename(columns={'actual_q_0': 'q0', 'actual_q_1': 'q1', 'actual_q_2': 'q2', 'actual_q_3': 'q3', 'actual_q_4': 'q4', 'actual_q_5': 'q5'}, inplace=True)
    # home = home[['timestamp']]
    # move = move[['timestamp']]

    
    # assign to home and move dfs
    # home tcp has become a single column df, each cell containing a 6-element list, need to split back into 'actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2', 'actual_TCP_pose_3', 'actual_TCP_pose_4', 'actual_TCP_pose_5'
    home_tcp = pd.DataFrame(home_tcp.tolist(), columns=['x', 'y', 'z', 'rx', 'ry', 'rz'])
    move_tcp = pd.DataFrame(move_tcp.tolist(), columns=['x', 'y', 'z', 'rx', 'ry', 'rz'])
    home_tcp_delta = pd.DataFrame(home_tcp_delta.tolist(), columns=['dx', 'dy', 'dz', 'drx', 'dry', 'drz'])
    move_tcp_delta = pd.DataFrame(move_tcp_delta.tolist(), columns=['dx', 'dy', 'dz', 'drx', 'dry', 'drz'])
    print(home_tcp)
    print(type(home_tcp))

    home[['x', 'y', 'z', 'rx', 'ry', 'rz']] = home_tcp
    move[['x', 'y', 'z', 'rx', 'ry', 'rz']] = move_tcp
    home[['dx', 'dy', 'dz', 'drx', 'dry', 'drz']] = home_tcp_delta
    move[['dx', 'dy', 'dz', 'drx', 'dry', 'drz']] = move_tcp_delta

    # load json
    grasp_pth = f"{path}/grasp.json"
    obj_text = codecs.open(grasp_pth, 'r', encoding='utf-8').read()
    gl = np.array(json.loads(obj_text))
    grasp_log = pd.DataFrame(gl.tolist()) # convert to pd df
    # keep only timestamp, aperture, gripper_vel, and contact_force cols, add actual_qd columns from actual_qd_0 to actual_qd_6 from the last row of move to grasp_log

    gl_copy = grasp_log.copy()
    grasp_log = grasp_log[['timestamp', 'aperture', 'applied_force', 'contact_force']]
    # look one row ahead to get the n+1 aperture and applied_force, subtract current aperture and applied force, and set to d_aperture, d_applied_force
    grasp_log['d_aperture'] = grasp_log['aperture'].diff().shift(-1)
    grasp_log['d_applied_force'] = grasp_log['applied_force'].diff().shift(-1)
    # set the last row's d_aperture and d_applied_force to 0
    grasp_log.iloc[-1, -2:] = 0.0
    grasp_log[['q0', 'q1', 'q2', 'q3', 'q4', 'q5', 'x', 'y', 'z', 'rx', 'ry', 'rz']] = move.iloc[-1][['q0', 'q1', 'q2', 'q3', 'q4', 'q5', 'x', 'y', 'z', 'rx', 'ry', 'rz']]
    grasp_log[['dx', 'dy', 'dz', 'drx', 'dry', 'drz']] = 0.0
    # reorder columns so that it is: timestamp, x, y, z, qx, qy, qz, qw, dx, dy, dz, dqx, dqy, dqz, dqw, aperture, d_aperture, applied_force, d_applied_force
    grasp_log = grasp_log[['timestamp', 'q0', 'q1', 'q2', 'q3', 'q4', 'q5', 'x', 'y', 'z', 'rx', 'ry', 'rz', 'dx', 'dy', 'dz', 'drx', 'dry', 'drz', 'aperture', 'd_aperture', 'applied_force', 'd_applied_force', 'contact_force']]
    
    # add columns to move and home dfs
    move['aperture'] = grasp_log['aperture'].iloc[0]
    move['d_aperture'] = 0.0
    move['contact_force'] = 0.0
    move['applied_force'] = 0.0
    move['d_applied_force'] = 0.0
    home['aperture'] = grasp_log['aperture'].iloc[-1]
    home['d_aperture'] = 0.0
    home['contact_force'] = grasp_log['contact_force'].iloc[-1]
    home['applied_force'] = grasp_log['applied_force'].iloc[-1]
    home['d_applied_force'] = 0.0

    # add task column to move, grasp_log, and home
    move_task = f"moving to grasp {obj}"
    grasp_task = f"grasping {obj}"
    home_task = f"moving back to original position with {obj}"
    task = f"grasping {obj} and returning to original position"
    move['subtask'] = move_task
    move['task'] = task
    grasp_log['subtask'] = grasp_task
    grasp_log['task'] = task
    home['subtask'] = home_task
    home['task'] = task

    def extract_timestamp(filename):
        # extracting the float
        img = filename.split('_')[1].split('.')[:2]
        return float(f"{img[0]}.{img[1]}")

    # # get initial values, iteratively subtract 0.1s from last_move and last_home and assign to to move and home timestamps, from last to first
    move_workspace_img = sorted([extract_timestamp(f) for f in os.listdir(f"{path}/workspace_img") if f.startswith("0_")])
    move_wrist_img = sorted([extract_timestamp(f) for f in os.listdir(f"{path}/wrist_img") if f.startswith("0_")])
    last_move_workspace_img = move_workspace_img[-1]
    last_move_wrist_img = move_wrist_img[-1]
    last_move = max(last_move_workspace_img, last_move_wrist_img)

    home_workspace_img = sorted([extract_timestamp(f) for f in os.listdir(f"{path}/workspace_img") if f.startswith("2_")])
    home_wrist_img = sorted([extract_timestamp(f) for f in os.listdir(f"{path}/wrist_img") if f.startswith("2_")])
    last_home_workspace_img = home_workspace_img[-1]
    last_home_wrist_img = home_wrist_img[-1]
    last_home = max(last_home_workspace_img, last_home_wrist_img)

    for i in np.arange(len(move)-1, -1, -1):
        # iterate backwards through move timestamps
        move.loc[i, 'timestamp'] = last_move
        last_move -= 0.096
    
    for i in np.arange(len(home)-1, -1, -1):
        # iterate backwards through home timestamps
        home.loc[i, 'timestamp'] = last_home
        last_home -= 0.096


    grasp_workspace_img = sorted([extract_timestamp(f) for f in os.listdir(f"{path}/workspace_img") if f.startswith("1_")])
    grasp_wrist_img = sorted([extract_timestamp(f) for f in os.listdir(f"{path}/wrist_img") if f.startswith("1_")])
    
    def timestamp_to_filename(timestamp, index):
        # converting the float to a string
        return f"{index}_{str(timestamp)}.jpeg"

    move_workspace_interpolated = []
    grasp_workspace_interpolated = []
    home_workspace_interpolated = []
    move_wrist_interpolated = []
    grasp_wrist_interpolated = []
    home_wrist_interpolated = []
    # go thropugh each timestamp in move, grasp, and home
    # find the closest img timestamp and append to move_interpolated, grasp_interpolated, and home_interpolated
    for m in move['timestamp']:
        move_workspace_interpolated.append(timestamp_to_filename(min(move_workspace_img, key=lambda x:abs(x-m)), 0))
        move_wrist_interpolated.append(timestamp_to_filename(min(move_wrist_img, key=lambda x:abs(x-m)), 0))
    for g in grasp_log['timestamp']:
        grasp_workspace_interpolated.append(timestamp_to_filename(min(grasp_workspace_img, key=lambda x:abs(x-g)), 1))
        grasp_wrist_interpolated.append(timestamp_to_filename(min(grasp_wrist_img, key=lambda x:abs(x-g)), 1))
    for h in home['timestamp']:
        home_workspace_interpolated.append(timestamp_to_filename(min(home_workspace_img, key=lambda x:abs(x-h)), 2))
        home_wrist_interpolated.append(timestamp_to_filename(min(home_wrist_img, key=lambda x:abs(x-h)), 2))

    # make new columns in dataframes
    move['img'] = move_workspace_interpolated
    move['wrist_img'] = move_wrist_interpolated
    grasp_log['img'] = grasp_workspace_interpolated
    grasp_log['wrist_img'] = grasp_wrist_interpolated
    home['img'] = home_workspace_interpolated
    home['wrist_img'] = home_wrist_interpolated

    # load actual image, not just the filename, cast from jpeg to numpy array
    move['img'] = [np.array(Image.open(f"{path}/workspace_img/{i}")) for i in move['img']]
    move['wrist_img'] = [np.array(Image.open(f"{path}/wrist_img/{i}")) for i in move['wrist_img']]
    grasp_log['img'] = [np.array(Image.open(f"{path}/workspace_img/{i}")) for i in grasp_log['img']]
    grasp_log['wrist_img'] = [np.array(Image.open(f"{path}/wrist_img/{i}")) for i in grasp_log['wrist_img']]
    home['img'] = [np.array(Image.open(f"{path}/workspace_img/{i}")) for i in home['img']]
    home['wrist_img'] = [np.array(Image.open(f"{path}/wrist_img/{i}")) for i in home['wrist_img']]
    
    # write move, grasp_log, and home to csv files
    # move.to_csv(f"{path}/0.csv", index=False)
    # grasp_log.to_csv(f"{path}/1.csv", index=False)
    # home.to_csv(f"{path}/2.csv", index=False)

    # # combine dataframes vertically
    df = pd.concat([move, grasp_log, home], axis=0)
    # # round all float columns to 6 decimal places
    df = df.round(6)
    # # save df to csv
    # df.to_csv(f"{path}/trajectory.csv", index=False)

    # return df as pickled np array
    df_pkl = df.to_numpy()
    np.save(f"{path}/episode_{obj}_{timestamp}.npy", df_pkl)

    return move, grasp_log, home

def df_to_rlds(df, img, language_instruction="", path="robot_logs/episode.tfds", obj=""):
    episode_steps = df.to_dict(orient='records')

    # # make images array with length == len(steps) that is img at index 0 and a blank image the rest
    # images = [img] + [np.zeros_like(img) for _ in range(len(episode_steps)-1)]

    # Convert list of dictionaries to RLDS episode
    # rlds: https://github.com/google-research/rlds?tab=readme-ov-file#load-with-tfds
    episode = {
        'steps':[{
            'timestamp': step['timestamp'],
            'action': [
                step['x'],
                step['y'],
                step['z'],
                step['rx'],
                step['ry'],
                step['rz'],
                step['aperture'],
                step['gripper_vel'],
                step['contact_force'],
                step['applied_force'],
                ],
            'observation': {
                'state': [
                step['x'],
                step['y'],
                step['z'],
                step['rx'],
                step['ry'],
                step['rz'],
                step['aperture'],
                step['gripper_vel'],
                step['contact_force'],
                step['applied_force'],
                ],
                'image': np.array(img)
            },
            'language_instruction': language_instruction,
            'reward': 0,  # Placeholder, assuming no reward data is available
            'is_terminal': False,  # Placeholder, assuming steps are not terminal
            'is_first': False,  # Placeholder, assuming steps are not terminal
            'is_last': False  # Placeholder, assuming steps are not terminal
        } for step in episode_steps]
        # } for step, i in zip(episode_steps, images)]
    }

    # Convert the episode to a list of tuples
    steps = [
        (
            step['timestamp'],     # timestep
            step['action'],       # action
            step['observation'],  # observation
            step['language_instruction'],  # language_instruction
            step['reward'],       # reward
            step['is_terminal'],   # is_terminal
            step['is_first'],   # is_terminal
            step['is_last']   # is_terminal
        )
        for step in episode['steps']
    ]

    # Generator function
    def generator():
        for step in steps:
            yield step

    # Define the dataset types and shapes
    output_types = (
        tf.float32,  # timestamp,
        tf.float32,  # action
        {'state': tf.float32, 'image': tf.uint8},  # observation
        tf.string,   # language_instruction
        tf.float32,  # reward
        tf.bool,     # is_terminal
        tf.bool,     # is_first
        tf.bool      # is_last
    )

    output_shapes = (
        tf.TensorShape([]),  # timestamp
        tf.TensorShape([10]),  # action
        {'state': tf.TensorShape([10]), 'image': tf.TensorShape([480, 640, 3])},  # observation
        tf.TensorShape([]),   # language_instruction
        tf.TensorShape([]),   # reward
        tf.TensorShape([]),   # is_terminal
        tf.TensorShape([]),   # is_first
        tf.TensorShape([])   # is_last
    )

    # Create the tf.data.Dataset
    dataset = tf.data.Dataset.from_generator(
        generator,
        output_types=output_types,
        output_shapes=output_shapes
    )

    # save tfds as file
    tf.data.Dataset.save(dataset, f'robot_logs/episode_{path}_{obj}_FIX')
    episode = rlds.build_episode(tfds.as_numpy(dataset), metadata=output_types[0])
    # rlds.save_as_tfds(dataset, f'robot_logs/{path}.tfds')

    return episode, dataset