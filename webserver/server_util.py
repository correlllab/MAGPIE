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

async def move_robot_and_record_images(robot, pose, camera, path="robot_logs/move.csv", move_type="linear"):
    # configure move type
    robot.start()
    robot_motion = None
    move_type = move_type.lower()
    if move_type == "linear":
        robot_motion = robot.moveL 
    elif move_type == "cartesian":
        robot_motion = robot.move_tcp_cartesian 
    
    camera.begin_record(filepath=f"{path}")
    loop = asyncio.get_event_loop()
    with ThreadPoolExecutor() as pool:
        await loop.run_in_executor(pool, robot_motion, pose)
        await asyncio.sleep(SLEEP_RATE * 3)
    
    await camera.stop_record()
    # need to stop robot inside async function
    # if not, i could have generic method
    # execute_function_and_record_images. oh well
    robot.stop_recording()
    robot.stop()

async def execute_grasp_and_record_images(code_executor, code, camera, path="robot_logs/move.csv"):
    camera.begin_record(filepath=f"{path}")
    loop = asyncio.get_event_loop()
    with ThreadPoolExecutor() as pool:
        grasp_log = await loop.run_in_executor(pool, code_executor, code)
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

def log_to_df(path="robot_logs/df_combined.csv", timestamp=0, obj=""):
    '''
    combine 3 logs:
    1. move (csv)
    2. grasp_log (json)
    3. home (csv)
    into one dataframe
    '''
    # required format
    # observation: 6x joint velocities, gripper velocity, gripper force
    # camera: 640x480x3 RGB image
    # alternative: depth image too?

    # list of dictionaries to tfds
    # https://stackoverflow.com/questions/68567630/converting-a-list-of-dictionaries-to-a-tf-dataset

    # load csv to pd df
    home = pd.read_csv(f"robot_logs/home_{timestamp}.csv")
    move = pd.read_csv(f"robot_logs/move_{timestamp}.csv")
    
    # extract 6d poses, x, y, z, rx, ry, rz from actual_TCP_pose_0 to actual_TCP_pose_5
    home_tcp = home[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2', 'actual_TCP_pose_3', 'actual_TCP_pose_4', 'actual_TCP_pose_5']]
    move_tcp = move[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2', 'actual_TCP_pose_3', 'actual_TCP_pose_4', 'actual_TCP_pose_5']]

    # extract timestamp columns and actual_qd columns from actual_qd_0 to actual_qd_5
    home = home[['timestamp']]
    move = move[['timestamp']]

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
    
    # assign to home and move dfs
    # home tcp has become a single column df, each cell containing a 6-element list, need to split back into 'actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2', 'actual_TCP_pose_3', 'actual_TCP_pose_4', 'actual_TCP_pose_5'
    home_tcp = pd.DataFrame(home_tcp.tolist(), columns=['x', 'y', 'z', 'rx', 'ry', 'rz'])
    move_tcp = pd.DataFrame(move_tcp.tolist(), columns=['x', 'y', 'z', 'rx', 'ry', 'rz'])

    home[['x', 'y', 'z', 'rx', 'ry', 'rz']] = home_tcp
    move[['x', 'y', 'z', 'rx', 'ry', 'rz']] = move_tcp

    # load json
    grasp_pth = f"robot_logs/grasp_log_{timestamp}.json"
    obj_text = codecs.open(grasp_pth, 'r', encoding='utf-8').read()
    gl = np.array(json.loads(obj_text))
    grasp_log = pd.DataFrame(gl.tolist()) # convert to pd df
    # keep only timestamp, aperture, gripper_vel, and contact_force cols, add actual_qd columns from actual_qd_0 to actual_qd_6 from the last row of move to grasp_log

    grasp_log = grasp_log[['timestamp', 'aperture', 'gripper_vel', 'contact_force', 'applied_force']]
    # grasp_log[['actual_qd_0', 'actual_qd_1', 'actual_qd_2', 'actual_qd_3', 'actual_qd_4', 'actual_qd_5']] = move.iloc[-1][['actual_qd_0', 'actual_qd_1', 'actual_qd_2', 'actual_qd_3', 'actual_qd_4', 'actual_qd_5']]
    grasp_log[['x', 'y', 'z', 'rx', 'ry', 'rz']] = move.iloc[-1][['x', 'y', 'z', 'rx', 'ry', 'rz']]
    grasp_log = grasp_log[['timestamp', 'x', 'y', 'z', 'rx', 'ry', 'rz', 'aperture', 'gripper_vel', 'contact_force', 'applied_force']]
    
    # add columns to move and home dfs, zeroed out
    move['aperture'] = 0.0
    move['gripper_vel'] = 0.0
    move['contact_force'] = 0.0
    move['applied_force'] = 0.0
    home['aperture'] = 0.0
    home['gripper_vel'] = 0.0
    home['contact_force'] = 0.0
    home['applied_force'] = 0.0


    # get initial values, iteratively subtract 0.1s from ti and assign to to move timestamps, from last to first
    ti = gl[0]['timestamp']
    ai = gl[0]['aperture']

    for i in np.arange(len(move)-1, -1, -1):
        # iterate backwards through move timestamps
        ti -= 0.1
        move.loc[i, 'timestamp'] = ti
        move.loc[i, 'aperture'] = ai

    # get last timestamp tf, iteratively add 0.1s to tf and assign to to home timestamps, from first to last
    tf = gl[-1]['timestamp']
    af = gl[-1]['aperture']
    cff = gl[-1]['contact_force']
    aff = gl[-1]['applied_force']

    for i in np.arange(len(home)):
        # iterate forwards through home timestamps
        tf += 0.1
        home.loc[i, 'timestamp'] = tf
        home.loc[i, 'aperture'] = af
        home.loc[i, 'contact_force'] = cff
        home.loc[i, 'applied_force'] = aff

    # combine dataframes vertically
    df = pd.concat([move, grasp_log, home], axis=0)
    # save df to csv
    df.to_csv(f"robot_logs/df_{path}_{obj}.csv", index=False)

    return df

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