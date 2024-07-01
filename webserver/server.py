import ast
import base64
import codecs
import dataclasses
from flask import Flask, render_template, request, jsonify
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
import sys
import time
import tensorflow as tf
import tensorflow_datasets as tfds
from typing import Any
sys.path.append("../")

# LLM
from magpie.prompt_planner.prompts import mp_prompt_thinker_coder_muk as mptc
from magpie.prompt_planner.prompts import mp_prompt_tc_vision as mptcv
from magpie.prompt_planner.prompts import mp_prompt_tc_vision_phys as mptcvp
from magpie.prompt_planner.prompts import mp_prompt_tc_phys as mptcp
from magpie.prompt_planner import conversation
from magpie.prompt_planner import confirmation_safe_executor
from magpie.prompt_planner import task_configs

# Perception
from magpie.perception.label import Label
from magpie.perception.label_owlvit import LabelOWLViT
# from magpie.perception.mask_sam import MaskSAM

on_robot = platform.system() == "Linux"
if on_robot:
    sys.path.append("../")
    from magpie.gripper import Gripper
    from magpie import grasp as gt # funky gripper-ur5 utilities, need to stop using/refactor
    from magpie import ur5 as ur5
    import magpie.realsense_wrapper as real
    from magpie.perception import pcd
    from magpie.prompt_planner.prompts import mp_prompt_tc_vision as mptc

import simulated_romi_prompt as srp
app = Flask(__name__)

# metadata / logging
CONFIG = {"move": "3D Pos", "grasp": "dg", "llm": "gpt-4-turbo", "vlm": "owl-vit"}
INTERACTIONS = 0
MESSAGE_LOG = {}
CONNECTED = False

# Perception Configuration
CAMERA = None
label_models = {'owl-vit': "google/owlvit-base-patch32"}
LABEL = None
APERTURE = None
IMAGE = None

# LLM Configuration
API_KEY = os.environ['CORRELL_API_KEY']
MODEL = None
TASK_CONFIG = task_configs.ALL_TASKS["magpie"]
PROMPT_TYPE = 'thinker_coder'
openai.api_key = API_KEY
safe_executor = confirmation_safe_executor.ConfirmationSafeExecutor(
    skip_confirmation=True,
    local_execute=True)
PROMPT = TASK_CONFIG.prompts[PROMPT_TYPE]
VISION = False
RESPONSE = None
PROMPT_MODEL = None
CONVERSATION = None

# hardware
SERVO_PORT = "/dev/ttyACM0"
GRIPPER = None
ROBOT_IP = "192.168.0.4"
HOME_POSE = None
SLEEP_RATE = 0.5
GOAL_POSE = None
AT_GOAL = False

def encode_image(pil_img, decoder='ascii'):
    img_io = io.BytesIO()
    pil_img.save(img_io, 'jpeg', quality=100)
    img_io.seek(0)
    img = base64.b64encode(img_io.getvalue()).decode(decoder)
    img_tag = f'<img src="data:image/jpg;base64,{img}" class="img-fluid"/>'
    # return img_tag
    return img

def parse_object_description(user_input):
    return user_input, user_input

def handle_prompt_name(policy):
    global VISION
    global on_robot
    prompt_stub = "thinker_coder"
    # stupid, but order matters
    if 'vision' in policy:
        if on_robot:
            prompt_stub += "_vision"
            VISION = True
        else:
            print("Vision policy selected but not on robot. No vision policy will be used.")
    if 'cot' in policy:
        prompt_stub += "_phys"
    print(f"Prompt policy: {prompt_stub}")
    return prompt_stub

def log_grasp(grasp_log):
    path = "robot_logs/grasp_log.json"
    # list of dictionaries to json
    # get content after last newline of stdout
    sep = grasp_log.split('\n')
    gl = np.array(ast.literal_eval(sep[-2]))
    # write list of dicts to json
    with open(path, 'w') as f:
        json.dump(gl, f)

def log_to_df():
    '''
    combine 3 logs:
    1. move (csv)
    2. grasp_log (json)
    3. home (csv)
    into one tfds in rlds format
    rlds: https://github.com/google-research/rlds?tab=readme-ov-file#load-with-tfds
    '''
    global IMAGE
    # required format
    # observation: 6x joint velocities, gripper velocity, gripper force
    # camera: 640x480x3 RGB image
    # alternative: depth image too?

    # list of dictionaries to tfds
    # https://stackoverflow.com/questions/68567630/converting-a-list-of-dictionaries-to-a-tf-dataset

    # load csv to pd df
    home = pd.read_csv("robot_logs/home.csv")
    move = pd.read_csv("robot_logs/move.csv")

    # extract timestamp columns and actual_qd columns from actual_qd_0 to actual_qd_5
    home = home[['timestamp', 'actual_qd_0', 'actual_qd_1', 'actual_qd_2', 'actual_qd_3', 'actual_qd_4', 'actual_qd_5']]
    move = move[['timestamp', 'actual_qd_0', 'actual_qd_1', 'actual_qd_2', 'actual_qd_3', 'actual_qd_4', 'actual_qd_5']]

    # load json
    path = "robot_logs/grasp_log.json"
    obj_text = codecs.open(path, 'r', encoding='utf-8').read()
    gl = np.array(json.loads(obj_text))
    grasp_log = pd.DataFrame(gl.tolist()) # convert to pd df
    # keep only timestamp, aperture, gripper_vel, and contact_force cols, add actual_qd columns from actual_qd_0 to actual_qd_6 from the last row of move to grasp_log

    grasp_log = (grasp_log[['timestamp']] + 
                 move.iloc[-1][['actual_qd_0', 'actual_qd_1', 'actual_qd_2', 'actual_qd_3', 'actual_qd_4', 'actual_qd_5']] +
                 grasp_log[['aperture', 'gripper_vel', 'contact_force']])
    
    # add columns to move and home dfs, zeroed out
    move['aperture'] = 0
    move['gripper_vel'] = 0
    move['contact_force'] = 0
    home['aperture'] = 0
    home['gripper_vel'] = 0
    home['contact_force'] = 0


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
    cf = gl[-1]['contact_force']

    for i in np.arange(len(home)):
        # iterate forwards through home timestamps
        tf += 0.1
        home.loc[i, 'timestamp'] = tf
        home.loc[i, 'aperture'] = af
        home.loc[i, 'contact_force'] = cf

    # combine dataframes vertically
    df = pd.concat([move, grasp_log, home], axis=0)
    # save df to csv
    df.to_csv("robot_logs/combined.csv", index=False)

    return df

def df_to_rlds(df):
    episode_steps = df.to_dict(orient='records')

    # Convert list of dictionaries to RLDS episode
    episode = {
        'steps':[{
            'observation': {
                'timestamp': step['timestamp'],
                'actual_qd': [
                    step['actual_qd_0'],
                    step['actual_qd_1'],
                    step['actual_qd_2'],
                    step['actual_qd_3'],
                    step['actual_qd_4'],
                    step['actual_qd_5']
                ],
                'aperture': step['aperture'],
                'gripper_vel': step['gripper_vel'],
                'contact_force': step['contact_force'],
                'image': np.zeros([64, 64, 3])
            },
            'action': 1,  # Placeholder, assuming no action data is available
            'reward': 0,  # Placeholder, assuming no reward data is available
            'is_terminal': False  # Placeholder, assuming steps are not terminal
        } for step in episode_steps]
    }

    # Convert the episode to a list of tuples
    steps = [
        (
            step['observation'],  # observation
            step['action'],       # action
            step['reward'],       # reward
            step['is_terminal']   # is_terminal
        )
        for step in episode['steps']
    ]

    # Generator function
    def generator():
        for step in steps:
            yield step

    # Define the dataset types and shapes
    output_types = (
        {
            'timestamp': tf.float32,
            'actual_qd': tf.float32,
            'aperture': tf.float32,
            'gripper_vel': tf.float32,
            'contact_force': tf.float32,
            'image': tf.uint8
        },
        tf.float32,  # action
        tf.float32,  # reward
        tf.bool      # is_terminal
    )

    output_shapes = (
        {
            'timestamp': tf.TensorShape([]),
            'actual_qd': tf.TensorShape([6]),
            'aperture': tf.TensorShape([]),
            'gripper_vel': tf.TensorShape([]),
            'contact_force': tf.TensorShape([]),
            'image': tf.TensorShape([64, 64, 3])
        },
        tf.TensorShape([]),  # action
        tf.TensorShape([]),  # reward
        tf.TensorShape([])   # is_terminal
    )

    # Create the tf.data.Dataset
    dataset = tf.data.Dataset.from_generator(
        generator,
        output_types=output_types,
        output_shapes=output_shapes
    )

    # save tfds as file
    rlds.save_as_tfds(dataset, 'robot_logs/episode.tfds')

    return dataset

@app.route("/", methods=["GET", "POST"])
def index():
    global prompt
    global completion
    # return render_template("index.html", prompt=prompt, completion=completion)
    return render_template("index.html")

@app.route("/connect", methods=["POST"])
def connect():
    global CONFIG
    global CONNECTED
    # Robot
    global GRIPPER
    global SERVO_PORT
    global HOME_POSE
    global SLEEP_RATE
    global on_robot
    # Perception
    global CAMERA
    global LABEL
    # LLM
    global MODEL
    global TASK_CONFIG
    global PROMPT_MODEL
    global PROMPT
    global CONVERSATION
    global safe_executor
    global VISION

    new_conf = request.get_json()
    print(new_conf['policyconf'])
    CONFIG["move"] = new_conf["moveconf"]
    CONFIG["grasp"] = new_conf["graspconf"]
    CONFIG["policy"] = new_conf["policyconf"]
    CONFIG["llm"] = new_conf["llmconf"]
    CONFIG["vlm"] = new_conf["vlmconf"]
    print(CONFIG)
    MODEL = CONFIG["llm"]
    # MODEL = "gpt-3.5-turbo"

    connect_msg = ""
    # LLM Configuration
    try:
        if CONFIG["grasp"] == "dg":
            prompt_name = handle_prompt_name(CONFIG["policy"])
            PROMPT = TASK_CONFIG.prompts[prompt_name]
        PROMPT_MODEL = PROMPT(
            None, executor=safe_executor
        )
        CONVERSATION = conversation.Conversation(PROMPT_MODEL, MODEL, vision_model=VISION)
        connect_msg += "Created conversation agent.\n"
        CONNECTED = True
    except Exception as e:
        print(e)
        connect_msg += f"Failed to create conversation agent: {e}.\n"
        return jsonify({"CONFIG": CONFIG, "connected": False, "message": connect_msg})

    # Hardware Configuration
    try:
        if on_robot:
            robot = ur5.UR5_Interface(ROBOT_IP)
            GRIPPER = Gripper(SERVO_PORT)
            # GRIPPER.reset_parameters()
            robot.start()
            HOME_POSE = robot.getPose()
            time.sleep(SLEEP_RATE)
            robot.stop()
            # TODO: log home pose
            connect_msg += "Connected to robot arm and gripper.\n"
    except Exception as e:
        CONNECTED = False
        print(e)
        connect_msg += f"Failed to connect to robot and gripper: {e}.\n"
        return jsonify({"CONFIG": CONFIG, "connected": False, "message": connect_msg})

    # Camera Configuration
    try:
        if on_robot:
            rsc = real.RealSense()
            rsc.initConnection()
            CAMERA = rsc
        # LABEL = Label(label_models[CONFIG['vlm']])
        LABEL = LabelOWLViT(pth=label_models['owl-vit'])
        connect_msg += "Connected to camera and perception models.\n"
    except Exception as e:
        CONNECTED = False
        print(e)
        connect_msg += f"Failed to connect to camera or perception models: {e}.\n"
        return jsonify({"CONFIG": CONFIG, "connected": False, "message": connect_msg})
    
    return jsonify({"CONFIG": CONFIG, "connected": True, "message": connect_msg})

@app.route("/chat", methods=["POST"])
def chat():
    global INTERACTIONS
    global MESSAGE_LOG
    global CONFIG
    # Perception
    global CAMERA
    global LABEL
    global GOAL_POSE
    global APERTURE
    global IMAGE
    # LLM
    global PROMPT_MODEL
    global CONVERSATION
    global RESPONSE
    global VISION

    user_command = request.get_json()['message']
    # INPUT PARSING
    MESSAGE_LOG[INTERACTIONS] = [{"type": "text", "role": "user", "content": user_command}]


    # PERCEPTION
    # TODO: segment object description from user input
    enc_img = None
    try:
        p, rgbd_image = CAMERA.getPCD()
        image = np.array(rgbd_image.color)
        # scale image to 640 x 480
        IMAGE = Image.fromarray(image).resize((640, 480))
        queries, abbrevq = parse_object_description(user_command)
        bboxes, _ = LABEL.label(image, queries, abbrevq, topk=True, plot=False)
        preds_plot = LABEL.preds_plot
        enc_img = encode_image(Image.fromarray(preds_plot))
        index = 0 # TODO: make this user selection, for now take highest confidence
        _, ptcld, GOAL_POSE, _ = pcd.get_segment(LABEL.sorted_labeled_boxes_coords, 
                                         index, 
                                         rgbd_image, 
                                         CAMERA, 
                                         type="box-dbscan", 
                                        #  type="box", 
                                        #  method="quat", 
                                         method="iterative", 
                                         display=False)
        APERTURE = pcd.get_minimum_width(ptcld)/1000.0 # convert to meters
    except Exception as e:
        print(e)
        msg = "Perception failed. Is the camera connected? " + str(e) + "\n"
        err_msg = {"type": "text", "role": "system", "content": msg}
        return(jsonify(messages=[err_msg]))

    # print(conv._message_queues)
    img_path = "static/favicon.jpg"
    # enc_img = encode_image(Image.open(f'{img_path}'))

    messages = [
        {"type": "text",  "role": "llm", "content": "This is a text message."},
        {"type": "image", "role": "vlm", "content": enc_img},
    ]
    # add user input to front of messages
    MESSAGE_LOG[INTERACTIONS] += messages
    # print(MESSAGE_LOG)
    return jsonify(messages=messages)

@app.route("/new_interaction", methods=["POST"])
def new_interaction():
    global INTERACTIONS
    global MESSAGE_LOG
    global CONFIG

    # save MESSAGE_LOG to json
    messages = {"messages": MESSAGE_LOG[INTERACTIONS], "config": CONFIG}
    print(messages)
    # get YYYY_MM_DD_HH_MM_SS
    t = time.localtime()
    timestamp = time.strftime('%Y_%m_%d_%H_%M_%S', t)
    with open(f'interaction_logs/{timestamp}_message_log_id-{INTERACTIONS}.json', 'w') as f:
        f.write(json.dumps(messages))

    # save robot log to rlds
    # assert that grasp_log.json, move.csv, and home.csv exist in robot/logs
    path = "robot_logs/grasp_log.json"
    if not os.path.exists(path):
        return jsonify({"success": False, "message": "No grasp log found."})
    if not os.path.exists("robot_logs/move.csv"):
        return jsonify({"success": False, "message": "No move log found."})
    if not os.path.exists("robot_logs/home.csv"):
        return jsonify({"success": False, "message": "No home log found."})
    df = log_to_df()
    dataset = df_to_rlds(df)
    
    INTERACTIONS += 1
    return jsonify({"success": True, "message": "New interaction started."})

@app.route("/grasp_policy", methods=["POST"])
def grasp_policy():
    global PROMPT_MODEL
    global CONVERSATION
    global RESPONSE
    global IMAGE
    global VISION
    global MESSAGE_LOG
    global INTERACTIONS

    user_command = request.get_json()['message']
    conv = CONVERSATION
    pm = PROMPT_MODEL
    desc, code = None, None

    if CONFIG["grasp"] == "dg" or VISION:
        try:
            if VISION:
                b64img = encode_image(IMAGE, decoder='utf-8')
                RESPONSE = conv.send_command(user_command, b64img, vision=VISION)
            else:
                RESPONSE = conv.send_command(user_command)
            desc = conv._llm_responses[0]
            code = RESPONSE
        except Exception as e:
            msg = "Planning failed. Is the system connected? " + str(e) + "\n"
            err_msg = {"type": "text", "role": "system", "content": msg}
            return(jsonify(messages=[err_msg]))
    messages = [
        {"type": "code",  "role": "llm", "content": desc},
        {"type": "code",  "role": "llm", "content": code}
    ]

    MESSAGE_LOG[INTERACTIONS] += messages
    return jsonify(messages=messages)

@app.route("/execute", methods=["POST"])
def execute():
    global PROMPT_MODEL
    global RESPONSE
    global MESSAGE_LOG
    global INTERACTIONS

    pm = PROMPT_MODEL
    if RESPONSE is None:
        return jsonify(messages=[{"type": "text", "role": "system", "success": False, "content": "No response to execute."}])
    try:
        print("SERVER executing code")
        stdout = pm.code_executor(RESPONSE)
        grasp_log = stdout.split('\n')[-2] # hardcoded...very brittle...
        print(f"SERVER executed code with output {stdout}")
        msg = [{"type": "text", "role": "grasp", "content": f"{stdout}"}]
        # return jsonify({"success": True, "message": "Code executed successfully."})
        MESSAGE_LOG[INTERACTIONS] += msg
        print(MESSAGE_LOG[INTERACTIONS])
        return jsonify(messages=[msg])
    except Exception as e:  # pylint: disable=broad-exception-caught
        msg = "Execution failed. Is the robot connected? " + str(e) + "\n"
        err_msg = {"type": "text", "role": "system", "content": msg}
        return(jsonify(messages=[err_msg]))

@app.route("/home", methods=["POST"])
def home():
    global HOME_POSE
    global AT_GOAL
    global GRIPPER
    msg = {"operation": "home", "success": False, "message": f"moving to home pose {HOME_POSE}"}
    try:
        if on_robot:
            robot = ur5.UR5_Interface(ROBOT_IP, freq=10, record=True, record_path="robot_logs/home.csv") # 10Hz frequency
            robot.start()
            robot.moveL(HOME_POSE)
            time.sleep(SLEEP_RATE * 3)
            robot.stop_recording()
            AT_GOAL = False
            robot.stop()
            msg["success"] = True
    except Exception as e:
        print(e)
    return jsonify(msg)

@app.route("/move", methods=["POST"])
def move():
    global HOME_POSE
    global GOAL_POSE
    global APERTURE
    global CONFIG
    global AT_GOAL
    msg = {"operation": "move", "success": False, "message": "moving to goal pose"}
    if GOAL_POSE is None:
        msg["message"] = "No goal pose set."
        return jsonify(msg)
    if AT_GOAL:
        msg["message"] = "Already at goal pose."
        return jsonify(msg)
    try:
        if on_robot:
            robot = ur5.UR5_Interface(ROBOT_IP, freq=10, record=True, record_path="robot_logs/move.csv") # 10Hz frequency
            robot.start()
            # current_pose = robot.getPose()
            # desired_pose = np.array(current_pose) @ np.array(GOAL_POSE)
            # robot.moveL(GOAL_POSE)
            # z = 0.05 + APERTURE # TODO: figure this out
            z = 0.12
            print(f"z_offset: {z}")
            gt.move_to_L(np.array(GOAL_POSE)[:3, 3], robot, z_offset=z)
            time.sleep(SLEEP_RATE * 3)
            AT_GOAL = True
            robot.stop_recording()
            robot.stop()
            msg["success"] = True
        return jsonify(msg)
    except Exception as e:
        print(e)
        msg["message"] = f"Failed to move to goal pose: {e}"
        return jsonify(msg)

@app.route("/release", methods=["POST"])
def release():
    global GRIPPER
    msg = {"operation": "release", "success": False}
    try:
        if on_robot:
            # GRIPPER.release()
            # GRIPPER.open_gripper()
            GRIPPER.reset_parameters()
            msg["success"] = True
    except Exception as e:
        print(e)
    return jsonify(msg)

@app.route("/grasp", methods=["POST"])
def grasp():
    global GRIPPER

    msg = {"operation": "grasp", "success": False}
    try:
        if on_robot:
            # GRIPPER.grasp()
            GRIPPER.close_gripper()
            msg["success"] = True
    except Exception as e:
        print(e)

    return jsonify(msg)

if __name__ == "__main__":
    app.run(debug=True)
