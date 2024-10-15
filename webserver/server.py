# general package imports
import ast
import asyncio
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
import server_util as su
from server_util import encode_image, parse_object_description, log_grasp, log_to_df, df_to_rlds, move_robot_and_record_images
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
from magpie.perception.label_owlv2  import LabelOWLv2
from magpie.perception.label_dino   import LabelDINO
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

app = Flask(__name__)

# metadata / logging
CONFIG = {"move": "3D Pos", "grasp": "dg", "llm": "gpt-4-turbo", "vlm": "owl-vit", "vla": "dp"}
INTERACTIONS = 0
MESSAGE_LOG = {}
CONNECTED = False
# LOG_DIR = "../datasets/trajectories"
LOG_DIR = "/home/will/MAGPIE/datasets/trajectories"
GRASP_TIMESTAMP = 0
GRASP_LOG_DIR = ""

# Perception Configuration
CAMERA_SERIAL_INFO = None
CAMERA_PATH_DICT = None
WRIST_CAMERA = None
WORKSPACE_CAMERA = None
CAMERA = None
label_models = {'owl-vit': "google/owlvit-base-patch32",
                'owl-v2':'google/owlv2-base-patch16-ensemble',
                'dino': 'IDEA-Research/grounding-dino-tiny'}
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
OBJECT_NAME = None
KEEP_POLICY_FLAG = False

# VLA Mode
import policy as pol
VLA_MODE = False
VLA_ROBOT = None
ROBOT_TOGGLE = False
ACTED = False
POLICY = None
# by default we will run receding horizon control, i.e. act once, observe
# this cannot do anything yet because DROID:DiffusionPolicy 
# is hardcoded to T_a=1 in eval_mode
T_a = 1
LAST_OBS  = {}
ACT = []
OBS_QUEUE = []
LAST_OBS_QUEUE = []
ACT_QUEUE = []
ACT_DICT_QUEUE = []
# I should set up a config file for this
hmpth = "/home/will/workspace/dp_models"
MODEL_CKPT_DICT = {
    "dp": f"{hmpth}/DG.PTH",
    "dp_nf": f"{hmpth}/DGNF.PTH",
    "dp_go": f"{hmpth}/DGGO.PTH",
    "dp_go_nf": f"{hmpth}/DGGONF.PTH",
    "octo_ft": "octo_ft_ckpt",
    "octo_ft_nf": "octo_ft_nf_ckpt",
}
RECORD_LOAD = False

# hardware
SERVO_PORT = "/dev/ttyACM0"
GRIPPER = None
ROBOT_IP = "192.168.0.4"
HOME_POSE = None
SLEEP_RATE = 0.5
GOAL_POSE = None
AT_GOAL = False
UR5_TEACH_MODE = False

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


@app.route("/teach_mode", methods=["GET", "POST"])
def teach_mode():
    global UR5_TEACH_MODE, ROBOT_IP
    try:
        if VLA_ROBOT is None:
            robot = ur5.UR5_Interface(ROBOT_IP)
            robot.start()
        else: robot = VLA_ROBOT
        if not UR5_TEACH_MODE:
            robot.ctrl.teachMode()
        else:
            robot.ctrl.endTeachMode()
            if VLA_ROBOT is None: robot.stop()
        UR5_TEACH_MODE = not UR5_TEACH_MODE
        return jsonify({"success": True, "message": f"UR5 teach mode enabled: {UR5_TEACH_MODE}."})
    except Exception as e:
        return jsonify({"success": False, "message": f"Failed to toggle UR5 teach mode: {e}."})

@app.route("/vla_robot_toggle", methods=["GET", "POST"])
def vla_robot_toggle():
    global VLA_MODE, VLA_ROBOT, ROBOT_TOGGLE, ROBOT_IP
    if not VLA_MODE:
        return(jsonify({"success": False, "message": "VLA mode not enabled"}))
    if ROBOT_TOGGLE:
        VLA_ROBOT.stop()
        time.sleep(0.1)
        ROBOT_TOGGLE = False
        return(jsonify({"success": True, "message": "Robot stopped."}))
    else:
        VLA_ROBOT = ur5.UR5_Interface(ROBOT_IP, record=False)
        VLA_ROBOT.start()
        time.sleep(0.1)
        ROBOT_TOGGLE = True
        return(jsonify({"success": True, "message": "Robot started."}))

@app.route("/vla_record_load", methods=["GET", "POST"])
def vla_record_load():
    global RECORD_LOAD
    if not VLA_MODE:
        return(jsonify({"success": False, "message": "VLA mode not enabled."}))
    RECORD_LOAD = not RECORD_LOAD
    return jsonify({"success": True, "message": f"Record load set to {RECORD_LOAD}."})

@app.route("/vla_reset_policy", methods=["GET", "POST"])
def vla_reset_policy():
    global POLICY, VLA_MODE, ACTED, OBS_QUEUE, ACT_QUEUE, LAST_OBS, LAST_OBS_QUEUE, ACT, ACT_DICT_QUEUE, OBJECT_NAME, CONFIG, GRIPPER
    if not VLA_MODE:
        return(jsonify({"success": False, "message": "VLA mode not enabled."}))
    ACTED = False
    # write obs and act to file
    if len(LAST_OBS_QUEUE) > 0:
        dp_log_pth = "/home/will/MAGPIE/datasets/dp_grasp_logs"
        su.dp_log(LAST_OBS_QUEUE, ACT_DICT_QUEUE, OBJECT_NAME, CONFIG["vla"], dp_log_pth)
    GRIPPER.reset_parameters()
    LAST_OBS_QUEUE = []
    OBS_QUEUE = []
    ACT_QUEUE = []
    ACT_DICT_QUEUE = []
    LAST_OBS = {}
    ACT = []
    pol.reset_policy(POLICY)
    return jsonify({"success": True, "message": "Policy reset."})

@app.route("/vla_obs", methods=["GET", "POST"])
def vla_obs():
    global VLA_MODE, LAST_OBS, OBS_QUEUE, LAST_OBS_QUEUE, ACTED, POLICY, ACT, ACT_QUEUE, ACT_DICT_QUEUE
    global GRIPPER, WORKSPACE_CAMERA, WRIST_CAMERA, VLA_ROBOT, OBJECT_NAME, CONFIG
    if not VLA_MODE:
        return(jsonify({"success": False, "message": "VLA mode not enabled."}))
    if not ACTED and len(OBS_QUEUE) > 0:
        return(jsonify({"success": False, "message": "Have not acted, cannot observe."}))
    try:
        sensors = {
            "robot": VLA_ROBOT,
            "gripper": GRIPPER,
            "wrist_camera": WRIST_CAMERA,
            "workspace_camera": WORKSPACE_CAMERA,
            "language_instruction": f"grasp {OBJECT_NAME} {'and return to original position' if 'go' not in CONFIG['vla'] else ''}",
        }
        OBS_QUEUE, LAST_OBS = su.get_observation(sensors, OBS_QUEUE, LAST_OBS, CONFIG["vla"])
        LAST_OBS_QUEUE.append(LAST_OBS)
        print(f"Observation queue length: {len(OBS_QUEUE)}")
        ACT = POLICY(OBS_QUEUE[-1])
        print(f"Policy generated action: {ACT}")
        ad = su.parse_dp_action(ACT, CONFIG["vla"])
        ACT_DICT_QUEUE.append(ad)
        ACT_QUEUE.append(ACT)
    except Exception as e:
        print(e)
        return(jsonify({"success": False, "message": f"Failed to observe: {e}"}))
    ACTED = False
    return jsonify({"success": True, "message": f"obs: {LAST_OBS}"})


@app.route("/vla_act", methods=["GET", "POST"])
def vla_act():
    global VLA_MODE, LAST_OBS, OBS_QUEUE, ACTED, POLICY, ACT, ACT_QUEUE
    global GRIPPER, WORKSPACE_CAMERA, WRIST_CAMERA, VLA_ROBOT, CONFIG, RECORD_LOAD
    if not VLA_MODE:
        return(jsonify({"success": False, "message": "VLA mode not enabled."}))
    if ACTED or len(OBS_QUEUE) == 0:
        return(jsonify({"success": False, "message": "Already acted or no observation, cannot act."}))
    try:
        actuators = {
            "robot": VLA_ROBOT,
            "gripper": GRIPPER,
        }
        su.apply_action(ACT, actuators, action_flag=CONFIG["vla"], record_load=RECORD_LOAD)
    except Exception as e:
        print(e)
        return(jsonify({"success": False, "message": f"Failed to act: {e}"}))
    ACTED = True
    return jsonify({"success": True, "message": f"act: {ACT}"})

@app.route("/vla_obs_act", methods=["GET", "POST"])
def vla_obs_act():
    global VLA_MODE, LAST_OBS, OBS_QUEUE, ACTED, POLICY, ACT, ACT_QUEUE
    global GRIPPER, WORKSPACE_CAMERA, WRIST_CAMERA, ROBOT_IP, CONFIG, RECORD_LOAD
    if not VLA_MODE:
        return(jsonify({"success": False, "message": "VLA mode not enabled."}))
    cycles = int(request.get_json()['message'])
    for i in range(int(cycles)):
        vla_obs()
        # time.sleep(0.05)
        vla_act()
        # time.sleep(0.22)
    return jsonify({"success": True, "message": f"obs_act: {cycles} cycles"})

@app.route("/", methods=["GET", "POST"])
def index():
    global prompt
    global completion
    # return render_template("index.html", prompt=prompt, completion=completion)
    return render_template("index.html")

@app.route("/connect", methods=["POST"])
def connect():
    global CONFIG, CONNECTED
    # Robot
    global GRIPPER, SERVO_PORT, HOME_POSE, SLEEP_RATE, on_robot
    # Perception
    global CAMERA, LABEL, CAMERA_SERIAL_INFO, WRIST_CAMERA, WORKSPACE_CAMERA, CAMERA_SERIAL_INFO 
    # LLM
    global MODEL, TASK_CONFIG, PROMPT_MODEL, PROMPT, CONVERSATION, safe_executor, VISION

    new_conf = request.get_json()
    print(new_conf['policyconf'])
    CONFIG["move"] = new_conf["moveconf"]
    CONFIG["grasp"] = new_conf["graspconf"]
    CONFIG["policy"] = new_conf["policyconf"]
    CONFIG["llm"] = new_conf["llmconf"]
    CONFIG["vlm"] = new_conf["vlmconf"]
    CONFIG["vla"] = new_conf["vlaconf"]
    print(CONFIG)
    MODEL = CONFIG["llm"]

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

    # Robot + Gripper Configuration
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

    # Camera(s) Configuration
    try:
        if on_robot:
            CAMERA_SERIAL_INFO = real.poll_devices()
            print(CAMERA_SERIAL_INFO)
            if len(CAMERA_SERIAL_INFO) < 2:
                rsc = real.RealSense(fps=15, w=640, h=480, device_name="D405")
                rsc.initConnection(device_serial=CAMERA_SERIAL_INFO['D405'])
                WRIST_CAMERA = CAMERA = WORKSPACE_CAMERA = rsc
            else:
                WRIST_CAMERA = real.RealSense(fps=5, w=640, h=480, device_name="D405")
                WRIST_CAMERA.initConnection(device_serial=CAMERA_SERIAL_INFO['D405'])
                print(f"Connected to wrist camera")
                WORKSPACE_CAMERA = real.RealSense(zMax=5, fps=6, w=640, h=480, device_name="D435")
                CAMERA_SERIAL_INFO = real.poll_devices()
                WORKSPACE_CAMERA.initConnection(device_serial=CAMERA_SERIAL_INFO['D435'])
                print(f"Connected to workspace camera")
        # LABEL = Label(label_models[CONFIG['vlm']])
        LABEL = LabelOWLViT(pth=label_models['owl-vit'])
        connect_msg += "Connected to camera and perception models.\n"
    except Exception as e:
        CONNECTED = False
        print(e)
        connect_msg += f"Failed to connect to camera or perception models: {e}.\n"
        return jsonify({"CONFIG": CONFIG, "connected": False, "message": connect_msg})
    
    # VLA Mode Configuration
    try:
        global POLICY, VLA_MODE, VLA_ROBOT
        VLA_MODE = True
        print(f"VLA mode enabled: {VLA_MODE}")
        print(f"config vla: {CONFIG['vla']}")
        print(f"model ckpt: {MODEL_CKPT_DICT[CONFIG['vla']]}")
        ckpt = MODEL_CKPT_DICT[CONFIG["vla"]]
        POLICY, _ = pol.create_policy(ckpt)
        print(f"Created policy: {POLICY}")
        connect_msg += "Connected to VLA policy.\n"
    except Exception as e:
        connect_msg += f"No VLA policy found: exception {e}.\n"
        return jsonify({"CONFIG": CONFIG, "connected": False, "message": connect_msg})

    return jsonify({"CONFIG": CONFIG, "connected": True, "message": connect_msg})

@app.route("/chat", methods=["POST"])
def chat():
    global INTERACTIONS, MESSAGE_LOG, CONFIG
    # Perception
    global CAMERA, LABEL, GOAL_POSE, APERTURE, IMAGE, WRIST_CAMERA
    # LLM
    global PROMPT_MODEL, CONVERSATION, RESPONSE, VISION, OBJECT_NAME

    user_command = request.get_json()['message']
    # INPUT PARSING
    MESSAGE_LOG[INTERACTIONS] = [{"type": "text", "role": "user", "content": user_command}]


    # PERCEPTION
    # TODO: segment object description from user input
    enc_img = None
    try:
        p, rgbd_image = WRIST_CAMERA.getPCD()
        image = np.array(rgbd_image.color)
        # scale image to 640 x 480
        IMAGE = Image.fromarray(image).resize((640, 480))
        queries, abbrevq = parse_object_description(user_command)
        # cast queries to lower case and replace ' ' with '_'
        OBJECT_NAME = queries.lower().replace(' ', '_')
        bboxes, _ = LABEL.label(image, queries, abbrevq, topk=True, plot=False)
        preds_plot = LABEL.preds_plot
        enc_img = encode_image(Image.fromarray(preds_plot))
        index = 0 # TODO: make this user selection, for now take highest confidence
        _, ptcld, GOAL_POSE, _ = pcd.get_segment(LABEL.sorted_labeled_boxes_coords, 
                                         index, 
                                         rgbd_image, 
                                         WRIST_CAMERA, 
                                         type="box-dbscan", 
                                        #  type="box", 
                                        #  method="quat", 
                                         method="iterative", 
                                         display=False)
        print(f"GOAL_POSE: {GOAL_POSE}")
        APERTURE = pcd.get_minimum_width(ptcld)/1000.0 # convert to meters
    except Exception as e:
        print(e)
        msg = "Perception failed. Is the camera connected? " + str(e) + "\n"
        err_msg = {"type": "text", "role": "system", "content": msg}
        return(jsonify(messages=[err_msg]))

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
    global INTERACTIONS, MESSAGE_LOG, CONFIG, IMAGE, GRASP_TIMESTAMP, OBJECT_NAME
    # save MESSAGE_LOG to json
    messages = {"messages": MESSAGE_LOG[INTERACTIONS], "config": CONFIG}
    print(messages)
    # get YYYY_MM_DD_HH_MM_SS
    t = time.localtime()
    timestamp = time.strftime('%Y_%m_%d_%H_%M_%S', t)
    with open(f'interaction_logs/{timestamp}_message_log_id-{INTERACTIONS}.json', 'w') as f:
        f.write(json.dumps(messages))

    # save robot log to rlds
    # assert that GRASP_LOG_DIR exists
    if not os.path.exists(GRASP_LOG_DIR):
        return jsonify({"success": False, "message": "No grasp log directory found."})
    log_to_df(path=GRASP_LOG_DIR, timestamp=GRASP_TIMESTAMP, obj=OBJECT_NAME)

    INTERACTIONS += 1
    return jsonify({"success": True, "message": "New interaction started."})

@app.route("/grasp_policy", methods=["POST"])
def grasp_policy():
    global PROMPT_MODEL, CONVERSATION, RESPONSE, IMAGE, VISION, MESSAGE_LOG, INTERACTIONS, KEEP_POLICY_FLAG

    user_command = request.get_json()['message']
    conv = CONVERSATION
    pm = PROMPT_MODEL
    desc, code = None, None

    if (CONFIG["grasp"] == "dg" or VISION) and not KEEP_POLICY_FLAG:
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
async def execute():
    global PROMPT_MODEL, RESPONSE, MESSAGE_LOG, INTERACTIONS, GRASP_LOG_DIR
    pm = PROMPT_MODEL
    if RESPONSE is None:
        return jsonify(messages=[{"type": "text", "role": "system", "success": False, "content": "No response to execute."}])
    try:
        print("SERVER executing code")
        stdout = await su.execute_grasp_and_record_images(pm.code_executor, 
                                                          RESPONSE, 
                                                          CAMERA_PATH_DICT,
                                                          index=1)
        # print("stdout:", stdout)
        log_grasp(stdout, path=f"{GRASP_LOG_DIR}/grasp.json") # still hardcoded...brittle...
        print(f"SERVER executed code with output {stdout}")
        msg = [{"type": "text", "role": "grasp", "content": f"{stdout}"}]
        MESSAGE_LOG[INTERACTIONS] += msg
        print(MESSAGE_LOG[INTERACTIONS])
        return jsonify(messages=[msg])
    except Exception as e:
        msg = "Execution failed. Is the robot connected? " + str(e) + "\n"
        err_msg = {"type": "text", "role": "system", "content": msg}
        return(jsonify(messages=[err_msg]))

@app.route("/home", methods=["POST"])
async def home():
    global HOME_POSE, AT_GOAL, GRIPPER, CAMERA, ROBOT_IP, GRASP_TIMESTAMP, GRASP_LOG_DIR
    msg = {"operation": "home", "success": False, "message": f"moving to home pose {HOME_POSE}"}
    try:
        if on_robot:
            robot = ur5.UR5_Interface(ROBOT_IP, 
                                      freq=6, # 6Hz frequency
                                      record=True,
                                      record_path=f"{GRASP_LOG_DIR}/home.csv")
            await su.move_robot_and_record_images(robot, 
                                            HOME_POSE, 
                                            CAMERA_PATH_DICT,
                                            index=2,
                                            move_type="linear")
            AT_GOAL = False
            msg["success"] = True
    except Exception as e:
        print(e)
    return jsonify(msg)

@app.route("/move", methods=["POST"])
async def move():
    global HOME_POSE, GOAL_POSE, APERTURE, CONFIG, AT_GOAL
    global CAMERA, CAMERA_PATH_DICT, CAMERA_SERIAL_INFO, WRIST_CAMERA, WORKSPACE_CAMERA
    global GRASP_TIMESTAMP, LOG_DIR, GRASP_LOG_DIR, OBJECT_NAME
    msg = {"operation": "move", "success": False, "message": "moving to goal pose"}
    if GOAL_POSE is None:
        msg["message"] = "No goal pose set."
        return jsonify(msg)
    if AT_GOAL:
        msg["message"] = "Already at goal pose."
        return jsonify(msg)
    try:
        if on_robot:
            GRASP_TIMESTAMP = time.time()
            GRASP_LOG_DIR = f"{LOG_DIR}/{GRASP_TIMESTAMP}_{OBJECT_NAME}_id-{INTERACTIONS}"
            os.mkdir(f"{GRASP_LOG_DIR}")
            os.mkdir(f"{GRASP_LOG_DIR}/workspace_img")
            os.mkdir(f"{GRASP_LOG_DIR}/wrist_img")
            CAMERA_PATH_DICT = {WRIST_CAMERA: f"{GRASP_LOG_DIR}/wrist_img", 
                                WORKSPACE_CAMERA: f"{GRASP_LOG_DIR}/workspace_img",
                                # CAMERA: f"{GRASP_LOG_DIR}/img"
                                }            

            robot = ur5.UR5_Interface(ROBOT_IP, 
                                      freq=6, # 10Hz frequency
                                      record=True, 
                                      record_path=f"{GRASP_LOG_DIR}/move.csv")
            robot.z_offset = 0.0125 # 1.2cm, move 4.0mm closer to object than typical
            print("moving robot")
            await su.move_robot_and_record_images(robot, 
                                            GOAL_POSE, 
                                            CAMERA_PATH_DICT, 
                                            index=0,
                                            move_type="cartesian")
            AT_GOAL = True
            msg["success"] = True
        return jsonify(msg)
    except Exception as e:
        print(f"Error: {e}")
        msg["message"] = f"Failed to move to goal pose: {e}"
        return jsonify(msg)

@app.route("/release", methods=["POST"])
def release():
    global GRIPPER
    msg = {"operation": "release", "success": False}
    try:
        if on_robot:
            GRIPPER.reset_parameters()
            msg["success"] = True
    except Exception as e:
        print(e)
    return jsonify(msg)

@app.route("/grasp", methods=["POST"])
def grasp():
    global GRIPPER
    global CONFIG
    msg = {"operation": "grasp", "success": False}
    try:
        if on_robot:
            if CONFIG["grasp"] == "cag":
                GRIPPER.adaptive_grasp()
            else:
                GRIPPER.close_gripper()
            msg["success"] = True
    except Exception as e:
        print(e)

    return jsonify(msg)

@app.route("/set_home", methods=["POST"])
def set_home():
    global HOME_POSE
    global AT_GOAL
    try:
        if on_robot and not AT_GOAL:
            robot = ur5.UR5_Interface(ROBOT_IP)
            robot.start()
            HOME_POSE = robot.getPose()
            robot.stop()
    except Exception as e:
        print(e)
    return jsonify({"message": "Home pose set to current robot pose."})

@app.route("/keep_policy", methods=["POST"])
def keep_policy():
    global KEEP_POLICY_FLAG, RESPONSE
    if RESPONSE is not None:
        KEEP_POLICY_FLAG = not KEEP_POLICY_FLAG
    return jsonify({"messages": "Reusing DeliGrasp policy."})

if __name__ == "__main__":
    app.run(debug=True)
