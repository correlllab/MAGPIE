from flask import Flask, render_template, request, jsonify
import openai
import os
import sys
import base64
from PIL import Image
import numpy as np
import io
import logging
import platform
import time
import dataclasses
from typing import Any
sys.path.append("../")
from magpie.prompt_planner.prompts import mp_prompt_thinker_coder_muk as mptc
from magpie.prompt_planner import conversation
from magpie.prompt_planner import confirmation_safe_executor
from magpie.prompt_planner import task_configs

on_robot = platform.system() == "Linux"
if on_robot:
    sys.path.append("../")
    from magpie.gripper import Gripper
    # from magpie import ur5 as ur5
    import magpie.realsense_wrapper
    from magpie.perception import pcd
    # from magpie.perception import label_owlvit
    # from magpie.perception import mask_sam
    from magpie.prompt_planner.prompts import mp_prompt_thinker_coder_muk as mptc

import simulated_romi_prompt as srp
app = Flask(__name__)

# metadata / logging
CONFIG = {"move": "3D Pos", "grasp": "dg", "llm": "gpt-4-turbo-preview", "vlm": "owl-vit"}
INTERACTIONS = 0
MESSAGE_LOG = {}
CONNECTED = False

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
RESPONSE = None
PROMPT_MODEL = None
CONVERSATION = None

# hardware
SERVO_PORT = "/dev/ttyACM0"
GRIPPER = None
ROBOT_IP = "192.168.0.6"
HOME_POSE = None
SLEEP_RATE = 0.5


def encode_image(pil_img):
    img_io = io.BytesIO()
    pil_img.save(img_io, 'jpeg', quality=100)
    img_io.seek(0)
    img = base64.b64encode(img_io.getvalue()).decode('ascii')
    img_tag = f'<img src="data:image/jpg;base64,{img}" class="img-fluid"/>'
    # return img_tag
    return img

@app.route("/", methods=["GET", "POST"])
def index():
    global prompt
    global completion
    # return render_template("index.html", prompt=prompt, completion=completion)
    return render_template("index.html")

@app.route("/generate", methods=["POST"])
def generate():
    prompt = str(request.get_json())
    # Use the OpenAI API to generate a completion using "codex" engine
    response = openai.Completion.create(
        engine="code-davinci-002",  # Use "codex" engine for code completions
        prompt=srp.prompt + prompt,
        temperature=1.0,
        max_tokens=200  # Adjust the number of tokens in the completion
    )
    completion = response.choices[0].text.strip()
    return jsonify({"completion": completion})

@app.route("/connect", methods=["POST"])
def connect():
    global CONFIG
    global CONNECTED
    global GRIPPER
    global SERVO_PORT
    # global robot
    global HOME_POSE
    global SLEEP_RATE
    global MODEL
    global PROMPT_MODEL
    global CONVERSATION
    global safe_executor

    new_conf = request.get_json()
    CONFIG["move"] = new_conf["moveconf"]
    CONFIG["grasp"] = new_conf["graspconf"]
    CONFIG["llm"] = new_conf["llmconf"]
    CONFIG["vlm"] = new_conf["vlmconf"]
    print(CONFIG)
    # MODEL = CONFIG["llm"]
    MODEL = "gpt-3.5-turbo"

    connect_msg = ""
    # LLM Configuration
    try:
        PROMPT_MODEL = PROMPT(
            None, executor=safe_executor
        )
        CONVERSATION = conversation.Conversation(PROMPT_MODEL, MODEL)
        connect_msg += "Created conversation agent.\n"
        CONNECTED = True
    except Exception as e:
        print(e)
        connect_msg += f"Failed to create conversation agent: {e}.\n"
        return jsonify({"CONFIG": CONFIG, "connected": False, "message": connect_msg})
        PROMPT_MODEL = None
        CONVERSATION = None

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
            connect_msg += "Connected to robot and gripper.\n"
        return jsonify({"CONFIG": CONFIG, "connected": True, "message": connect_msg})
    except Exception as e:
        CONNECTED = False
        print(e)
        connect_msg += f"Failed to connect to robot and gripper: {e}.\n"
        return jsonify({"CONFIG": CONFIG, "connected": False, "message": connect_msg})


@app.route("/chat", methods=["POST"])
def chat():
    global INTERACTIONS
    global MESSAGE_LOG
    global PROMPT_MODEL
    global CONVERSATION
    global CONFIG
    global RESPONSE

    # MODEL = CONFIG["llm"]

    user_command = request.get_json()['message']
    conv = CONVERSATION
    pm = PROMPT_MODEL
    try:
        RESPONSE = conv.send_command(user_command)
        desc = conv._llm_responses[0]
        code = RESPONSE
    except Exception as e:
        msg = "Planning failed. Is the system connected? " + str(e) + "\n"
        err_msg = {"type": "text", "role": "system", "content": msg}
        return(jsonify(messages=[err_msg]))

    print(conv._message_queues)
    img_path = "static/favicon.jpg"
    enc_img = encode_image(Image.open(f'{img_path}'))
#     code = """from magpie.GRIPPER import Gripper
# G = Gripper()
# G.reset_parameters()

# goal_aperture = 18.0
# initial_force = slip_threshold = 0.0625
# additional_closure = 1.0
# additional_force_increase = 0.01
# k = G.grasp(goal_aperture, slip_threshold, additional_closure, additional_force_increase)
# """
    messages = [
        {"type": "text",  "role": "llm", "content": "This is a text message."},
        {"type": "image", "role": "vlm", "content": enc_img},
        {"type": "code",  "role": "llm", "content": desc},
        {"type": "code",  "role": "llm", "content": code}
        # {"type": "image", "content": encode_image(np.array(Image.open("static/favicon.jpg")))},
    ]
    # add user input to front of messages
    MESSAGE_LOG[INTERACTIONS] = [{"type": "text", "role": "user", "content": user_command}, *messages]
    # print(MESSAGE_LOG)
    return jsonify(messages=messages)

@app.route("/execute", methods=["POST"])
def execute():
    global PROMPT_MODEL
    global RESPONSE

    pm = PROMPT_MODEL
    if RESPONSE is None:
        return jsonify({"success": False, "message": "No response to execute."})
    try:
        pm.code_executor(RESPONSE)
        return jsonify({"success": True, "message": "Code executed successfully."})
    except Exception as e:  # pylint: disable=broad-exception-caught
        msg = "Execution failed. Is the robot connected? " + str(e) + "\n"
        err_msg = {"type": "text", "role": "system", "content": msg}
        return(jsonify(messages=[err_msg]))

@app.route("/home", methods=["POST"])
def home():
    global robot
    global HOME_POSE
    msg = {"operation": "home", "success": False}
    try:
        if on_robot:
            robot = ur5.UR5_Interface(ROBOT_IP)
            robot.start()
            robot.moveL(HOME_POSE)
            time.sleep(SLEEP_RATE * 2)
            robot.stop()
            msg["success"] = True
    except Exception as e:
        print(e)
    return jsonify(msg)

@app.route("/move", methods=["POST"])
def move():
    pass

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
