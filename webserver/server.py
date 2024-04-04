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

on_robot = platform.system() == "Linux"
if on_robot:
    sys.path.append("../")
    from magpie.gripper import Gripper
    from magpie import ur5 as ur5
    import magpie.realsense_wrapper
    from magpie.perception import pcd
    # from magpie.perception import label_owlvit
    # from magpie.perception import mask_sam
    from magpie.prompt_planner.prompts import mp_prompt_thinker_coder_muk as mptc

import simulated_romi_prompt as srp
app = Flask(__name__)

# Set your OpenAI API key here
openai.api_key_path = "api_key.txt"
generate_clicked = False
prompt = "# Enter a prompt or upload a file"
completion = ""

# metadata / logging
config = {"move": "3D Pos", "grasp": "dg", "llm": "gpt-4-turbo-preview"}
interactions = 0
message_log = {}

# hardware
servo_port = "/dev/ttyACM0"
gripper = None

robot_ip = "192.168.0.6"
# robot = None
# robot = ur5.UR5_Interface(robot_ip)
home_pose = None
sleep_rate = 0.5

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
    return render_template("index.html", prompt=prompt, completion=completion)

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
    global config
    global gripper
    global servo_port
    # global robot
    global home_pose
    global sleep_rate

    new_conf = request.get_json()
    config["move"] = new_conf["moveconf"]
    config["grasp"] = new_conf["graspconf"]
    config["llm"] = new_conf["llmconf"]
    print(config)
    try:
        if on_robot:
            robot = ur5.UR5_Interface(robot_ip)
            gripper = Gripper(servo_port)
            # gripper.reset_parameters()
            robot.start()
            home_pose = robot.getPose()
            time.sleep(sleep_rate)
            robot.stop()
            # TODO: log home pose
        return jsonify({"config": config, "connected": True})
    except Exception as e:
        print(e)
        return jsonify({"config": config, "connected": False})


@app.route("/chat", methods=["POST"])
def chat():
    global interactions
    global message_log
    # global prompt
    # global completion
    # prompt = str(request.get_json())
    # # Use the OpenAI API to generate a completion using "davinci" engine
    # response = openai.Completion.create(
    #     engine="davinci",  # Use "davinci" engine for chat completions
    #     prompt=prompt,
    #     temperature=0.7,
    #     max_tokens=150  # Adjust the number of tokens in the completion
    # )
    # completion = response.choices[0].text.strip()
    # return jsonify({"completion": completion})
    input_text = request.get_json()['message']
    print(input_text)
    img_path = "static/favicon.jpg"
    enc_img = encode_image(Image.open(f'{img_path}'))

    code = """from magpie.gripper import Gripper
G = Gripper()
G.reset_parameters()

goal_aperture = 18.0
initial_force = slip_threshold = 0.0625
additional_closure = 1.0
additional_force_increase = 0.01
k = G.grasp(goal_aperture, slip_threshold, additional_closure, additional_force_increase)
"""
    messages = [
        {"type": "text", "content": "This is a text message."},
        {"type": "image", "content": enc_img},
        {"type": "code", "content": code}
        # {"type": "image", "content": encode_image(np.array(Image.open("static/favicon.jpg")))},
    ]
    # add user input to front of messages
    message_log[interactions] = [{"type": "text", "content": input_text}, *messages]
    print(message_log)
    return jsonify(messages=messages)

@app.route("/execute", methods=["POST"])
def execute():
    pass

@app.route("/home", methods=["POST"])
def home():
    global robot
    global home_pose
    msg = {"operation": "home", "success": False}
    try:
        if on_robot:
            robot = ur5.UR5_Interface(robot_ip)
            robot.start()
            robot.moveL(home_pose)
            time.sleep(sleep_rate * 2)
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
    global gripper
    msg = {"operation": "release", "success": False}
    try:
        if on_robot:
            # gripper.release()
            # gripper.open_gripper()
            gripper.reset_parameters()
            msg["success"] = True
    except Exception as e:
        print(e)
    return jsonify(msg)

@app.route("/grasp", methods=["POST"])
def grasp():
    global gripper

    msg = {"operation": "grasp", "success": False}
    try:
        if on_robot:
            # gripper.grasp()
            gripper.close_gripper()
            msg["success"] = True
    except Exception as e:
        print(e)
    
    return jsonify(msg)

if __name__ == "__main__":
    app.run(debug=True)
