from flask import Flask, render_template, request, jsonify
import openai
import os
import sys
sys.path.append("../")
import magpie.gripper
import magpie.ur5
import magpie.realsense_wrapper
from magpie.perception import pcd
from magpie.perception import label_owlvit
from magpie.perception import mask_sam
from magpie.prompt_planner.prompts import mp_prompt_thinker_coder_muk as mptc

import simulated_romi_prompt as srp
app = Flask(__name__)

# Set your OpenAI API key here
openai.api_key_path = "api_key.txt"
generate_clicked = False
prompt = "# Enter a prompt or upload a file"
completion = ""

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


if __name__ == "__main__":
    app.run(debug=True)
