import openai
import os
import sys
import termcolor
sys.path.append("../")
from magpie.prompt_planner.prompts import mp_prompt_thinker_coder_muk as mptc
from magpie.prompt_planner import conversation
from magpie.prompt_planner import confirmation_safe_executor

API_KEY = os.environ['CORRELL_API_KEY']

import numpy as np
from PIL import Image
import base64
import io
import cv2 

def encode_image(img):
    # Convert NumPy array to base64 encoded string
    return base64.b64encode(img).decode('utf-8')


i = encode_image(np.array(Image.open("static/favicon.jpg")))

MODEL = "gpt-3.5-turbo"
import dataclasses
from typing import Any

@dataclasses.dataclass(frozen=True)
class TaskConfig:
  client: None
  prompts: dict[str, type[Any]]

ALL_TASKS = {
    'magpie': TaskConfig(
        client=None,
        prompts={
            'thinker_coder': mptc.PromptThinkerCoder,
        },
    ),
}

safe_executor = confirmation_safe_executor.ConfirmationSafeExecutor()
task_config = ALL_TASKS['magpie']
openai.api_key = API_KEY
prompt = task_config.prompts['thinker_coder']
MODEL = "gpt-3.5-turbo"

try:
    prompt_model = prompt(
        None, executor=safe_executor
    )
    conv = conversation.Conversation(prompt_model, MODEL)
    while True:
      user_command = input(termcolor.colored("User: ", "red", attrs=["bold"]))
      try:
        response = conv.send_command(user_command)
        print(termcolor.colored("Magpie: ", "green", attrs=["bold"]) + response + "\n")
      except Exception as e:  # pylint: disable=broad-exception-caught
        print("Planning failed, try something else... " + str(e) + "\n")
        continue
      
      try:
        prompt_model.code_executor(response)
      except Exception as e:  # pylint: disable=broad-exception-caught
        print("Execution failed, try something else... " + str(e) + "\n")

except Exception as e:
    print(e)
    prompt_model = None