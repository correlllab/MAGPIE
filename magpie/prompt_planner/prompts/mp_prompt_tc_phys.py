prompt_thinker = """
Control a robot gripper with torque control and contact information. 
This is a gripper with two independently actuated fingers.
The gripper's parameters can be adjusted corresponding to the type of object that it is trying to grasp.
As well as the kind of grasp it is attempting to perform.
The gripper has a measurable max force of 16N and min force of 0.15N, a maximum aperture of 105mm and a minimum aperture of 1mm.

Some grasps may be incomplete, intended for observing force information about a given object.
Describe the grasp strategy using the following form:

[start of description]
* This {CHOICE: [is, is not]} a new grasp.
* This grasp should be [GRASP_DESCRIPTION: <str>].
* This is a {CHOICE: [complete, incomplete]} grasp.
* This grasp {CHOICE: [does, does not]} contain multiple grasps.
 * This object is more fragile than [example object:  <str>] and less fragile than [example object:  <str>]
* This grasp is for an object with {CHOICE: [high, medium, low]} compliance.
* This grasp is for an object with {CHOICE: [high, medium, low]} weight.
* This object has more mass than [example object:  <str>], which has an approximate mass of [PNUM: 0.0] g 
* This object has less mass than [example object:  <str>], which has an approximate mass of [PNUM: 0.0] g
* Thus, the object has an approximate mass of [PNUM: 0.0] grams, which is between those masses.
* The object has an approximate spring constant of [PNUM: 0.0] Newtons per meter.
* The gripper and object have an approximate friction coefficient of [PNUM: 0.0]
* This grasp should set the goal aperture to [PNUM: 0.0] mm.
* If the gripper slips, this grasp should close an additional [PNUM: 0.0] mm.
* Based on object mass and friction coefficient, grasp should initially set the force to [PNUM: 0.0] Newtons.
* If the gripper slips, this grasp should increase the output force by [PNUM: 0.0] Newtons.
* [optional] This grasp {CHOICE: [does, does not]} use the default minimum grasp force force.
* [optional] This grasp sets the force to  [PNUM: 0.0], which is {CHOICE: [lower, higher]} than the default initial force because of [GRASP_DESCRIPTION: <str>].
[end of description]

Rules:

1. If you see phrases like {NUM: default_value}, replace the entire phrase with a numerical value. If you see {PNUM: default_value}, replace it with a positive, non-zero numerical value.
2. If you see phrases like {CHOICE: [choice1, choice2, ...]}, it means you should replace the entire phrase with one of the choices listed. Be sure to replace all of them. If you are not sure about the value, just use your best judgement.
3. If you see phrases like [GRASP_DESCRIPTION: default_value], replace the entire phrase with a brief, high level description of the grasp and the object to be grasp, including physical characteristics or important features.
4. If you see phrases like [example object: <str>], replace the entire phrase with an appropriate example object that is similar to the object to be grasped.
4. By default the minimum grasp force can be estimated by dividing the object weight (mass  gravitational constant) by the friction coefficient: (mg/μ).
5. Using knowledge of the object and the grasp description, set the initial grasp force either to this default value or an appropriate value. 
6. If you deviate from the default value, explain your reasoning using the optional bullet points. It is not common to deviate from the default value.
7. Using knowledge of the object and how compliant it is, estimate the spring constant of the object. This can range broadly from 20 N/m for a very soft object to 2000 N/m for a very stiff object. 
8. Using knowledge of the object and the grasp description, if the grasp slips, first estimate an appropriate increase to the aperture closure, and then the gripper output force.
9. The increase in gripper output force the maximum value of (0.05 N, or the product of the estimated aperture closure, the spring constant of the object, and a damping constant 0.1: (k*additional_closure*0.0001)).
10. I will tell you a behavior/skill/task that I want the gripper to perform in the grasp and you will provide the full description of the grasp plan, even if you may only need to change a few lines. Always start the description with [start of description] and end it with [end of description].
11. We can assume that the gripper has a good low-level controller that maintains position and force as long as it's in a reasonable pose.
12. The goal aperture of the gripper will be supplied externally, do not calculate it.
13. Do not add additional descriptions not shown above. Only use the bullet points given in the template.
14. Make sure to give the full description. Do not skip points if they are not optional.
15. Use as few bullet points as possible. Be concise.
"""

prompt_coder = """
We have a description of a gripper's motion and force sensing and we want you to turn that into the corresponding program with following class functions of the gripper:
The gripper has a measurable max force of 16N and min force of 0.15N, a maximum aperture of 105mm and a minimum aperture of 1mm.

```
def get_aperture(finger='both')
```
finger: which finger to get the aperture in mm, of, either 'left', 'right', or 'both'. If 'left' or 'right', returns aperture, or distance, from finger to center. If 'both', returns aperture between fingers.

```
def set_goal_aperture(aperture, finger='both', record_load=False)
```
aperture: the aperture to set the finger(s) to (in mm)
finger: which finger to set the aperture in mm, of, either 'left', 'right', or 'both'.
record_load: whether to record the load at the goal aperture. If true, will return array of (pos, load) tuples
This function will move the finger(s) to the specified goal aperture, and is used to close and open the gripper.
Returns a position-load data array of shape (2, n) --> [[positions], [loads]], average force, and max force after the motion.

```
def set_compliance(margin, flexibility, finger='both')
```
margin: the allowable error between the goal and present position (in mm)
flexibility: the compliance slope of motor torque (value 0-7, higher is more flexible) until it reaches the compliance margin
finger: which finger to set compliance for, either 'left', 'right', or 'both'

```
def set_force(force, finger='both')
```
force: the maximum force the finger is allowed to apply at contact with an object(in N), ranging from (0.1 to 16 N)
finger: which finger to set compliance for, either 'left', 'right', or 'both'

```
def deligrasp(goal_aperture, initial_force, additional_closure, additional_force, complete_grasp)
```
goal_aperture: the goal aperture to grasp the object (in mm)
initial_force: the initial force to apply to the object (in N)
additional_closure: the additional aperture to close if the gripper slips (in mm)
additional_force: the additional force to apply if the gripper slips (in N)
complete_grasp: whether the grasp is complete or incomplete (True or False)
This function will close the gripper to the goal aperture, apply the initial force, and adjust the force if the gripper slips. If the grasp is incomplete, the gripper will open after the slip check.

```
def poke(direction, speed, aperture)
```
direction: 'left' or 'right' to poke the left or right finger
speed of finger in m/s
aperture: distance to poke in mm (of one finger, not both)


Example answer code:
```
from magpie.gripper import Gripper # must import the gripper class
G = Gripper() # create a gripper object
import numpy as np  # import numpy because we are using it below

new_task = {CHOICE: [True, False]} # Whether or not the task is new
# Reset parameters to default since this is a new, delicate grasp to avoid crushing the raspberry.
if new_task:
    G.reset_parameters()

# [REASONING] 
goal_aperture = {PNUM: goal_aperture}
complete_grasp = {CHOICE: [True, False]} 
# Initial force. Convert mass (g) to (kg). The default value of object weight / friction coefficient.
initial_force = {PNUM: {CHOICE: [({PNUM: mass} * 9.81) / ({PNUM: mu} * 1000), {PNUM: different_inital_force}] }}}
# [REASONING for initial force choice]
additional_closure = {PNUM: additional_closure} 
# Additional force increase. The default value is the product of the object spring constant and the additional_closure, with a dampening constant 0.1.
additional_force = np.max([0.01, additional_closure * {PNUM: spring_constant} * 0.0001])

# Move quickly (without recording load) to a safe aperture that is wider than the goal aperture
G.set_goal_aperture(goal_aperture + additional_closure * 2, finger='both', record_load=False)

# [REASONING]
# [PREDICTION]
G.set_compliance(1, 3, finger='both')
G.set_force(initial_force, 'both')

G.deligrasp(goal_aperture, initial_force, additional_closure, additional_force, complete=complete_grasp, debug=True)
```

Remember:
1. Always format the code in code blocks. In your response all five functions above: get_aperture, set_goal_aperture, set_compliance, set_force, check_slip should be used.
2. Do not invent new functions or classes. The only allowed functions you can call are the ones listed above. Do not leave unimplemented code blocks in your response.
3. The only allowed library is numpy. Do not import or use any other library. If you use np, be sure to import numpy.
4. If you are not sure what value to use, just use your best judge. Do not use None for anything.
5. If you see phrases like [REASONING], replace the entire phrase with a code comment explaining the grasp strategy and its relation to the following gripper commands.
6. If you see phrases like [PREDICTION], replace the entire phrase with a prediction of the gripper's state after the following gripper commands are executed.
7. If you see phrases like {PNUM: default_value}, replace the value with the corresponding value from the grasp description.
8. If you see phrases like {CHOICE: [choice1, choice2, ...]}, it means you should replace the entire phrase with one of the choices listed. Be sure to replace all of them. If you are not sure about the value, just use your best judgement.
9. Remember to import the gripper class and create a Gripper at the beginning of your code.
"""

import re
import sys
sys.path.append('../../')

import magpie.prompt_planner.safe_executor as safe_executor
import magpie.prompt_planner.llm_prompt as llm_prompt
import magpie.prompt_planner.process_code as process_code
import magpie.prompt_planner.magpie_execution as magpie_execution
# import magpie_task_client


class PromptThinkerCoderPhys(llm_prompt.LLMPrompt):
  """Prompt with both Motion Descriptor and Reward Coder."""

  def __init__(
      self,
      client: None,
      executor: safe_executor.SafeExecutor,
  ):
    # self._agent = client.agent()
    self._safe_executor = magpie_execution.MagpieSafeExecutor(executor)

    self.name = "LanguageAndVision2StructuredLang2GraspParameters"

    self.num_llms = 2
    self.prompts = [prompt_thinker, prompt_coder]

    # The coder doesn't need to keep the history as it only serves a purpose for
    # translating to code
    self.keep_message_history = [True, False]
    self.response_processors = [
        self.process_thinker_response,
        self.process_coder_response,
    ]
    self.code_executor = self.execute_code

  # process the response from thinker, the output will be used as input to coder
  def process_thinker_response(self, response: str) -> str:
    try:
      motion_description = (
          re.split(
              "end of description",
              re.split("start of description", response, flags=re.IGNORECASE)[
                  1
              ],
              flags=re.IGNORECASE,
          )[0]
          .strip("[")
          .strip("]")
          .strip()
          .strip("```")
      )
      return motion_description
    except Exception as _:  # pylint: disable=broad-exception-caught
      return response

  def process_coder_response(self, response):
    """Process the response from coder, the output will be the python code."""
    return process_code.process_code_block(response)

  def execute_code(self, code: str) -> None:
    print("ABOUT TO EXECUTE\n", code)
    grasp_log = self._safe_executor.execute(code)
    print(grasp_log)
    return grasp_log
    # self._agent.set_task_parameters(mjpc_parameters.task_parameters)
    # self._agent.set_cost_weights(mjpc_parameters.cost_weights)
