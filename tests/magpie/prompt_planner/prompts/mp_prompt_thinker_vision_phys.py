prompt_thinker = """
Given the user instruction and image, control a robot gripper with torque control and contact information.
The gripper's parameters can be adjusted corresponding to the type of object that it is trying to manipulate.
As well as the kind of grasp it is attempting to perform, as described in the user command and provided image.
The gripper has a measurable max force of 16N, min force of 0.15N, max velocity of 0.3 m/s, a maximum aperture of 105mm and a minimum aperture of 1mm.
Some grasps may be incomplete, intended for observing force information about a given object.
Some manipulation can be pokes, taps, or other light touches to the object with the left or right finger.
Describe the grasp strategy using the following form:

[start of description]
* This {CHOICE: [is, is not]} a new grasp.
* This grasp should be [DESCRIPTION: <str>].
* The grasp and object description {CHOICE: [is, is not]} consistent with the provided image.
* In the provided image, I observe [DESCRIPTION: <str>]
* This is a {CHOICE: [complete, incomplete]} grasp.
* This grasp {CHOICE: [does, does not]} contain multiple grasps.
* This object is more fragile than [example object:  <str>] and less fragile than [example object:  <str>]
* This grasp is for an object with {CHOICE: [high, medium, low]} compliance.
* Thus, the object has an approximate spring constant of [PNUM: 0.0] Newtons per meter.
* This grasp is for an object with {CHOICE: [high, medium, low]} weight.
* This object has more mass than [example object:  <str>], which has an approximate mass of [PNUM: 0.0] g 
* This object has less mass than [example object:  <str>], which has an approximate mass of [PNUM: 0.0] g
* Thus, the object has a typical approximate mass of [PNUM: 0.0] grams, which is between those masses.
* Based off the image, the object has {CHOICE: [less, more]} mass than typical, which is [PNUM: 0.0] g.
* Based off the image, the object is {CHOICE: [rough, smooth]}.
* The gripper and object have an approximate friction coefficient of [PNUM: 0.0]
* This grasp should set the goal aperture to [PNUM: 0.0] mm.
* If the gripper slips, this grasp should close an additional [PNUM: 0.0] mm.
* Based on object mass and friction coefficient, grasp should initially set the contact force to [PNUM: 0.0] Newtons.
* If the gripper slips, this grasp should increase the output force by [PNUM: 0.0] Newtons.
* [optional] This grasp {CHOICE: [does, does not]} use the default minimum grasp force force.
* [optional] This grasp sets the force to  [PNUM: 0.0], which is {CHOICE: [lower, higher]} than the default initial contact force because of [GRASP_DESCRIPTION: <str>].
* [optional] Based on the image, the object and the surface it rests on have an approximate friction coefficient of [PNUM: 0.0].
* [optional] This grasp is to poke the object to the {CHOICE: [left, right]} by [PNUM: 0.0] mm.
[end of description]

Rules:
1. If you see phrases like {NUM: default_value}, replace the entire phrase with a numerical value. If you see {PNUM: default_value}, replace it with a positive, non-zero numerical value.
2. If you see phrases like {CHOICE: [choice1, choice2, ...]}, it means you should replace the entire phrase with one of the choices listed. Be sure to replace all of them. If you are not sure about the value, just use your best judgement.
3. If you see phrases like [DESCRIPTION: default_value], replace the entire phrase with a brief, high level description of the grasp and the object to be grasp, including physical characteristics or important features.
4. If you see phrases like [example object: <str>], replace the entire phrase with an appropriate example object that is similar to the object to be grasped.
5. By default the minimum grasp force can be estimated by dividing the object weight (mass * gravitational constant) by the friction coefficient: (m*g/μ).
6. Using knowledge of the object, the grasp description, and the provided image, set the initial grasp force either to this default value or an appropriate value. 
7. If you deviate from the default value, explain your reasoning using the optional bullet points. It is not common to deviate from the default value.
8. Using knowledge of the object, the provided image of the object, and how compliant it is, estimate the spring constant of the object. This can range broadly from 20 N/m for a very soft object to 2000 N/m for a very stiff object. 
9. Using knowledge of the object, the provided image of the object, and the grasp description, if the grasp slips, first estimate an appropriate increase to the aperture closure, and then the gripper output force.
10. The increase in gripper output force the maximum value of (0.01 N, or the product of the estimated aperture closure, the spring constant of the object, and a damping constant 0.1: (k*additional_closure*0.0001)).
11. I will tell you a behavior/skill/task that I want the gripper to perform in the grasp and you will provide the full description of the grasp plan, even if you may only need to change a few lines. Always start the description with [start of description] and end it with [end of description].
12. We can assume that the gripper has a good low-level controller that maintains position and force as long as it's in a reasonable pose.
13. The goal aperture of the gripper will be supplied externally, do not calculate it.
14. Do not add additional descriptions not shown above. Only use the bullet points given in the template.
15. Use as few bullet points as possible. Be concise.
"""

import re
import sys
sys.path.append('../../')

import magpie.prompt_planner.safe_executor as safe_executor
import magpie.prompt_planner.llm_prompt as llm_prompt
import magpie.prompt_planner.process_code as process_code
import magpie.prompt_planner.magpie_execution as magpie_execution
# import magpie_task_client


class PromptThinker(llm_prompt.LLMPrompt):
  """Prompt with both Motion Descriptor and Reward Coder."""

  def __init__(
      self,
      client: None,
      executor: safe_executor.SafeExecutor,
  ):
    # self._agent = client.agent()
    self._safe_executor = magpie_execution.MagpieSafeExecutor(executor)

    self.name = "Language2StructuredLang"

    self.num_llms = 1
    self.prompts = [prompt_thinker]

    # The coder doesn't need to keep the history as it only serves a purpose for
    # translating to code
    self.keep_message_history = [True, False]
    self.response_processors = [
        self.process_thinker_response,
    ]

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