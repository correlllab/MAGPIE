prompt_thinker = """
Control a robot gripper with force control and contact information. 
The gripper's parameters can be adjusted corresponding to the type of object that it is trying to grasp as well as the kind of grasp it is attempting to perform.
The gripper has a measurable max force of 16N and min force of 0.15N, a maximum aperture of 105mm and a minimum aperture of 1mm.

Some grasps may be incomplete, intended for observing force information about a given object.
Describe the grasp strategy using the following form:

[start of description]
* This {CHOICE: [is, is not]} a new grasp.
* In accordance with the user instruction, this grasp should be [GRASP_DESCRIPTION: <str>].
* This is a {CHOICE: [complete, incomplete]} grasp.
* This grasp {CHOICE: [does, does not]} contain multiple grasps.
* This grasp is for an object with {CHOICE: [high, medium, low]} weight.
* The object has an approximate mass of [PNUM: 0.0] grams
* This grasp is for an object with {CHOICE: [high, medium, low]} compliance.
* The object has an approximate spring constant of [PNUM: 0.0] Newtons per meter.
* The gripper and object have an approximate friction coefficient of [PNUM: 0.0]
* This grasp should set the goal aperture to [PNUM: 0.0] mm.
* If the gripper slips, this grasp should close an additional [PNUM: 0.0] mm.
* If the gripper slips, this grasp should increase the output force by [PNUM: 0.0] Newtons.
* [optional] Because of [GRASP_DESCRIPTION: <str>], this grasp sets the force to be {CHOICE: [lower, higher]} than the default minimum grasp force.
[end of description]

Rules:
1. If you see phrases like {NUM: default_value}, replace the entire phrase with a numerical value. If you see {PNUM: default_value}, replace it with a positive, non-zero numerical value.
2. If you see phrases like {CHOICE: [choice1, choice2, ...]}, it means you should replace the entire phrase with one of the choices listed. Be sure to replace all of them. If you are not sure about the value, just use your best judgement.
3. If you see phrases like [GRASP_DESCRIPTION: default_value], use information from the user instruction to provide a description of the grasp or the object to be grasped, including mentioned physical characteristics or features.
4. Using information from the user instruction about the object and the grasp description, set the initial grasp force either to this default value or an appropriate value. 
5. If you deviate from the default force value, explain your reasoning using the optional bullet points. It is not common to deviate from the default value.
6. Using knowledge of the object and how compliant it is, estimate the spring constant of the object. This can range broadly from 20 N/m for a very soft object to 2000 N/m for a very stiff object. 
7. Using knowledge of the object and the grasp description, if the grasp slips, first estimate an appropriate increase to the aperture closure, and then the gripper output force.
8. The increase in gripper output force the maximum value of (0.05 N, or the product of the estimated aperture closure, the spring constant of the object, and a damping constant 0.1: (k*additional_closure*0.0001)).
9. Provide the full description of the grasp plan, even if you may only need to change a few lines. Always start the description with [start of description] and end it with [end of description].
10. Do not add additional descriptions not shown above. Only use the bullet points given in the template.
11. Make sure to give the full description. Do not skip points if they are not optional.
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