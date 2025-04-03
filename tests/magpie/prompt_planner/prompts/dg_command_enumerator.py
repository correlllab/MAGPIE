prompt_command_enumerator = """
You are helping to break down a user instruction for robot manipulation into key elements. 
From any user instruction, enumerate the key elements according to these rules:

1. The object or objects to be manipulated.
2. Specific qualities about the object or objects that are relevant to the manipulation.
3. The type of manipulation that is to be performed.
4. Specific qualities about the manipulation that are relevant to the task. 
5. If there is insufficient information to complete the enumeration, please indicate so and request for clarification.
6. Enumerate these key elements as if you are writing a dictionary in Python, such that the string literal can be directly converted to a Python data structure. If the object is a string, include it in quotes. If the object is a list, include it in square brackets. If the object is a dictionary, include it in curly brackets.

Strictly use the following format and follow the above rules in your response:
[start of enumeration]
{'objects': ['object_name_1', 'object_name_2', ...],
'object_qualities': {'object_name_1': ['quality_1', 'quality_2', ...], 'object_name_2': ['quality_1', 'quality_2', ...], ...}
'manipulation': 'manipulation_type'
'manipulation_qualities': ['quality_1', 'quality_2', ...]}
[end of enumeration]
"""

scraps = """
Strictly use the following format and follow the above rules in your response:
[start of enumeration]
1. Objects: [object_name_1, object_name_2, ...]
2. Object Traits: {object_name_1: [trait_1, trait_2, ...], object_name_2: [trait_1, trait_2, ...], ...}
3. Manipulation: manipulation_type
4. Manipulation Qualities: [quality_1, quality_2, ...]
[end of enumeration]

"""
import re
import sys
import ast
sys.path.append('../../')

import magpie.prompt_planner.safe_executor as safe_executor
import magpie.prompt_planner.llm_prompt as llm_prompt
import magpie.prompt_planner.magpie_execution as magpie_execution
# import magpie_task_client


class PromptCommandEnumerator(llm_prompt.LLMPrompt):
  """Prompt with both Motion Descriptor and Reward Coder."""

  def __init__(
      self,
      client: None,
      executor: safe_executor.SafeExecutor,
  ):
    # self._agent = client.agent()
    self._safe_executor = magpie_execution.MagpieSafeExecutor(executor)

    self.name = "DeliGraspCommandEnumerator"

    self.num_llms = 1
    self.prompts = [prompt_command_enumerator]

    # The coder doesn't need to keep the history as it only serves a purpose for
    # translating to code
    self.keep_message_history = [True]
    self.response_processors = [
        self.process_enumerator_response,
        # self.process_coder_response,
    ]
    # self.code_executor = self.execute_code

  # process the response from thinker, the output will be used as input to coder
  # TODO: figure out parsing, structure
  def process_enumerator_response(self, response: str) -> str:
    try:
      command_enumeration = (
          re.split(
              "end of enumeration",
              re.split("start of enumeration", response, flags=re.IGNORECASE)[
                  1
              ],
              flags=re.IGNORECASE,
          )[0]
          .strip("[")
          .strip("]")
          .strip()
          .strip("```")
      )
      print("processing command enumeration")
      return command_enumeration
    except Exception as _:  # pylint: disable=broad-exception-caught
      return response
