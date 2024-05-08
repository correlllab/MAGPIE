# Copyright 2023 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""Config file for evaluation and user interaction scripts."""

import dataclasses
from typing import Any

# import task_clients
# import magpie_task_client
# import prompts.mp_prompt_coder_only    as mp_prompt_coder_only
# import prompts.mp_prompt_low_level     as mp_prompt_low_level
# import prompts.mp_prompt_thinker_coder as mp_prompt_thinker_coder

import sys
sys.path.append('../../')
import magpie.prompt_planner.prompts.mp_prompt_thinker_coder_muk as mptc
import magpie.prompt_planner.prompts.mp_prompt_tc_vision as mptcv
import magpie.prompt_planner.prompts.mp_prompt_tc_vision_phys as mptcvp
import magpie.prompt_planner.prompts.mp_prompt_tc_phys as mptcp
import magpie.prompt_planner.prompts.dg_system_executor as dgse
import magpie.prompt_planner.prompts.dg_command_enumerator as dgce

@dataclasses.dataclass(frozen=True)
class TaskConfig:
  client: None
  prompts: dict[str, type[Any]]

# @dataclasses.dataclass(frozen=True)
# class TaskConfig:
#   client: type[task_clients.TaskClient]
#   prompts: dict[str, type[Any]]

ALL_TASKS = {
    'magpie': TaskConfig(
        client=None,
        prompts={
            'thinker_coder': mptc.PromptThinkerCoder,
            'thinker_coder_phys': mptcp.PromptThinkerCoderPhys,
            'thinker_coder_vision': mptcv.PromptThinkerCoderVision,
            'thinker_coder_vision_phys': mptcvp.PromptThinkerCoderVisionPhys
        },
    ),
    'system': TaskConfig(
      client=None,
        prompts={
            'system_executor': dgse.PromptSystemExecutor
        },
    ),
    'command_enumerator': TaskConfig(
        client=None,
            prompts={
                'command_enumerator': dgce.PromptCommandEnumerator
        },
    ),
}

# ALL_TASKS = {
#     'barkour': TaskConfig(
#         client=barkour_l2r_task_client.BarkourClient,
#         prompts={
#             'thinker_coder': bk_prompt_thinker_coder.PromptThinkerCoder,
#             'coder_only': bk_prompt_coder_only.PromptCoder,
#             'low_level': bk_prompt_low_level.PromptLowLevel,
#         },
#     ),
#     'magpie': TaskConfig(
#         client=None,
#         prompts={
#             'thinker_coder': mptc.PromptThinkerCoder,
#             'coder_only': mp_prompt_coder_only.PromptCoder,
#             'low_level': mp_prompt_low_level.PromptLowLevel,
#         },
#     ),
# }
