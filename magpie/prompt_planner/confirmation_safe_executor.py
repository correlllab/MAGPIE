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

"""An alternative SafeExecutor implementation which asks for user confirmation.

This should only be used in platforms where the sandbox is not available, e.g.
on OS X.
"""

import os
import subprocess
import tempfile
import termcolor
import sys
import platform
sys.path.append('../../')

import magpie.prompt_planner.safe_executor as safe_executor
# import safe_executor


def default_interpreter() -> str:  
  if platform.system() == "Windows":
    return os.path.dirname(sys.executable) + "/python.exe"
  # return os.getenv("HOME") + "/miniconda3/envs/l2r-go1/bin/python3.11"
  return sys.executable


_SERIOUS_WARNING = (
    "\nYou are about to execute untrusted code.\n"
    "Code executed this way can perform any operation on your PC, and "
    "can be a security risk."
    '\nOnce you have reviewed the code above, type "yes" to continue.\n'
)

_REPEATED_WARNING = '\nAbout to execute the code above. Type "y" to continue.\n'


class ConfirmationSafeExecutor(safe_executor.SafeExecutor):
  """An executor that asks for user confirmation before executing the code."""

  def __init__(self, interpreter_path=None, skip_confirmation=False, local_execute=False):
    super().__init__()
    self._confirmed_once = False
    self._interpreter_path = interpreter_path or default_interpreter()
    self._skip_confirmation = skip_confirmation
    self._local_execute = local_execute

  def safe_execute(self, code: str) -> str:
    if not self._confirmed_once:
      while not self._skip_confirmation:
        confirm = input(
            termcolor.colored(_SERIOUS_WARNING, "red", attrs=["bold"])
        )
        if confirm.lower() == "yes":
          break
      self._confirmed_once = True
    else:
      while not self._skip_confirmation:
        confirm = input(
            termcolor.colored(_REPEATED_WARNING, "red", attrs=["bold"])
        )
        if confirm.lower() in ("y", "yes"):
          break
    return self._execute(code)

  def _execute(self, code: str) -> str:
    print("now im really ABOUT TO EXECUTE")
    f = tempfile.NamedTemporaryFile(suffix=".py", mode="w", delete=False)

    f.write(code)
    f.close()
    # also write f to local filepath, generations/<filename>
    local_py = None
    with open(f'generations/{os.path.basename(f.name)}', 'w') as gen_file:
      gen_file.write(code)
      local_py = gen_file.name
    filepath = f.name

    # Start by compiling the code to pyc (to get compilation errors)
    try:
      # subprocess.run(
      #     [self._interpreter_path, "-m", "compileall", "-b", filepath],
      #     check=True,
      # )
      # subprocess.run(["/usr/bin/python3", "-m", "compileall", "-b", "-d", "__pycache__", "test.py"], check=True,)
      subprocess.run(
          [self._interpreter_path, "-m", "py_compile", filepath],
          check=True,
      )
    except subprocess.CalledProcessError as e:
      raise ValueError("Failed to compile code.") from e
    finally:
      os.unlink(filepath)

    # py_compile should output a pyc file in the pycache directory
    # filename = os.path.basename(filepath)
    filename = os.path.splitext(os.path.basename(filepath))[0]
    directory = os.path.dirname(filepath)
    pycache_dir = os.path.join(directory, "__pycache__")
    # pyc_filepath = os.path.join(pycache_dir, filename + ".cpython-310.pyc")
    pyc_filepath = os.path.join(pycache_dir, filename + ".cpython-310.pyc")
    # pyc_filepath = os.path.join(pycache_dir, filename + "c")
    # also write pyc to local filepath, generations/<pycname>
    local_pyc = None
    with open(f'generations/{os.path.basename(pyc_filepath)}', 'w') as gen_file:
      gen_file.write(code)
      local_pyc = gen_file.name
    
    # Now execute the pyc file
    try:
      completed_process = subprocess.run(
          # [self._interpreter_path, pyc_filepath],
          # [self._interpreter_path, local_pyc],
          [self._interpreter_path, local_py if self._local_execute else pyc_filepath],
          capture_output=True,
          check=True,
      )
    except subprocess.CalledProcessError as e:
      print("stdout", e.stdout)
      print("stderr", e.stderr)
      raise ValueError("Failed to run code.") from e
    finally:
      os.unlink(pyc_filepath)
    print("completed_process", completed_process.stdout.decode("utf-8"))
    return completed_process.stdout.decode("utf-8")

if __name__ == "__main__":
  pass