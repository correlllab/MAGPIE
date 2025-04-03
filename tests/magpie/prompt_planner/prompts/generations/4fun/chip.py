desc = """
[start of description]

This is a new grasp.
This grasp should be for picking a fragile potato chip with careful force application to avoid breakage.
This is a complete grasp.
This grasp does not contain multiple grasps.
This grasp is for an object with low compliance.
This grasp is for an object with low weight.
The object has an approximate mass of 5.0 grams
The object has an approximate spring constant of 50.0 Newtons per meter.
The gripper and object have an approximate friction coefficient of 0.3
This grasp should set the goal aperture to 30.0 mm.
If the gripper slips, this grasp should close an additional 0.5 mm.
Based on object mass and friction coefficient, grasp should initially set the force to 0.16 Newtons.
If the gripper slips, this grasp should increase the output force by 0.05 Newtons.
This grasp does not use the default minimum grasp force force.
This grasp sets the initial force to a different initial force 0.16 because of the object's fragile nature and the need for delicate handling to avoid breakage. [end of description]

"""

from magpie.gripper import Gripper # must import the gripper class
import numpy as np  # import numpy because we are using it below

G = Gripper() # create a gripper object

# Reset parameters since this is a new grasp, especially for a delicate object like a potato chip.
G.reset_parameters()

# This grasp is for a fragile potato chip requiring careful handling to avoid breakage.

goal_aperture = 40.0
complete_grasp = True
initial_force = 0.16  # Given the object's fragile nature and considering its low weight and grip requirements.
additional_closure = 0.5  
additional_force = 0.05  # To be used if slippage is detected to increase gripping force carefully.

# Move quickly (without recording load) to a safe aperture slightly wider than the goal aperture,
# ensuring enough space to accommodate the potato chip without applying any force initially.
G.set_goal_aperture(goal_aperture + 3, finger='both', record_load=False)

# Setting compliance to have slight flexibility avoiding damage to the chip,
# considering that it's a non-compliant and fragile object.
G.set_compliance(1, 3, finger='both')

# Set the initial force to be gentle enough to not break the chip on contact.
G.set_force(initial_force, 'both')

# Attempting initial grasp with a goal aperture slightly smaller than the chip to secure it without exerting excessive force.
load_data = G.set_goal_aperture(goal_aperture - additional_closure, finger='both', record_load=True)

# Getting the current aperture to confirm the gripped state and initializing variables for slip detection.
curr_aperture = G.get_aperture(finger='both')
applied_force = initial_force
slip_threshold = initial_force

# Check for potential slip. Slippage here would indicate that the grasp force was insufficient to hold the chip securely.
slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

# If slippage is detected, the loop adjusts the force and aperture to securely grip the potato chip.
while slippage:
  goal_aperture = curr_aperture - additional_closure
  if np.mean(avg_force) > 0.10: # increase force only if there has been some contact indicating insufficient grip.
    applied_force += additional_force
  G.set_force(applied_force, 'both')
  print(f"Adjusting grip: Previous aperture: {curr_aperture:.2f} mm, Goal Aperture: {goal_aperture:.2f} mm, Applied Force: {applied_force:.2f} N.")
  load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)
  
  # Check grip status after the adjustment.
  curr_aperture = G.get_aperture(finger='both')
  slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

if complete_grasp: # Finalize the grasp by setting to the current aperture, ensuring the chip is held securely for transport.
  curr_aperture = G.get_aperture(finger='both')
  G.set_goal_aperture(curr_aperture - additional_closure, finger='both', record_load=False)
  print(f"Grasp completed. Final aperture: {curr_aperture:.2f} mm, Applied Force: {applied_force:.2f} N.")
else: # Open the gripper if the task doesn't complete the grasp for some reason
  G.open_gripper()