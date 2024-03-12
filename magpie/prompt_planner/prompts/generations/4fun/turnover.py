desc = """
[start of description]

This is not a new grasp.
This grasp should be gentle, to avoid squishing the raspberry turnover, while ensuring a secure hold.
This is a complete grasp.
This grasp does not contain multiple grasps.
This grasp is for an object with high compliance.
This grasp is for an object with low weight.
The object has an approximate mass of 40 grams.
The object has an approximate spring constant of 50 Newtons per meter.
The gripper and object have an approximate friction coefficient of 0.3.
This grasp should set the goal aperture to 60.0 mm.
If the gripper slips, this grasp should close an additional 2.0 mm.
Based on object mass and friction coefficient, grasp should initially set the force to 1.31 Newtons.
If the gripper slips, this grasp should increase the output force by 0.1 Newtons. [end of description]
"""

from magpie.gripper import Gripper  # import the gripper class
import numpy as np  # import numpy because we are using it below

G = Gripper()  # create a gripper object

# Since this is not a new grasp, we skip resetting parameters and instead proceed directly to the grasp logic.
# Setting the goal aperture to 60 mm. This is slightly bigger than the object to ensure a gentle contact.
goal_aperture = 60.0
# This is a complete grasp, meaning we aim to securely hold the raspberry turnover without multiple grasping attempts.
complete_grasp = True
# Initial force calculated based on object weight and friction coefficient is 1.31 Newtons.
initial_force = 1.31
# If the gripper slips, it should close an additional 2.0 mm.
additional_closure = 2.0
# If the gripper slips, it should increase the output force by 0.1 Newtons.
additional_force = 0.1

# Move quickly (without recording load) to a safe aperture that is slightly wider than the goal aperture to ensure a gentle approach.
G.set_goal_aperture(goal_aperture + 5, finger='both', record_load=False)

# Setting a compliance margin and flexibility to adapt to the high compliance of the object.
# This allows the gripper to adjust its force and position more softly, preventing damage to the raspberry turnover.
G.set_compliance(1, 3, finger='both')
# Setting initial force to ensure the grasp is gentle yet firm enough to hold the low weight, high compliance object without slipping.
G.set_force(initial_force, 'both')
# Now moving to the goal aperture minus the additional closure, to ensure a secure hold.
load_data = G.set_goal_aperture(goal_aperture, finger='both')

# Checking current aperture after attempting to reach the goal aperture. This is because the actual position might differ due to compliance settings.
curr_aperture = G.get_aperture(finger='both')
# For slip detection, we initialize both applied_force and slip_threshold to the initial force value.
applied_force = initial_force
slip_threshold = initial_force
# Checking if the gripper has slipped by comparing the applied force to the force when contact was made.
slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

# Preparing to adjust grasp based on slip detection results.
while slippage:
    goal_aperture = curr_aperture - additional_closure
    # Increase the applied force if necessary, considering the gripper might not always make contact.
    if np.mean(avg_force) > 0.10:  # Only increase force when there's evidence of contact with the object.
        applied_force += additional_force
    G.set_force(applied_force, 'both')
    print(f"Adjusting. Previous aperture: {curr_aperture} mm, New Goal Aperture: {goal_aperture} mm, Applied Force: {applied_force} N.")
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=False)
  
    # Re-check the aperture and slip after the adjustment.
    curr_aperture = G.get_aperture(finger='both')
    slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

# If the grasp is complete and there's no slippage, finalize the grip.
if complete_grasp:
    # No additional closure is applied here since we're in the loop until no slippage.
    print(f"Final grasp established successfully. Final aperture: {curr_aperture} mm, Applied Force: {applied_force} N.")
else:
    # If not aiming for a complete grasp, the gripper is opened, although this scenario does not apply here as `complete_grasp` is True.
    G.open_gripper()