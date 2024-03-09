"""
[start of description]

    This is a new grasp.
    This grasp should be gentle and precise to avoid damaging the soft fabric of the stuffed animal's tail.
    This is a complete grasp.
    This grasp does not contain multiple grasps.
    This grasp is for an object with high compliance.
    This grasp is for an object with low weight.
    The object has an approximate mass of 50 grams.
    The object has an approximate spring constant of 100 Newtons per meter.
    The gripper and object have an approximate friction coefficient of 0.8.
    This grasp should set the goal aperture to 25.0 mm.
    If the gripper slips, this grasp should close an additional 2.0 mm.
    Based on object mass and friction coefficient, grasp should initially set the force to 0.61 Newtons.
    If the gripper slips, this grasp should increase the output force by 0.05 Newtons. [end of description]
"""

from magpie.gripper import Gripper  # import the gripper class
G = Gripper()  # create a gripper object
import numpy as np  # import numpy because we will use it below

# As this is a new grasp for a soft and compliant object like a stuffed animal's tail, parameters are set gently.
# The goal is to avoid damaging the object while ensuring a secure grasp.

goal_aperture = 25.0
complete_grasp = True
initial_force = 0.61  # Calculated based on the object's mass and the friction coefficient.
additional_closure = 2.0  
additional_force = 0.05  # Increment force when slippage detected.

# Move quickly (without recording load) to a safe aperture that is wider than the goal aperture.
G.set_goal_aperture(goal_aperture + 5, finger='both', record_load=False)

# Set compliance and force with gentle parameters to avoid damaging the stuffed animal's soft fabric.
G.set_compliance(1, 3, finger='both')  # Setting compliance to allow for some flexibility in grasp.
G.set_force(initial_force, 'both')  # Initial force setting based on object's weight and friction coefficient.

# Predicted state: Gripper fingers are positioned with a wider aperture. The compliance and force are set for a soft grasp.

load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# Check the current aperture after trying to set the goal aperture. To adjust goal aperture if necessary.
curr_aperture = G.get_aperture(finger='both')
applied_force = initial_force
slip_threshold = initial_force

# Initially check if slip has occurred with the predefined slip threshold.
slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

# Loop to adjust force and aperture if slippage is detected. Aims for a firm yet gentle grasp, avoiding damage.
while slippage:
    goal_aperture = curr_aperture - additional_closure
    applied_force += additional_force  # Increase force gently if slippage occurred.

    G.set_force(applied_force, 'both')  # Update force based on slippage detection and feedback.

    # Log and adjust based on new settings and slippage feedback.
    print(f"Adjusting to Goal Aperture: {goal_aperture} mm, with Applied Force: {applied_force} N.")
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)
    
    curr_aperture = G.get_aperture(finger='both')
    slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

# If it's a complete grasp and no slippage, finalize the grasp position with a slight additional closure.
if complete_grasp:
    # Slightly narrow the aperture for a final secure fitting.
    final_aperture = curr_aperture - 1  # A gentler close given the object's compliance.
    G.set_goal_aperture(final_aperture, finger='both', record_load=False)
    print(f"Final adjustment to Aperture: {final_aperture} mm, with Applied Force: {applied_force} N.")
else:
    G.open_gripper()  # If not a full grasp, revert to open gripper.