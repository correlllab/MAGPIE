"""[start of description]

    This is a new grasp.
    This grasp should be a delicate pinch around the top of the raspberry without crushing it.
    This is a complete grasp.
    This grasp does not contain multiple grasps.
    This grasp is for an object with high compliance.
    This grasp is for an object with low weight.
    The object has an approximate mass of 5.0 grams
    The object has an approximate spring constant of 50.0 Newtons per meter.
    The gripper and object have an approximate friction coefficient of 0.8
    This grasp should set the goal aperture to 18.0 mm.
    If the gripper slips, this grasp should close an additional 1.0 mm.
    Based on object mass and friction coefficient, grasp should initially set the force to 0.0625 Newtons.
    If the gripper slips, this grasp should increase the output force by 0.05 Newtons.
    This grasp does use the default minimum grasp force force. [end of description]"""

from magpie.gripper import Gripper # must import the gripper class
G = Gripper() # create a gripper object
import numpy as np  # import numpy because we will use it below

# Since this is a new, delicate grasp, reset parameters to default to avoid crushing the raspberry.
G.reset_parameters()

# This grasp is for a delicate pinch, so a precise goal aperture is required, set at 18.0 mm.
goal_aperture = 18.0
complete_grasp = True  # This is a complete grasp.

# Given the object's low weight and the given friction coefficient, we start with an initial force of 0.0625 N.
initial_force = 0.0625
additional_closure = 1.0  # If there is slippage detected, the grasp should close an additional 1.0 mm.

# The specified increase in output force if slippage is detected is 5.0 N.
additional_force = 0.01

# Move quickly without recording load to a safe aperture that is slightly wider than the goal to prevent initial contact
G.set_goal_aperture(goal_aperture + 3, finger='both', record_load=False)

# Set compliance and initial force for a delicate grasp, especially important for high compliance and low weight objects.
G.set_compliance(1, 3, finger='both')
G.set_force(initial_force, 'both')
load_data = G.set_goal_aperture(goal_aperture - additional_closure, finger='both')

# Check for any slippage at the current grip state, adjusting as necessary.
curr_aperture = G.get_aperture(finger='both')
applied_force = initial_force
slippage, avg_force, max_force = G.check_slip(load_data, initial_force, 'both')

prev_aperture = curr_aperture
k_avg = []  # To record the spring constants over slip detection

while slippage:
    goal_aperture = curr_aperture - additional_closure
    if np.mean(avg_force) > 0.1:  # Ensure there's contact before increasing the applied force.
        applied_force += additional_force
    G.set_force(applied_force, 'both')
    print(f"Previous aperture: {curr_aperture} mm, Goal Aperture: {goal_aperture} mm, Applied Force: {applied_force} N.")
    load_data = G.set_goal_aperture(goal_aperture, finger='both')

    # Update data after each adjustment for reporting and decision-making.
    curr_aperture = G.get_aperture(finger='both')
    slippage, avg_force, max_force = G.check_slip(load_data, applied_force, 'both')

    # Calculate and record the average spring constants over detected slips
    distance = abs(curr_aperture - prev_aperture)
    k_avg.append(np.mean(avg_force) * distance * 1000.0)
    prev_aperture = curr_aperture

if complete_grasp:
    # Final confirmation of grip details
    curr_aperture = G.get_aperture(finger='both')
    print(f"Final aperture: {curr_aperture} mm, Controller Goal Aperture: {goal_aperture} mm, Applied Force: {applied_force} N.")
    G.set_goal_aperture(curr_aperture - 2, finger='both', record_load=False)
else:
    # If the grasp wasn't meant to be complete, open the gripper.
    G.open_gripper()