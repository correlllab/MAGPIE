desc = """
[start of description]

This is not a new grasp.
This grasp should be adjusted for a delicate, light object with an oval shape, like an egg.
This is a complete grasp.
This grasp does not contain multiple grasps.
This grasp is for an object with low compliance.
This grasp is for an object with low weight.
The object has an approximate mass of 50 grams.
The object has an approximate spring constant of 250 Newtons per meter.
The gripper and object have an approximate friction coefficient of 0.3.
This grasp should set the goal aperture to 35.0 mm.
If the gripper slips, this grasp should close an additional 2.0 mm.
Based on object mass and friction coefficient, grasp should initially set the force to 1.63 Newtons.
If the gripper slips, this grasp should increase the output force by 0.05 Newtons. [end of description]

"""

from magpie.gripper import Gripper # must import the gripper class
import numpy as np  # import numpy because we are using it below

G = Gripper()  # create a gripper object

# Since this is not a new grasp and is for a delicate, light object like an egg, 
# there is no need to reset parameters. Instead, careful adjustments are made 
# to handle the object's low compliance and low weight with precision.

goal_aperture = 35.0
complete_grasp = True
initial_force = 1.63  # calculated from object weight and friction coefficient
additional_closure = 2.0
additional_force = 0.05  # force increase upon slippage

# Move quickly (without recording load) to a slightly wider aperture than the goal 
# to safely approach the object without immediate contact.
G.set_goal_aperture(goal_aperture + 5, finger='both', record_load=False)

# Setting compliance with a small margin for a precise grasp, and moderate flexibility
# to mitigate the risk of damaging the low compliance, delicate object.
G.set_compliance(0.5, 4, finger='both')

# Setting the initial force based on calculated value to achieve a secure grip without
# exerting excessive force on the light and delicate object.
G.set_force(initial_force, 'both')

# Attempting to reach the goal aperture, taking into consideration the possibility of object deformation.
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# Initial check for slippage to ensure secure grasp
curr_aperture = G.get_aperture(finger='both')
applied_force = initial_force
slip_threshold = initial_force
slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

prev_aperture = curr_aperture
k_avg = []

# Iteratively adjust aperture and force if slippage is detected, ensuring gentle handling.
while slippage:
    goal_aperture = curr_aperture - additional_closure
    applied_force += additional_force  # Increasing force slightly to enhance grip without overloading
    
    G.set_force(applied_force, 'both')
    print(f"Previous aperture: {curr_aperture:.2f} mm, New Goal Aperture: {goal_aperture:.2f} mm, Applied Force: {applied_force:.2f} N.")
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

    curr_aperture = G.get_aperture(finger='both')
    slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

    # Calculate and record the effective spring constant over iterations using distance moved and force applied.
    distance = abs(curr_aperture - prev_aperture)
    if distance > 0:
        k_avg.append(np.mean(avg_force) / distance * 1000.0)  # conversion factor for units
    prev_aperture = curr_aperture

if complete_grasp:
    # Upon ensuring no slip, make a final adjustment for a complete grasp, if necessary.
    curr_aperture = G.get_aperture(finger='both')
    G.set_goal_aperture(curr_aperture - additional_closure, finger='both', record_load=False)
    print(f"Final adjustment made for complete grasp: Current Aperture: {curr_aperture:.2f} mm, Applied Force: {applied_force:.2f} N.")