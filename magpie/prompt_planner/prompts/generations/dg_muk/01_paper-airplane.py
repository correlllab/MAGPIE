desc = """
[start of description]

    This is a new grasp.
    This grasp should be gentle and precise to avoid damaging the structure of the paper airplane.
    This is a complete grasp.
    This grasp does not contain multiple grasps.
    This grasp is for an object with high compliance.
    This grasp is for an object with low weight.
    The object has an approximate mass of 5.0 grams
    The object has an approximate spring constant of 20.0 Newtons per meter.
    The gripper and object have an approximate friction coefficient of 0.5
    This grasp should set the goal aperture to 30.0 mm.
    If the gripper slips, this grasp should close an additional 2.0 mm.
    Based on object mass and friction coefficient, grasp should initially set the force to 0.098 Newtons.
    If the gripper slips, this grasp should increase the output force by 0.4 Newtons. [end of description]
"""

from magpie.gripper import Gripper  # must import the gripper class
import numpy as np  # import numpy because we are using it below

G = Gripper()  # create a gripper object

# Since this is a new, delicate grasp to avoid damaging the structure of the paper airplane, no specific need to reset parameters is mentioned but understood as best practice
# G.reset_parameters()

# Setting the goal aperture slightly wider initially to ensure the delicate paper airplane isn't damaged on approach
goal_aperture = 30.0

# Initial force determined by object weight and friction coefficient
initial_force = 0.098  # Calculated from (5.0 grams * 9.81 m/s^2) / 0.5 friction coefficient, converted to Newtons

# Additional closure amount and force increase if the gripper slips
additional_closure = 2.0
additional_force_increase = 0.01

# Ensuring there's a gentle and precise approach to hold the paper airplane without damage
# Setting the compliance slightly more flexible due to the high compliance nature of the task
G.set_compliance(1, 4, finger='both')

# Setting the initial force as it's vital in a grasp of a low-weight and highly compliant object like a paper airplane
G.set_force(initial_force, 'both')

# Move to the initial goal aperture to attempt the grasp
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# Check the current aperture and adjust the goal aperture if necessary for a delicate grasp
curr_aperture = G.get_aperture(finger='both')

# Initializing the applied_force variable with the initial_force value
applied_force = initial_force

# Checking for slip at the initial attempt, indicating whether the grip is firm or needs adjustment
slippage = G.check_slip(load_data, initial_force, 'both')

# If there's slippage, the gripper needs more force and possibly a tighter grasp
while slippage:
    goal_aperture = curr_aperture - additional_closure
    applied_force += additional_force_increase
    G.set_force(applied_force, 'both')
    print(f"Previous aperture: {curr_aperture} mm, Goal Aperture: {goal_aperture} mm, Applied Force: {applied_force} N.")
  
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)
  
    # Report data after each adjustment
    curr_aperture = G.get_aperture(finger='both')
    print(f"Current aperture: {curr_aperture} mm")
  
    # It's necessary to check slippage again after readjusting the force and aperture
    slippage = G.check_slip(load_data, applied_force, 'both')

# Confirming the final grasp settings
curr_aperture = G.get_aperture(finger='both')
print(f"Final aperture: {curr_aperture} mm, Controller Goal Aperture: {goal_aperture} mm, Applied Force: {applied_force} N.")