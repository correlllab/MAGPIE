"""
[start of description]

    This is a new grasp.
    This grasp should be gentle to avoid damaging the fruit.
    This is a complete grasp.
    This grasp does not contain multiple grasps.
    This grasp is for an object with medium compliance.
    This grasp is for an object with low weight.
    The object has an approximate mass of 150 grams.
    The object has an approximate spring constant of 500 Newtons per meter.
    The gripper and object have an approximate friction coefficient of 0.8.
    This grasp should set the goal aperture to 50 mm.
    If the gripper slips, this grasp should close an additional 2 mm.
    Based on object mass and friction coefficient, grasp should initially set the force to 1.88 Newtons.
    If the gripper slips, this grasp should increase the output force by 0.1 Newtons. [end of description]
"""

from magpie.gripper import Gripper  # Importing the Gripper class
import numpy as np  # Importing numpy for calculations

G = Gripper()  # Creating a new Gripper object

# Since this is a new grasp, we reset parameters to ensure a gentle grasp to avoid damaging the fruit.
if True:  # New grasp indicated
    G.reset_parameters()

# Setting initial conditions based on grasp requirements
goal_aperture = 50
complete_grasp = True
initial_force = (150 * 9.81) / 0.8 / 1000  # Calculating the initial force as weight (in N) divided by friction coefficient
additional_closure = 2  # Additional closure if gripper slips
additional_force = 0.1  # Increase in force if gripper slips

# Strategy: Move quickly to a position slightly wider than the goal aperture to ensure safe approach.
G.set_goal_aperture(goal_aperture + 3, record_load=False)

# Setting compliance and initial force for a medium compliance object
# Compliance margin(1mm) and slope settings are chosen based on desired flexibility and gentleness for the fruit.
G.set_compliance(1, 3)
G.set_force(initial_force)

# Moving to the target aperture minus additional closure to preemptively compensate for any potential slip.
load_data = G.set_goal_aperture(goal_aperture)

# Checking the current aperture and setting initial parameters for slip detection.
curr_aperture = G.get_aperture()
applied_force = initial_force
slip_threshold = initial_force

# Checking for slip and acting accordingly
slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold)

# Loop for adjusting grip based on slippage
while slippage:
    goal_aperture = curr_aperture - additional_closure  # Adjusting the goal aperture based on the previous slip.
    if np.mean(avg_force) > 0.1:  # Ensuring there's significant force to adjust
        applied_force += additional_force  # Incrementing the applied force to improve grip.
    G.set_force(applied_force)  # Setting the new increased force on the gripper
    
    print(f"Previous aperture: {curr_aperture} mm, Goal Aperture: {goal_aperture} mm, Applied Force: {applied_force} N.")
    load_data = G.set_goal_aperture(goal_aperture)  # Attempting the grasp again with the new parameters.
    
    curr_aperture = G.get_aperture()
    slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold)  # Checking for slip again
    
    # If there's no slippage, we exit the loop and complete the grasp. Otherwise, we retry with adjusted settings.


# Completing the grasp after successfully preventing slippage
if complete_grasp:
    curr_aperture = G.get_aperture()
    G.set_goal_aperture(curr_aperture - 2, record_load=False)
    print(f"Final aperture: {curr_aperture - 2} mm, Applied Force: {applied_force} N.")
else:
    G.open_gripper()