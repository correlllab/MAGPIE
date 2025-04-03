"""
check an avocado for ripeness
mp_thinker_muk

[start of description]

    This is not a new grasp.
    This grasp should be lightly pressing on the avocado to assess its compliance without causing damage.
    This is an incomplete grasp.
    This grasp does not contain multiple grasps.
    This grasp is for an object with medium compliance.
    This grasp is for an object with low weight.
    The object has an approximate mass of 200 grams.
    The object has an approximate spring constant of 1000 Newtons per meter due to needing a more accurate measurement of its compliance while being gentle.
    The gripper and object have an approximate friction coefficient of 0.8.
    This grasp should set the goal aperture to 60 mm.
    If the gripper slips, this grasp should close an additional 1 mm.
    Based on object mass and friction coefficient, grasp should initially set the force to 2.45 Newtons.
    If the gripper slips, this grasp should increase the output force by 0.1 Newtons.
    This grasp sets the initial force to a different initial force 1 Newton because of requiring gentle pressing to assess ripeness without causing indentations. [end of description]


"""

from magpie.gripper import Gripper # must import the gripper class
import numpy as np  # import numpy because we are using it below

G = Gripper() # create a gripper object

# Since this is not a new grasp, there is no need to reset parameters.

# Setting the goal aperture to a wider position initially to ensure safe closure towards the actual size of the avocado.
goal_aperture = 60  # As described
# The delicate nature of this task requires gentle pressing, hence setting a different initial force.
initial_force = 1  # Altered initial force due to gentle pressing requirements

# Move quickly (without recording load) to an aperture that is slightly wider than the goal aperture for safety
G.set_goal_aperture(goal_aperture + 5, finger='both', record_load=False)

# Set compliance to medium values to start with a gentle approach that allows for some positional adjustment without causing damage.
G.set_compliance(1, 3, finger='both')
# Set the force to the minimum initially specified to avoid causing damage when assessing the avocado's ripeness.
G.set_force(initial_force, 'both')
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# Check the current aperture and adjust the goal aperture if necessary.
curr_aperture = G.get_aperture(finger='both')

# Initial applied_force and slip_threshold should be equal to initial_force.
applied_force = initial_force
slip_threshold = initial_force

# Initially check for slippage with the set slip threshold
slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

# As described, if the gripper slips, close an additional 1mm and increase force by 0.1N.
while slippage:
    goal_aperture = curr_aperture - 1  # closing an additional 1 mm
    applied_force += 0.1  # increase the output force by 0.1 Newtons
    G.set_force(applied_force, 'both')
    print(f"Adjusting: Goal Aperture: {goal_aperture} mm, Applied Force: {applied_force} N.")
    
    # Recording load data for feedback.
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)
    
    # Rechecking current aperture and slip condition with updated values
    curr_aperture = G.get_aperture(finger='both')
    slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

goal_aperture = curr_aperture  # reassign the goal aperture to the current after completing the slip check

# Final condition for incomplete grasp, leaving the gripper partially closed around the avocado for possible further interaction.
print(f"Final assessment: Aperture: {goal_aperture} mm, Applied Force: {applied_force} N.")