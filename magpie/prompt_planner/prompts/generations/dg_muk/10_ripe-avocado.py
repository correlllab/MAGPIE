"""
[start of description]

    This is a new grasp.
    This grasp should be gentle to avoid damaging the ripe avocado.
    This is a complete grasp.
    This grasp does not contain multiple grasps.
    This grasp is for an object with medium compliance.
    This grasp is for an object with low weight.
    The object has an approximate mass of 200 grams.
    The object has an approximate spring constant of 500 Newtons per meter.
    The gripper and object have an approximate friction coefficient of 0.5.
    This grasp should set the goal aperture to 60 mm.
    If the gripper slips, this grasp should close an additional 2 mm.
    Based on object mass and friction coefficient, grasp should initially set the force to 3.924 Newtons.
    If the gripper slips, this grasp should increase the output force by 0.1 Newtons. [end of description]
"""

from magpie.gripper import Gripper # must import the gripper class
G = Gripper() # create a gripper object
import numpy as np  # import numpy because we are using it below

# Since this is a new grasp, particularly delicate for a ripe avocado, parameters are reset to default. 
G.reset_parameters()

new_task = True
goal_aperture = 60
complete_grasp = True
initial_force = (200 * 9.81) / 0.5 / 1000  # object weight in grams to kg, then applying F = ma and dividing by mu
additional_closure = 2
additional_force = max(0.05, additional_closure * 500 * 0.0001)  # spring constant (N/m) * displacement (m) * damping factor

# Moving to a safe aperture wider than the goal to avoid immediate contact
G.set_goal_aperture(goal_aperture + 3, finger='both', record_load=False)

# Setting compliance parameters suitable for medium compliance objects like an avocado.
G.set_compliance(1, 3, finger='both') # Error margin set to 1mm with a moderate flexibility

G.set_force(initial_force, 'both')
load_data = G.set_goal_aperture(goal_aperture - additional_closure, finger='both', record_load=True)

curr_aperture = G.get_aperture(finger='both')
applied_force = initial_force
slippage, avg_force, max_force = G.check_slip(load_data, initial_force, 'both')

# Record spring constants over slip detection period
prev_aperture = curr_aperture
k_avg = []

while slippage:
  goal_aperture = curr_aperture - additional_closure
  if np.mean(avg_force) > 0.10: # Increase force only if there's contact
    applied_force += additional_force
  G.set_force(applied_force, 'both')
  print(f"Previous aperture: {curr_aperture} mm, Goal Aperture: {goal_aperture} mm, Applied Force: {applied_force} N.")
  load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)
  
  # Reporting data after each adjustment
  curr_aperture = G.get_aperture(finger='both')
  slippage, avg_force, max_force = G.check_slip(load_data, initial_force, 'both') # Updating to check for the new force

  # Recording spring constant over slip detection
  distance = abs(curr_aperture - prev_aperture)
  k_avg.append(np.mean(avg_force) * distance * 1000.0) # Converting distance to meters for spring constant calculation
  prev_aperture = curr_aperture

if complete_grasp:
    curr_aperture = G.get_aperture(finger='both')
    G.set_goal_aperture(curr_aperture - 2, finger='both', record_load=False) # Final grasp tightening without load recording
    print(f"Final aperture: {curr_aperture - 2} mm, Controller Goal Aperture: {goal_aperture} mm, Applied Force: {applied_force} N.")
else:
    G.open_gripper() # If not a complete grasp, release the object