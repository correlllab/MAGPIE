desc = """
[start of description]

This is a new grasp.
This grasp should be gentle to not crush the soft and slightly compliant texture of mozzarella cheese while ensuring a secure grasp.
This is a complete grasp.
This grasp does not contain multiple grasps.
This grasp is for an object with medium compliance.
This grasp is for an object with low weight.
The object has an approximate mass of 200 grams
The object has an approximate spring constant of 50.0 Newtons per meter.
The gripper and object have an approximate friction coefficient of 0.8
This grasp should set the goal aperture to 50.0 mm.
If the gripper slips, this grasp should close an additional 5.0 mm.
Based on object mass and friction coefficient, grasp should initially set the force to 2.45 Newtons.
If the gripper slips, this grasp should increase the output force by 0.025 Newtons. [end of description]

"""
# actual mass: 226g

from magpie.gripper import Gripper # must import the gripper class
import numpy as np  # import numpy because we are using it below

G = Gripper() # create a gripper object

# Reset parameters to default since this is a new, delicate grasp to avoid crushing the mozzarella cheese.
G.reset_parameters()

# Describing the need for a gentle but secure grip on the slightly compliant mozzarella cheese without crushing it. 
# The goal aperture is set based on the approximate size of the mozzarella cheese.
goal_aperture = 50.0 

# Since this grasp is for a complete hold on a softly compliant object, we set the compliance to moderate values. 
# This allows for some give in the event of slight miscalculations in size or force, protecting the texture of the cheese.
G.set_compliance(1, 3, finger='both')

# Setting the initial force based on the object's weight and friction coefficient. 
# This force is calculated to be just enough to lift the object without slipping.
initial_force = 2.45 
G.set_force(initial_force, 'both')

# Moving quickly to a safe aperture wider than the goal, to ensure a gentle but secure approach.
G.set_goal_aperture(goal_aperture + 3, finger='both', record_load=False)

# Predicting that this setting of compliance and force will lead to a grip that is secure 
# yet gentle enough not to damage the mozzarella cheese. We are attempting to achieve a "lock" on the cheese without excess pressure.
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# Current aperture and applied forces need to be monitored for maintaining a secure grasp without slipping.
curr_aperture = G.get_aperture(finger='both')
applied_force = initial_force
slip_threshold = initial_force

# Checking for slippage at the initial force. If slipping occurs, it indicates the grip needs adjustment.
slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

# Adjusting the gripper's force and aperture based on detected slippage, enhancing the grip without compromising the cheese's integrity.
additional_closure = 5.0 
additional_force = 0.025 

# Recording the spring constants over slip detection to ensure consistency in force adjustments and compliance.
prev_aperture = curr_aperture
k_avg = []

while slippage:
  goal_aperture = curr_aperture - additional_closure 
  if np.mean(avg_force) > 0.10: # Ensuring force is only increased if there's confirmed contact, avoiding unnecessary force buildup.
    applied_force += additional_force
  G.set_force(applied_force, 'both')
  print(f"Previous aperture: {curr_aperture} mm, Goal Aperture: {goal_aperture} mm, Applied Force: {applied_force} N.")
  load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)
  
  curr_aperture = G.get_aperture(finger='both')
  slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

  distance = abs(curr_aperture - prev_aperture)
  k_avg.append(np.mean(avg_force) * distance * 1000.0) # capturing spring constant across adjustments.
  prev_aperture = curr_aperture

# Complete the grasp by finalizing the position without further recordings, 
# ensuring a finished and steady hold on the mozzarella cheese.
curr_aperture = G.get_aperture(finger='both')
G.set_goal_aperture(curr_aperture, finger='both', record_load=False)
print(f"Final aperture: {curr_aperture} mm, Applied Force: {applied_force} N, secures the mozzarella without slipping or crushing.")