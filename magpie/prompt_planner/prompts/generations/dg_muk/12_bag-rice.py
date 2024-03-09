"""
[start of description]

    This is a new grasp.
    This grasp should be for gently picking up a bag of rice without puncturing or damaging the bag.
    This is a complete grasp.
    This grasp does not contain multiple grasps.
    This grasp is for an object with low compliance.
    This grasp is for an object with medium weight.
    The object has an approximate mass of 1000 grams.
    The object has an approximate spring constant of 200 N/m.
    The gripper and object have an approximate friction coefficient of 0.5.
    This grasp should set the goal aperture to 80.0 mm.
    If the gripper slips, this grasp should close an additional 5.0 mm.
    Based on object mass and friction coefficient, grasp should initially set the force to 20.0 Newtons.
    If the gripper slips, this grasp should increase the output force by 0.1 Newtons. [end of description]"""

from magpie.gripper import Gripper
import numpy as np

G = Gripper()  # Creating a gripper object for the new task

# Since this is a new grasp for a bag of rice, which requires gentle handling to avoid damage, reset parameters to defaults
G.reset_parameters()

# Reasoning: Setting initial conditions based on the delicate and specific needs of grasping a bag of rice without causing damage.
goal_aperture = 80.0
complete_grasp = True
initial_force = 20.0  # Calculated based on object's mass and friction coefficient, but adjusted due to the necessity for a higher starting force
additional_closure = 5.0
additional_force = 0.1  # Increase in force if the gripper slips, to enhance grip without damaging the bag

# Move quickly to a safe aperture that is slightly wider than the goal aperture to ensure a secure initial grip without exerting unnecessary force
G.set_goal_aperture(goal_aperture + 5, finger='both', record_load=False)  # Adding 5mm for safety margin

# Reasoning: Setting compliance and initial force prepares the gripper for a controlled and adaptive grip.
# Prediction: The gripper will be ready to adapt its grip force within the compliance margin, minimizing risk of damage.
G.set_compliance(1, 3, finger='both')
G.set_force(initial_force, 'both')

# Attempt to grasp the object by closing the gripper to the goal aperture, and record the load data for slip check
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# Reasoning: Checking for slippage immediately helps in early detection and adjustment, securing a successful grasp.
# Prediction: If slippage is detected, the applied force will be adjusted upward, and the gripper will close further.
curr_aperture = G.get_aperture(finger='both')
applied_force = initial_force
slip_threshold = initial_force
slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

# Loop to adjust the grip in case of slippage, ensuring a secure grasp without damaging the bag
while slippage:
  goal_aperture = curr_aperture - additional_closure  # Reduce aperture to increase grip
  applied_force += additional_force  # Increase force slightly to enhance grip without harm
  G.set_force(applied_force, 'both')
  print(f"Adjusting grip: New aperture: {goal_aperture} mm, New applied force: {applied_force} N.")
  load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)
  
  curr_aperture = G.get_aperture(finger='both')
  slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

# If the grasp is set to be a complete grasp, ensure the gripper is stationary at the final aperture
if complete_grasp:
    # No further action required as the final command in the loop already achieves a complete grasp
    print(f"Final grasp established: Aperture at {curr_aperture} mm, Force applied: {applied_force} N.")
else:
    # This part of the code is not reached as complete_grasp is True, but serves as a template for handling different types of grasps
    G.open_gripper()