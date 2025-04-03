"""
[start of description]

    This is not a new grasp.
    This grasp should be delicate and requires accurate force control to avoid damaging the yuba while ensuring a stable grip.
    This is a complete grasp.
    This grasp does not contain multiple grasps.
    This grasp is for an object with high compliance.
    This grasp is for an object with low weight.
    The object has an approximate mass of 20 grams
    The object has an approximate spring constant of 200 Newtons per meter.
    The gripper and object have an approximate friction coefficient of 0.5
    This grasp should set the goal aperture to 25.0 mm.
    If the gripper slips, this grasp should close an additional 2.0 mm.
    Based on object mass and friction coefficient, grasp should initially set the force to 0.39 Newtons.
    If the gripper slips, this grasp should increase the output force by 0.40 Newtons. [end of description]"""

from magpie.gripper import Gripper  # must import the gripper class
import numpy as np  # import numpy because we are using it below

G = Gripper()  # create a gripper object

# This is a new grasp, so reset parameters to default 
# to avoid shattering the delicate yuba, which is dried tofu skin.
G.reset_parameters()

# Grasp parameters
new_task = True
complete_grasp = True
goal_aperture = 25  # set the goal aperture to 25 mm
mass = 0.02  # object's approximate mass is 20 grams
spring_constant = 200  # object's approximate spring constant is 100 N/m
mu = 0.5  # the gripper and object's approximate friction coefficient is 0.5
additional_closure = 2  # if the gripper slips, it should close an additional 2 mm

# Based on object mass and friction coefficient, initially set the force to 0.2 N
initial_force = (mass * 9.81) / mu
initial_force = max(min(initial_force, 16), 0.1)  # Ensure the force is within the gripper's capability

# If the gripper slips, increase the output force by 0.2 N
additional_force = 0.04

# Move quickly to a safe aperture 3 mm wider than the goal aperture
G.set_goal_aperture(goal_aperture + 3, finger='both', record_load=False)

# Set compliance for a high-compliance, low-weight object
G.set_compliance(1, 3, finger='both')
G.set_force(initial_force, 'both')
load_data = G.set_goal_aperture(goal_aperture - additional_closure, finger='both', record_load=True)

# Check for slip and iterate adjustment as necessary
curr_aperture = G.get_aperture(finger='both')
applied_force = initial_force
slippage, avg_force, max_force = G.check_slip(load_data, initial_force, 'both')

prev_aperture = curr_aperture
k_avg = []  # To store spring constants over slip detection

while slippage:
    goal_aperture = curr_aperture - additional_closure
    if avg_force > 0.1:  # Apply additional force only if there's contact
        applied_force += additional_force
    G.set_force(applied_force, 'both')
    print(f"Previous aperture: {curr_aperture} mm, Goal Aperture: {goal_aperture} mm, Applied Force: {applied_force} N.")
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)
    
    # Report data after each adjustment
    curr_aperture = G.get_aperture(finger='both')
    slippage, avg_force, max_force = G.check_slip(load_data, initial_force, 'both')

    # Record spring constants over slip detection
    distance = abs(curr_aperture - prev_aperture)
    k_avg.append(avg_force * distance * 1000.0)
    prev_aperture = curr_aperture

if complete_grasp:
    curr_aperture = G.get_aperture(finger='both')
    print(f"Final aperture: {curr_aperture} mm, Controller Goal Aperture: {goal_aperture} mm, Applied Force: {applied_force} N.")
    # Perform a final squeeze for secure grasp
    G.set_goal_aperture(curr_aperture - 2, finger='both', record_load=False)
else:
    G.open_gripper()  # Open the gripper if not a complete grasp