from magpie.gripper import Gripper # must import the gripper class
import numpy as np  # import numpy for potential operations on load data

# Create a gripper object
G = Gripper()

# Initial setup for lifting a medium-sized plastic water bottle.
# When grasping a plastic water bottle, setting the aperture to 70mm should suffice to wrap around the body.
goal_aperture = 70.0  
# A lower compliance and weight object like a plastic water bottle needs a precise grip,
# but not too tight to avoid damaging it. Starting with a mild force of 2N.
initial_force = 2.5 

# If the gripper slips, it indicates that the grip wasn't tight enough, so closing additional 5mm.
additional_closure = 5.0 
# Similarly, if slippage occurs, the force was insufficient to maintain a grasp.
# Increasing the force by 1N might assist in achieving a more secure grip.
additional_force = 1.0

# Setting initial conditions for the grasp
G.set_force(initial_force, 'both')  # Starting with 2N force for both fingers.

# Attempt to grasp with initial settings
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)
# Record the initial applied force to adjust in the case of slippage.
applied_force = initial_force

# Checking for slip. If slip occurs, adjust aperture and force as specified.
slip_detected = G.check_slip(load_data, initial_force, 'both')
while slip_detected:
    print(f"Slip detected. Adjusting goal aperture from {goal_aperture} to {goal_aperture - additional_closure}mm and force from {applied_force} to {applied_force + additional_force}N.")
    goal_aperture -= additional_closure  # Reducing the goal aperture by 5mm for a tighter grasp.
    applied_force += additional_force  # Increasing the force by 1N for a firmer grip.
    
    # Apply the new force setting
    G.set_force(applied_force, 'both')
    # Attempt a new grasp with updated conditions.
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)
    # Re-check for slip with the new force applied.
    slip_detected = G.check_slip(load_data, applied_force, 'both')

# Final output indicating settings for a successful grasp, with no slip detected.
print(f"Final grasp achieved with aperture: {goal_aperture}mm and force: {applied_force}N.")