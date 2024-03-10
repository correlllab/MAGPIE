from magpie.gripper import Gripper  # import the gripper class
import numpy as np  # import numpy for later data manipulation

G = Gripper()  # create a gripper object

# Reset parameters to default since this is a new, delicate grasp to avoid crushing the raspberry.
# The goal is to be gentle and precise, given the high compliance and low weight of the object.
goal_aperture = 30  # The initial goal aperture is set to 30mm to softly enclose the raspberry without squishing it.

# Set the gripper to have a soft touch by setting the initial gripping force very low.
# Additionally, enable compliance to make the gripper adaptable to minor size variations of the raspberry.
G.set_compliance(1, 3, finger='both')  # A small margin with a middle-ground flexibility is chosen for compliance.
initial_force = 0.20  # Initial force is set to be 0.15 Newtons to avoid damaging the delicate object.
G.set_force(initial_force, 'both')  # Apply the initial gentle force to both fingers.

# The anticipated action below will attempt to grasp the raspberry at the set aperture and force.
# There's an expectancy to achieve a gentle, safe grasp around the raspberry without exerting too much pressure.
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# Check the current aperture after setting the goal to ensure proper grip without damaging the raspberry.
curr_aperture = G.get_aperture(finger='both')

# If the gripper slips, indicated by check_slip, the aperture and force are adjusted incrementally.
# The strategy aims to enhance the grip securely without compromising the raspberry's integrity.
additional_closure = 1  # Amount to decrease the aperture by if slipping occurs.
additional_force_increase = 0.1  # Amount to increase the force by if slipping occurs.
while G.check_slip(load_data, initial_force, 'both'):
    # If a slip is detected, the aperture decreases slightly to attempt a firmer grip.
    goal_aperture -= additional_closure
    # Concomitantly, the grip force is moderately increased to try securing the raspberry better without harm.
    initial_force += additional_force_increase
    # Apply the adjusted force setting to both fingers.
    G.set_force(initial_force, 'both')
    # Re-attempt the grasp with modified settings aiming for a successful, delicate grip.
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)
    # Re-evaluate the current aperture to confirm the attainment of a more stable grip without slippage.
    curr_aperture = G.get_aperture(finger='both')