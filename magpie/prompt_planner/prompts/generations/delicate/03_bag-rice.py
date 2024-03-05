from magpie.gripper import Gripper  # import the gripper class
import numpy as np  # import numpy for later data manipulation

G = Gripper()  # create a gripper object

# This task focuses on safely handling a plastic bag filled with rice, aiming to secure the bag without puncturing it.
# The plastic bag represents an object with low compliance, meaning it won't easily deform under the gripper's pressure.
goal_aperture = 105  # Setting the initial aperture to 105mm, the maximum, to accommodate the widest part of the bag.

# Adjusting compliance to ensure that even though the object has low compliance, the grip does not damage it.
# A lower margin and higher flexibility are chosen to accommodate the non-deformable nature of the plastic bag.
G.set_compliance(2, 5, finger='both')  # The parameters allow for a gentle grip adjustment to avoid bag damage.

initial_force = 1.5  # Starting with a force of 1.5 Newtons, ensuring the grip is firm enough to lift the medium-weight rice bag without slipping.
G.set_force(initial_force, 'both')  # This initial force is applied to both fingers and should not puncture the plastic.

# Attempting the grasp with the initial settings while enabling load recording to identify if adjustments are necessary.
# The goal is to balance the grip strength to lift the bag successfully without risking punctures.
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# If the gripper slips, indicating that the initial grip wasn't sufficient, the grasp settings are iteratively adjusted.
additional_closure = 3  # The aperture is reduced by 3mm if slippage is detected, aiming for a tighter grip.
additional_force_increase = 0.5  # If slippage occurs, the force increases by 0.5 Newtons to reinforce the grip without risking damage.

# Following the initial grasp attempt, adjustments are made based on the feedback from the slippage check.
while G.check_slip(load_data, initial_force, 'both'):
    goal_aperture -= additional_closure  # Tightening the grip with a 3mm smaller aperture to counteract slippage.
    initial_force += additional_force_increase  # Increasing the gripping force cautiously to hold the bag securely without causing damage.
    G.set_force(initial_force, 'both')  # Reapplying the updated force to ensure a delicate yet secure hold on the plastic bag.
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)  # Reattempting the grasp with updated parameters.