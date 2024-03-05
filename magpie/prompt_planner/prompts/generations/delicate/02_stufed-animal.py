from magpie.gripper import Gripper  # import the gripper class
import numpy as np  # import numpy for later data manipulation

G = Gripper()  # create a gripper object

# This task involves creating a grip that is careful enough to handle a soft and potentially deformable object, like a fuzzy stuffed animal,
# without causing damage. It must account for medium compliance and the low weight of the object.
goal_aperture = 25  # Set the aperture to 25mm as this size is expected to secure the stuffed animal without compressing it too much.

# Set compliance slightly higher since the object is soft and medium compliant. This allows for slight adjustments during grip without causing damage.
G.set_compliance(2, 4, finger='both')  # Setting a slightly higher margin and flexibility for medium compliance handling.

initial_force = 0.5  # Starting with a force of 0.5 Newtons to ensure a secure grip without crushing the object.
G.set_force(initial_force, 'both')  # Apply this gentle initial force to both fingers.

# Trying to grasp the stuffed animal with the initial settings, and enabling load recording to respond if adjustments are needed.
# This initial grasp aims to encase the stuffed animal gently but securely, avoiding any potential damage.
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# Slippage adjustments: To handle cases where the initial grip wasn't sufficient, indicated by a detected slippage.
additional_closure = 2  # The aperture will be decreased by 2mm if slippage is detected, for a tighter grip.
additional_force_increase = 0.1  # Increase the force by 0.1 Newtons if slippage occurs, for a slightly firmer grip.

# After the initial grasp attempt, the state of the grip is assessed.
# If the grip was insufficient (the gripper slipped), aperture and force adjustments are iteratively applied until a stable grip is achieved.
while G.check_slip(load_data, initial_force, 'both'):
    goal_aperture -= additional_closure  # Decreasing the aperture slightly for a tighter grip on detection of slippage.
    initial_force += additional_force_increase  # Increasing the force gently to avoid damaging the stuffed animal while improving grip stability.
    G.set_force(initial_force, 'both')  # Apply the updated force setting to both fingers for an enhanced, careful grip.
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)  # Re-attempting the grasp with the updated settings.