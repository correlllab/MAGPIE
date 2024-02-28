from magpie.gripper import Gripper  # must import the gripper class
G = Gripper()  # create a gripper object
import numpy as np  # import numpy because we are using it below

# Reset parameters to ensure the gripper is in a known state, especially important for a new task
G.reset_parameters()

# Reasoning: To grasp a credit card by its wide side, we need a precise aperture. The card's width is less than 55mm (standard credit card width), 
# ensuring that setting the gripper for its maximum precision helps in achieving a firm and precise grasp without causing damage.
goal_aperture = 55  # Approximated assuming standard credit card width

# Set compliance to allow for a firm but gentle grasp
# Adjusting to a lower flexibility to ensure firmness while still accommodating for slight size variances or positioning inaccuracies.
G.set_compliance(margin=0.5, flexibility=2, finger='both')

# Initially setting a lower force ensuring not to damage the card. This force will be increased later to secure the grip.
G.set_force(0.6, 'both')

# Move fingers to the estimated aperture for grasping a credit card by its wide side
G.set_goal_aperture(goal_aperture, finger='both', record_load=False)

# After reaching the initial positioning, increase the force slightly to confirm secure grasp without slipping or causing damage.
# The force is increased by 0.3N as indicated, ensuring it does not exceed the capability of handling low weight and compliance.
additional_force = 0.3  
final_force = min(0.6 + additional_force, 16)  # Ensuring not to exceed the gripper's maximum force capability
G.set_force(final_force, 'both')

# Check the actual aperture after attempting to grasp and adjust if necessary
# This step confirms that the fingers have not moved past the credit card due to a size misjudgment or slippage.
current_aperture = G.get_aperture('both')
if current_aperture > goal_aperture:
    # If the current aperture is larger than the goal, it means the grasp did not fully close as intended.
    # Adjusting the goal aperture to ensure the grasp is secure, taking into consideration the actual size and position of the card.
    G.set_goal_aperture(current_aperture, finger='both')