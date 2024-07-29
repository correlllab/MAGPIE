# `DEV PLAN`

## Baseline Planner
* `[N]` Suppress camera on robot start, The perception process will connect to it, 2024-07-24: Perception process is not longer separate
* `[Y]` Complete vision stack integration (See "vision" folder), 2024-07-24: OWL-ViT works, but only some of the time
* `[Y]` Rewrite UR5 task behaviors, 2024-07-24: Robot moves as planned, TCP is computed on the planner side rather than the robot
    - `[Y]` `Pick`, 2024-07-24: Confirmed
    - `[Y]` `Place`, 2024-07-24: Confirmed
    - `[Y]` `Stack`, 2024-07-24: Confirmed
    - `[Y]` `Unstack`, 2024-07-24: Confirmed
* 2024-07-24: ABANDON BASELINE PLANNER AT CORRELL'S INSTRUCTION
    - `[N]` Complete Baseline planner integration (See "za_Archive/130_rewrite" folder)
    - `[N]` Test Baseline planner
    - `[N]` Run $N=250$ experiments


## Responsive Planner
* `[Y]` Implement Last-Best-Reading, 2024-07-24: Works as designed
    - `[Y]` Develop trust measure, 2024-07-24: Works as designed
    - `[Y]` `MoveHolding` updates previous trusted measure pose, 2024-07-24: Works as designed
* `[Y]` Move `ObjectMemory.most_likely_objects` to the planner and adapt to merge the LKG and belief readings in a sane way, 2024-07-24: Needs testing
* `[Y]` Adapt Bayesian belief update to current planner, 2024-07-26: Changes make sense, but TESTING REQUIRED
    - `[Y]` Add time and quality info to the `ObjectReading` string `__rep__`, 2024-07-26: This will help with troubleshooting
    - `[Y]` Check that there is a distance limit on evidence application, 2024-07-26: This is now an environment variable
    - `[Y]` Check that belief timestamp is updated when the belief is updated, 2024-07-26: Timestamp now updates, but not for NULL evidence
        * 2024-07-26: NOT updating the timestamp for NULL evidence, as it should tend to remove a reading from consideration
    

* `[>]` ISSUE: The planner is getting confused and trying to grab ghost blocks
    - `[Y]` Visualize LKG & run until confusion happens, 2024-07-28: Need to look at the interaction with beliefs
        * 2024-07-28: The bottom block got lost and that should not have happened, the first move action should have fixed that entry at its destination 
        * 2024-07-28: Sometimes the bottom block changes distribution suddenly and changes identity, Does it become something different when the planner tries to resolve duplicates?
    - `[ ]` Visualize Beliefs
    - `[ ]` Visualize jumbled collection of {LKG,Beliefs}
    - `[ ]` Visualize reconciled collection of {LKG,Beliefs}
        * `[ ]` Highlight the readings that became symbols


* `[P]` Add periodic updates to all `MoveFree` behaviors
    - `[Y]` Lower robot movement speed so that updates *actually* occur during execution, 2024-07-26: Halved the default speed to 0.125
    - `[>]` Segment moves into alternating move and sense actions, 2024-07-26: Wow that took waaaaay too long
        * `[Y]` Test moves, 2024-07-26: Moves are WEIRD, Planning the experiment reset as a GRASP?
        * `[Y]` Fix moves, 2024-07-27: Fixed
        * `[Y]` After placing a block, need to suppress the starting sense action when the gripper is directly over the block, 2024-07-28: Run a 
        check with `check_and_correct_extreme_closeup` and suppress if check fails

        * `[>]` Integrate DeliGrasp gripper width code for the block presence check

    - `[Y]` Block unfair NULL updates to objects outside of the camera frustrum
        * `[Y]` Calc frustrum as the interior of 5 planes defined by X & Y FOVs and max depth, 2024-07-28: Tested
        * `[Y]` Check that a sphere w/ block radius is inside of all 4 planes, 2024-07-28: Tested
        * `[Y]` Suppress NULL update for blocks that fail check, 2024-07-28: Needs Testing
    - `[>]` Add the distribution change criterion back into the planner 
        * `[ ]` Q: How to quantify change to an LKG entry?
* `[ ]` Test planner



* `[ ]` Determine experimental confusion matrix
    - `[ ]` Attempt Correll's automation procedure

* `[ ]` Test planner
* `[ ]` Experiments x10
