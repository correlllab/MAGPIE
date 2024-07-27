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
    
* `[>]` Add periodic updates to all `MoveFree` behaviors
    - `[Y]` Lower robot movement speed so that updates *actually* occur during execution, 2024-07-26: Halved the default speed to 0.125
    - `[>]` Segment moves into alternating move and sense actions, 2024-07-26: Wow that took waaaaay too long
        * `[>]` Test moves 
    - `[ ]` Add the distribution change criterion back into the planner 
* `[ ]` Test planner

* `[!]` Determine whether objects outside of the camera frustrum are getting unfairly NULL updates!

* `[ ]` Determine experimental confusion matrix
    - `[ ]` Q: How many trials?
    - `{ }` Q: Is it possible to automate part of this?

* `[ ]` Test planner
* `[ ]` Experiments x10
