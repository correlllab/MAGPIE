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
* `[>]` Test Bayesian belief update
    - `[ ]` Check that there is a distance limit on evidence application
    - `[ ]` Add time and quality info to the `ObjectReading` string `__rep__`
* `[ ]` Test planner
* `[ ]` Experiments x10
