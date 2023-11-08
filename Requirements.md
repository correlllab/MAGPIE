# Requirements & Development Plan
Brainstorming / Steering / Planning Document

## General Requirements
* Python 3.9.x
* Experiments should be able to run from a Jupyter Lab notebook.
* Poses should be expressed in homogeneous coordinates (Numpy array)

## Preliminaries
1. `[Y]` Install PDDLStream and test at least one example, 2023-11-07: Only last 2 PyBullet examples and below seem to work.
    * `[Y]` Document steps including additional dependencies, 2023-11-07: Some examples require PyBullet
1. `[>]` Attempt to build a PDDLStream example
    * `[N]` Tutorial?, 2023-11-07: No direct tutprial, Move to example
    * `[>]` Choose simplest example as basis
1. `[ ]` Q: What data structures does PDDLStream rely on?
1. `[ ]` Use the Block Stacking Problem for Development and Testing?
    * `(+)` Simpler segmentation
    * `(-)` Very different from intended use cases

## Camera Process
* The Camera Process maintains a live feed from the RealSense D405 in the MAGPIE palm.
* Frame rate = ?
1. `[ ]` Color image stream
1. `[ ]` Color point cloud stream
1. `[ ]` Serve frames to client processes (XML-RPC?)

## Segmentation Process
* The Segmentation Process maintains a live feed of poses and distribution information for objects found in the camera feed. 
* Poses should be expressed in the UR5 frame of reference.
1. `[ ]` Choose a classification / segmentation method
    * `[ ]` Does it provide poses?
    * `[ ]` If not, how will object poses be determined?
1. `[ ]` Identify training requirements
1. `[ ]` Train
1. `[ ]` Benchmark segmentation: Speed & Accuracy
1. `[ ]` Serve identified objects to client processes (XML-RPC?)

## Belief-Desire-Intent (BDI) Process
* Responsible for reconciling live, continuous beliefs with the discrete, symbolic PDDL planner
* The BDI Process maintains a live blackboard of object beliefs
    - Class
    - Pose 
    - Confidence / Distribution
* The BDI Process maintains a live blackboard of the top $k$ plans and their rank (our confidence in their success)
* The top $k$ plans should be continually updated based on object beliefs (Priority Queue?)
* The top $k$ plans should be ready to be executed as soon as we commit to them
* Our confidence in a plan must not collapse if a past symbol has been successfully transformed. (Ignore "stale" states when scoring plans.)

### Belief Blackboard
* A live blackboard of object beliefs
* Belief Update
    - Strengthen beliefs with supporting observations
    - Decay beliefs with contradicting observations
    - Object Permanence: Objects that leave the frame do not receive belief updates unless an operator acts on it, or a contradiction is observed
    - Special cases?: `in_hand( OBJ )`, ...

1. `[ ]` Define object symbol
    * `[ ]` Timestamp
    * `[ ]` Current / Stale?
1. `[ ]` Define observation
    * `[ ]` Timestamp
1. `[ ]` Define predicate structure
    * `[ ]` Q: Probabilistic?
1. `[ ]` Q: When a predicates evaluated?
1. `[ ]` Do symbols/observations require a spatial distribution?
1. `[ ]` Need a reasoned algo to strengthen/decay beliefs
1. `[ ]` Detect observations that contradict existing symbols
1. `[ ]` Define a minimum threshold of confidence such that symbols are only retained above it, otherwise erase
1. `[ ]` Object Permanence: Objects that leave the frame do not receive belief updates unless an operator acts on it, or a contradiction is observed
1. `[ ]` Handle held objects
1. `[ ]` Handle held object contradictions

### Desire Blackboard
* A live blackboard of $k$ top plans and their rank
* MAGPIE should switch to the BT associated with the top-ranked plan if it is not already running it

1. `[ ]` Operator Structure
    * `[ ]` Abstract -or- Concrete?
    * `[ ]` Q: What if an operator only requires the existence of an object?
    * `[ ]` Q: Does success determination even matter if we are constantly replanning?  
    The alternative is counting on executed actions to make at least a little progress each run.
1. `[ ]` Plans reference objects, not contain them

### Intent Planner
1. `[ ]` Reconcile contradictory symbols into most likely states (Sampled or Algo?)
1. `[ ]` Create PDDL problems based on most likely states
1. `[ ]` Request plans from FastDownward
1. `[ ]` Rank plans from FastDownward 
1. `[ ]` Request grounded BTs from MAGPIE
1. `[ ]` Notify MAGPIE of Best Option

## MAGPIE Core
* **M**anipulation **A**rchitecture for **G**oal **P**lanning, **I**ntent management, and **E**xecution
* **T**ask **A**nd **M**otion **P**lanning framework
* Support multiple applications of robotic manipulation within Correll Lab & beyond

### PDDL Solver

1. `[ ]` Re-implement solver API from [Previous Project](https://github.com/correlllab/Factor-Graphs-for-Failure-Analysis/blob/main/zc_WeBots_PDDL/50_PDDL_Asm-Gear.ipynb)

### Symbolic Plans --to-> Motion Plans

1. `[ ]` Sample continuous parameters for symbolic operators

### Motion Plans --to-> Behavior Trees
1. `[ ]` Compose operators + parameters into Behavior Trees

### Behavior Tree Runner
1. `[ ]` Break out old MAGPIE runner function into BT runner class
1. `[ ]` Ability to quit a BT mid-run

### Motion Control
* UR5
* Dynamixel

1. `[ ]` Implement block stacking with MAGPIE control functions and confirm success
1. `[ ]` Implement identified improvements

&nbsp;  

# Desired Capabilities
For the lab to express additional capabilities that the framework should have.

* Idea 1
* Suggestion 2
* Daydream 3
* ...