# Requirements & Development Plan

## General Requirements
* Python 3.9.x
* Experiments should be able to run from a Jupyter Lab notebook.
* Poses should be expressed in homogeneous coordinates (Numpy array)


## Camera Process
* The Camera Process maintains a live feed from the RealSense D405 in the MAGPIE palm.
* Frame rate = ?
1. `[ ]` Color image stream
1. `{?}` Color point cloud stream?
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

### Desire Blackboard
* A live blackboard of $k$ top plans and their rank
* MAGPIE should switch to the BT associated with the top-ranked plan if it is not already running it

### Intent Planner
* Reconcile contradictory symbols into most likely states (Sampled or Algo?)
* Create PDDL problems based on most likely states
* Request plans from FastDownward
* Rank plans from FastDownward 
* Request grounded BTs from MAGPIE
* Notify MAGPIE of Best Option

## MAGPIE Core
* **M**anipulation **A**rchitecture for **G**oal **P**lanning, **I**ntent management, and **E**xecution
* **T**ask **A**nd **M**otion **P**lanning framework
* Support multiple applications of robotic manipulation within Correll Lab & beyond

### PDDL Solver

* `FIXME`

### Symbolic Plans --to-> Motion Plans

* `FIXME`

### Motion Plans --to-> Behavior Trees
* `FIXME`

### Behavior Tree Runner

* `FIXME`

### Motion Control
* UR5
* Dynamixel
* `FIXME`

1. `[ ]` Query TCP

&nbsp;  

# Desired Capabilities