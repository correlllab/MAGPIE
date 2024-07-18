# Requirements & Development Plan
Brainstorming / Steering / Planning Document  
* Claim ownership of a task with **your name in bold text**.
* Coordinate with individuals writing client code of your API.
* <u>Progress Key</u>: _Please use this to indicate work status_
    - `[ ]` Task not started
    - `[>]` Task in progress
    - `[Y]` Task completed, YYYY-MM-DD: Completion comment
    - `[N]` Task cancelled, YYYY-MM-DD: Reason for abandonment
    - `[P]` Task paused / suspended
    - `{ }` Optional / Unsure
    - `[ ]` Q: Question to answer
    - `[ ]` E: Evaluate feature & Describe results

## General Requirements
* Python 3.9.x
* Experiments should be able to run from a Jupyter Lab notebook.
* Poses should be expressed in homogeneous coordinates (Numpy array)

## Preliminaries
1. `[Y]` Install PDDLStream and test at least one example, 2023-11-07: Only last 2 PyBullet examples and below seem to work.
    * `[Y]` Document steps including additional dependencies, 2023-11-07: Some examples require PyBullet
1. `[Y]` Attempt to build simplest PDDLStream example, 2023-11-09: Simple blocksworld solution, no perception
    * Blocks World example, 2023-11-09: We have an IRL blocksworld problem
    * `[N]` Tutorial?, 2023-11-07: No direct tutprial, Move to example
    * `[Y]` Choose simplest example as basis, 2023-11-09: Simple blocksworld solution, no perception
        - `[Y]` Identify function calls, 2023-11-09: Simple blocksworld solution, no perception
1. `[Y]` Examine PDDLStream problem with actual streams (**James**), 2023-11-09: Use this as a roadmap for building example
    * Problem Chosen: `examples.pybullet.pr2_belief.run`
    * `[Y]` Determine stream format, How to create one?
        - Streams are declared in a PDDL file
            * Name
            * List of input objects
            * Predicates that must be true of the inputs
            * List of output objects
            * Predicates that are certified true of the outputs when stream is evaluated
        - `PDDLProblem` Class Constructor
            * PDDL Domain
            * Dictionary of problem constants
            * Stream PDDL specification
            * Mapping of strings to functions
            * Initial state
            * Goal state
    * `[Y]` Determine program flow, What is called and when?, 2023-11-09: Use this as a roadmap for building example
        1. Solve PDDL: `pddlstream_from_state`, `solve`
        1. Associate actions with robot commands: `plan_commands`
        1. Run robot commands: `apply_commands`

1. `[>]` Implement PDDLStream version of the RYB Block Problem, Minimum Viable Prototype (**James**)
    1. `[Y]` Q: WHAT IS EVEN HAPPENING? --> Get better understanding of the PDDLStream workflow.
        * 2023-11-20: See synopsis below: "**How does the solver work?**"
        * 2023-11-20: This is a wider issue, See items below
    1. `[Y]` Locate blocks
        - `[Y]` Inspect localization code from Andrea/Dylan, 2023-11-20: Needs testing
        - `[Y]` Confirm that the segmentation pipline runs, 2023-11-21: Segmentation hack depends on simple color mask, needs testing
    1. `[Y]` Resolve block predicates, 2023-11-21: WARNING, Visualization indicates blocks are below deck!
        * REMEMBER: Streams should `yield` instead of `return`!
        - `[Y]` "Dummy" IK Solver, 2023-11-21: Just use the pose
    1. `[Y]` PDDLStream Solution <-from-- "Incremental", 2023-11-21: Solution obtained!
    1. `[ ]` Visualize
        * `[ ]` Visualize `move_free` action
        * `[ ]` Visualize `pick` action
        * `[ ]` ISSUE: Blocks are BELOW the table level!
            - `[ ]` Check the camera transform
        * `[ ]` ISSUE: Red and Yellow blocks occupy the same space!
            - `[ ]` Is there a different way to define the color filter?
    1. `[ ]` Execute
        * `[ ]` Behavior Tree for one action
        * `[ ]` Behavior Tree for all actions
    1. `[ ]` Postmortem Question: How will MAGPIE differ from PDDLStream?
        * `[ ]` Discuss required engineering with team
        * `[ ]` Discuss GPL3 licensing with advisor/team
    1. `[ ]` PDDLStream --to--> MAGPIE Investigation
        * `[ ]` De-tangle camera from detector
        * `[ ]` Create a local copy of "incremental.py" to allow inspection and debugging of PDDLStream pipeline
            - `[ ]` Display which solver was chosen by `solve_finite`


### How does the solver work?

#### INCREMENTAL
(**James**)  
Location: incremental.py  
`solve_incremental( ... )`
1. `parse_problem` -> 
    * `evaluations`
    * `goal_expression`
    * `domain`
    * `externals`
1. Create a `SolutionStore`: Appears to mediate whether the solver is running or not???
    * Can this be repurposed as the plan queue in "working memory"?
1. `compile_fluents_as_attachments`: Appears to initiate external sources of predicates???
1. Create an `Instantiator`: Handles instantiating streams
1. `process_stream_queue`: Appears to evaluate streams up to a certain complexity?
    * The idea that we would monitor the number of times that the stream is evaluated seems at odds with the idea of a continuously-updatings "stream" of sensory information.  Different paradigm ...
1. While under the complexity limit && Not terminated
    1. Increment iterations
    1. Print status
    1. `solve_finite`
        * **FIXME**: 
            - I DON'T REALLY UNDERSTAND THE NEED FOR ALL THESE **BRANCHES**! 
            - WHY CAN'T WE USE FASTDOWNWARD FOR **ALL** PDDL PROBLEMS?
        * Q: I don't understand the difference between sequential and temporal solvers
        * If the domain is `SimplifiedDomain`, then `solve_temporal`
            * Q: What is a simplified domain? Why do we assume that a simplfied domain is temporal?
            1. `get_problem_pddl`
            1. `solve_tfd`: Solve with the selected PDDL planner
        * Else `solve_sequential`
            * Q: What is a sequential solver and why do we need it here?
            1. `get_problem`
            1. `task_from_domain_problem`
                * Q: Is this a subgoal?
                1. Break the problem into components
                1. Extract "Requirements"
                1. Check for duplicate objects
                1. Create a `pddl.Task`, "normalize" it, and return it
                    * What does "normalize" mean?
            1. If there are external attachments, then `solve_pyplanners`
                1. Check paths and import libraries
                1. Process actions
                1. Process axioms
                1. Process the goal state
                1. `solve_strips`
                1. Return actions and cost
            1. Else `abstrips_solve_from_task`
                * If `hierarchy == SERIALIZE` then `serialized_solve_from_task`
                    - `plan_subgoals`
                * Else `solve_from_task`
                    - `run_search`
    1. If got a plan, then store it
        * Note that we are continuing to solve at deeper levels up until the specified level in the hopes that there is a lower cost solution at a deeper level
    1. If terminated, then `break`
    1. Increment complexity
    1. `process_stream_queue`: Appears to evaluate streams up to a certain complexity?
1. Get and print summary
1. Store statistics
1. Return solution

#### FOCUSED
Location: focused.py  
`solve_focused_original( ... )`  
1. `solve_abstract(problem, max_skeletons=None, search_sample_ratio=None,bind=False, max_failures=max_failures, **kwargs)`
    * See below

#### BINDING
Location: focused.py  
`solve_binding( ... )`  
1. `solve_abstract(problem, max_skeletons=None, search_sample_ratio=None,bind=True, max_failures=max_failures, **kwargs)`
    * See below

#### ADAPTIVE
Location: focused.py  
`solve_adaptive( ... )`  
1. `solve_abstract(problem, max_skeletons=max_skeletons, search_sample_ratio=search_sample_ratio,bind=None, max_failures=None, **kwargs)`
    * See below

#### ABSTRACT
Location: focused.py  
`solve_abstract( ... )`  
1. `parse_problem`
1. `automatically_negate_externals`
1. `enforce_simultaneous`
1. `compile_fluent_streams`
1. `load_stream_statistics`
1. `partition_externals`
1. Create `SolutionStore`
1. Create `Instantiator`
1. `process_stream_queue`: Appears to evaluate streams up to a certain complexity?
    * The idea that we would monitor the number of times that the stream is evaluated seems at odds with the idea of a continuously-updatings "stream" of sensory information.  Different paradigm ...
1. While under the complexity limit && Not terminated
    1. Increment iterations
    1. Print status
    1. `solve_finite`: See above
    1. If got a plan, then store it
        * Note that we are continuing to solve at deeper levels up until the specified level in the hopes that there is a lower cost solution at a deeper level
    1. If terminated, then `break`
    1. Increment complexity
    1. `process_stream_queue`: Appears to evaluate streams up to a certain complexity?
1. Get and print summary
1. Store statistics
1. Return solution

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

1. `[ ]` Define object belief symbol
    * `[ ]` Class
    * `[ ]` Confidence
    * `[ ]` Mean Pose
    * `[ ]` Pose Distribution
    * `{ }` Current / Stale?
1. `[ ]` Define observation
    * `[ ]` Timestamp
1. `[ ]` Define predicate structure
    * `[ ]` Q: Probabilistic?
1. `[ ]` Q: When a predicates evaluated?
1. `[Y]` Do symbols/observations require a spatial distribution?, 2023-11-09: Yes!
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

1. `[N]` Re-implement solver API from [Previous Project](https://github.com/correlllab/Factor-Graphs-for-Failure-Analysis/blob/main/zc_WeBots_PDDL/50_PDDL_Asm-Gear.ipynb), 2023-11-09: No, that is very silly???

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