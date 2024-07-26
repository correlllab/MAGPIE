# `DEV GUIDE`

## Approach & Guidelines:
* Keep perception and robot motion in separate processes
    - Perception Process
        * Camera
        * LLM
    - Parent Process
        * UR5
        * BT
            - Keep the robot effector pose updated on the Perception Process


## Input: Message Structure
``` 
{
    'cmnd': <One of {"SET_VIZ", "POSE_IN", "SHUTDOWN"}>,
    'data': <One of {<Homog Effector Pose as 1x16 List>, 0, 1, None}>,
}
```

## Outputs: List of {Object Poses & Class Distributions}
```
[
    ...
    {
        'pose': <list>
        'labels': { ... , <str:label>: <float:confidence> , ... }
    },
    ...
]
```


# `DEV PLAN`

## Perception Stack, Complete, 2024-07-19: Works as designed!
* `[Y]` Test perception class as PY file, 2024-07-1X: Works as designed!
* `[Y]` Wrap in Subprocess, 2024-07-19: Works as designed!
    - `[Y]` Test initiating server from another file, 2024-07-19: Works as designed!
* `[Y]` Run ID service in a periodic loop as a separate process, 2024-07-19: Works as designed!
* `[Y]` Add all relevant blocks to the vision stack queries, 2024-07-2X: Added
* `[P]` Test multi-shot segmentation
    - `[P]` Apply repairs to multi-shot segmentation