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

## Perception Stack
* `[>]` Test perception class as PY file
* `[ ]` Wrap in XML-RPC
    - `[ ]` Test initiating server from Jupyter
* `[ ]` Run ID service in a periodic loop as a separate process