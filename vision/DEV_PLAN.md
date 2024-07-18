# `DEV GUIDE`

## Approach & Guidelines:
* Keep perception and robot motion separate
    - This includes the camera

## Input: Python Dictionary
``` 
{
    'camera_pose': <list[16]: homog coord>,
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