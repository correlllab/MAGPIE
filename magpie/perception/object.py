"""Abstract class for detected objects"""

import dataclasses
from typing import Dict, Any, Optional, Sequence, Tuple
import filterpy.discrete_bayes as db
import filterpy.kalman as kf
import numpy as np

@dataclasses.dataclass
class Object:
    name: str
    # traits dictionary: key = trait name, value = trait value
    state: Dict[str, Any] = dataclasses.field(default_factory=dict)
    state_estimators: Dict[str, Any] = dataclasses.field(default_factory=dict)
    traits: Dict[str, Any] = dataclasses.field(default_factory=dict)
    # history of traits: key = trait name, value = dict of of (measurement, confidence)
    traits_history: Dict[str, Dict[str, Sequence[Any]]] = dataclasses.field(default_factory=dict)
    
    def __post_init__(self):
        for trait in self.traits:
            self.traits_history[trait] = []
            self.state_estimators[trait] = kf.KalmanFilter(dim_x=1, dim_z=1)
            self.state_estimators[trait].x = np.array([self.traits[trait]])
            self.state_estimators[trait].F = np.array([[1.]])
            self.state_estimators[trait].H = np.array([[1.]])
            self.state_estimators[trait].P *= 1000
            self.state_estimators[trait].R *= 10
            self.state_estimators[trait].Q *= 0.0001

    def __str__(self):
        return f"Object(name={self.name}, traits={self.traits})"

    def __repr__(self):
        return str(self)
    
    def update_state(self, state_type: str, state_value: Any, confidence: float):
        self.state[state_type] = state_value

    def update_trait(self, trait: str, reward: float, confidence: float):
        if trait not in self.traits_history:
            self.traits_history[trait] = []
        self.traits_history[trait].append((reward, confidence))

    def get_trait(self, trait: str) -> Any:
        """Get the value of a trait for this object"""
        return self.traits[trait]
