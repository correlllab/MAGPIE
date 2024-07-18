"""Abstract class for detected objects"""

import dataclasses
from typing import Dict, Any, Optional, Sequence, Tuple
import filterpy.discrete_bayes as db
import filterpy.kalman as kf

@dataclasses.dataclass
class Object:
    name: str
    # traits dictionary: key = trait name, value = trait value
    traits: Dict[str, Any] = dataclasses.field(default_factory=dict)
    # history of traits: key = trait name, value = dict of of (measurement, confidence)
    traits_history: Dict[str, Dict[str, Sequence[Any]]] = dataclasses.field(default_factory=dict)
    
    def __str__(self):
        return f"Object(name={self.name}, traits={self.traits})"

    def __repr__(self):
        return str(self)
    
    def update(self, trait: str, value: Any, confidence: float):
        """Update the value of a trait for this object"""
        # self.traits[trait] = value
        # use discrete bayes filter to update the value of a trait for this object
        self.traits[trait]
        if trait not in self.traits_history:
            self.traits_history[trait] = []
        self.traits_history[trait].append((value, confidence))

    def get(self, trait: str) -> Any:
        """Get the value of a trait for this object"""
        return self.traits[trait]
