"""Abstract class for detected objects"""

import dataclasses
from typing import Dict, Any, Optional, Sequence, Tuple
import filterpy.discrete_bayes as db

@dataclasses.dataclass
class Object:
  name: str
  # traits dictionary: key = trait name, value = trait value
  traits: Dict[str, Any] = dataclasses.field(default_factory=dict)
  # history of traits: key = trait name, value = list of (trait value, confidence)
  traits_history: Dict[str, Sequence[Tuple[Any, float]]] = dataclasses.field(default_factory=dict)
  def __str__(self):
    return f"Object(name={self.name}, traits={self.traits})"

  def __repr__(self):
    return str(self)

