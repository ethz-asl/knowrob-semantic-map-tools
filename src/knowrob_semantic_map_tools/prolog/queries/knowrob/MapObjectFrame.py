from knowrob_semantic_map_tools.prolog.IRI import *
from knowrob_semantic_map_tools.prolog.queries.Compound import *

class MapObjectFrame(Compound):
  def __init__(self, object, frame = "Frame"):
    Compound.__init__(self, "map_object_frame", [str(object), frame])
    
    self._object = object
    self._frame = frame

  def getFrame(self):
    solution = self.solution
    
    if solution:
      return solution[self._frame]
    
    return None
    
  frame = property(getFrame)
