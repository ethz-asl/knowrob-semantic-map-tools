from knowrob_semantic_map_tools.prolog.IRI import *
from knowrob_semantic_map_tools.prolog.queries.Compound import *

class MapObjectType(Compound):
  def __init__(self, object, type = "Type"):
    Compound.__init__(self, "map_object_type", [str(object), type])
    
    self._object = object
    self._type = type

  def getType(self):
    solution = self.solution
    
    if solution:
      return IRI(solution[self._type])
    
    return None
    
  type = property(getType)
