from knowrob_semantic_map_tools.prolog.IRI import *
from knowrob_semantic_map_tools.prolog.queries.Compound import *

class MapChildObjects(Compound):
  def __init__(self, parent, children = "Children"):
    Compound.__init__(self, "map_child_objects",
      [str(parent), children])
    
    self._parent = parent
    self._children = children

  def getChildren(self):
    solution = self.solution
    
    if solution:
      return [IRI(child) for child in solution[self._children]]
    
    return []
    
  children = property(getChildren)
