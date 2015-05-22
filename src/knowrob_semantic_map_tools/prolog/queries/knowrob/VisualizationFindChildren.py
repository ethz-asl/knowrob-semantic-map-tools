from knowrob_semantic_map_tools.prolog.IRI import *
from knowrob_semantic_map_tools.prolog.queries.Compound import *

class VisualizationFindChildren(Compound):
  def __init__(self, parents, children = "Children"):
    Compound.__init__(self, "visualization_find_children",
      ["[%s]" % ",".join(str(parent) for parent in parents), children])
    
    self._parents = parents
    self._children = children

  def getChildren(self):
    solution = self.solution
    
    if solution:
      return [IRI(child) for child in solution[self._children]]
    
    return []
    
  children = property(getChildren)
