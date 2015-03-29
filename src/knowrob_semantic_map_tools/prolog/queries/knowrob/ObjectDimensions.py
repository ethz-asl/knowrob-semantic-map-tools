from knowrob_semantic_map_tools.prolog.queries.Compound import *

class ObjectDimensions(Compound):
  def __init__(self, identifier, width = "Width", height = "Height",
      depth = "Depth"):
    Compound.__init__(self, "object_dimensions",
      [str(identifier), depth, width, height])
    print str(self)
    
    self._width = width
    self._height = height
    self._depth = depth

  def getDimensions(self):
    solution = self.solution

    if solution:
      return {
        "x": float(solution[self._depth]),
        "y": float(solution[self._width]),
        "z": float(solution[self._height])
      }
    
    return None
    
  dimensions = property(getDimensions)
