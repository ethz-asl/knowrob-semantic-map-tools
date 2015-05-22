from knowrob_semantic_map_msgs.msg import SemMapObjectSize

from knowrob_semantic_map_tools.prolog.queries.Compound import *

class ObjectDimensions(Compound):
  def __init__(self, identifier = "Identifier", width = "Width",
      height = "Height", depth = "Depth"):
    Compound.__init__(self, "object_dimensions",
      [str(identifier), depth, width, height])
    
    self._identifier = identifier
    self._width = width
    self._height = height
    self._depth = depth

  def getDimensions(self):
    solution = self.solution

    if solution:
      return self.solutionToDimensions(solution)

    return None
    
  dimensions = property(getDimensions)

  def solutionToDimensions(self, solution):
    return {
      "x": float(solution[self._depth]),
      "y": float(solution[self._width]),
      "z": float(solution[self._height])
    }

  def getMessage(self):
    solution = self.solution

    if solution:
      return self.solutionToMessage(solution)

    return None
    
  message = property(getMessage)

  def solutionToMessage(self, solution):
    msg = SemMapObjectSize()
    
    msg.x = float(solution[self._depth]) 
    msg.y = float(solution[self._width]) 
    msg.z = float(solution[self._height]) 

    return msg
