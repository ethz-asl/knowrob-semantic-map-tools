import numpy

from tf_conversions import *

from knowrob_semantic_map_tools.prolog.queries.Compound import *

class ObjectPose(Compound):
  def __init__(self, identifier = "Identifier", pose = "Pose"):
    Compound.__init__(self, "current_object_pose",
      [str(identifier), pose])

    self._identifier = identifier
    self._pose = pose

  def getMatrix(self):
    solution = self.solution
    
    if solution:
      return self.solutionToMatrix(solution)

    return None
    
  matrix = property(getMatrix)

  def solutionToMatrix(self, solution):
    m = solution[self._pose]

    return [
      [m[0], m[1], m[2], m[3]],
      [m[4], m[5], m[6], m[7]],
      [m[8], m[9], m[10], m[11]],
      [m[12], m[13], m[14], m[15]]
    ]
  
  def getTf(self):
    solution = self.solution
    
    if solution:
      return self.solutionToTf(solution)
    
    return None
    
  tf = property(getTf)

  def solutionToTf(self, solution):
    matrix = self.solutionToMatrix(solution)    
    frame = fromMatrix(numpy.array(matrix))
    
    return toTf(frame)
    
  def getPose(self):
    solution = self.solution
    
    if solution:
      return self.solutionToPose(solution)
    
    return None
    
  pose = property(getPose)

  def solutionToPose(self, solution):
    tf = self.solutionToTf(solution)
    
    return {
      "position": tf[0],
      "orientation": tf[1]
    }

  def getMessage(self):
    solution = self.solution
    
    if solution:
      return self.solutionToMessage(solution)
    
    return None
    
  message = property(getMessage)

  def solutionToMessage(self, solution):
    matrix = self.solutionToMatrix(solution)    
    frame = fromMatrix(numpy.array(matrix))
    
    return toMsg(frame)
