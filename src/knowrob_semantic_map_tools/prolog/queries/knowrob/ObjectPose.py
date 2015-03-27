import numpy

from tf_conversions import *

from knowrob_semantic_map_tools.prolog.queries.Compound import *

class ObjectPose(Compound):
  def __init__(self, identifier, pose = "Pose"):
    Compound.__init__(self, "current_object_pose",
      [str(identifier), pose])
    
    self._pose = pose

  def getMatrix(self):
    solution = self.solution
    
    if solution:
      m = solution[self._pose]

      return [
        [m[0], m[1], m[2], m[3]],
        [m[4], m[5], m[6], m[7]],
        [m[8], m[9], m[10], m[11]],
        [m[12], m[13], m[14], m[15]]
      ]

    return None
    
  matrix = property(getMatrix)

  def getTf(self):
    matrix = self.matrix
    
    if matrix:
      frame = fromMatrix(numpy.array(matrix))
      return toTf(frame)
    
    return None
    
  tf = property(getTf)

  def getPose(self):
    tf = self.tf
    
    if tf:
      return {
        "position": tf[0],
        "orientation": tf[1]
      }
    
    return None
    
  pose = property(getPose)
