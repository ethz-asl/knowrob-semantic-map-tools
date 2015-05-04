import numpy

import rospy
from rospy.rostime import *

from tf_conversions import *

from knowrob_semantic_map_tools.prolog.IRI import *
from knowrob_semantic_map_tools.prolog.queries.Compound import *

class MapObjectInfo(Compound):
  def __init__(self, identifiers, identifier = "Identifier", type = "Type",
      label = "Label", frame = "Frame", stamp = "Stamp", pose = "Pose",
      width = "Width", height = "Height", depth = "Depth", parent = "Parent"):
    Compound.__init__(self, "map_object_info",
      ["[%s]" % ", ".join(str(identifier) for identifier in identifiers),
      "[%s, %s, %s, %s, %s, %s, [%s, %s, %s], %s]" % (identifier, type, label,
      frame, stamp, pose, width, height, depth, parent)])
    
    self._identifier = identifier
    self._type = type
    self._label = label
    self._frame = frame
    self._stamp = stamp
    self._pose = pose
    self._width = width
    self._height = height
    self._depth = depth
    self._parent = parent

  def getInfo(self):
    solution = self.solution
    
    if solution:
      return self.solutionToInfo(solution)
        
    return None
    
  info = property(getInfo)
  
  def getInfos(self):
    for solution in self.solutions:
      yield self.solutionToInfo(solution)
      
  infos = property(getInfos)

  def solutionToInfo(self, solution):
    m = solution[self._pose]
    m = [
      [m[0], m[1], m[2], m[3]],
      [m[4], m[5], m[6], m[7]],
      [m[8], m[9], m[10], m[11]],
      [m[12], m[13], m[14], m[15]]
    ]
    
    frame = fromMatrix(numpy.array(m))
    pose = toMsg(frame)
    
    return {
      "identifier": IRI(solution[self._identifier]),
      "type": IRI(solution[self._type]),
      "label": solution[self._label],
      "frame": solution[self._frame],
      "stamp": Time(secs = float(solution[self._stamp])),
      "pose": pose,
      "dimensions": {
        "x": float(solution[self._depth]),
        "y": float(solution[self._width]),
        "z": float(solution[self._height]),
      },
      "parent": IRI(solution[self._parent])
    }
