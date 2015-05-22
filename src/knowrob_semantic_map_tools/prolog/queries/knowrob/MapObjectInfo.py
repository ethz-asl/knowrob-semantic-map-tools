import numpy

import rospy
from rospy.rostime import *

from tf_conversions import *

from knowrob_semantic_map_tools.prolog.IRI import *
from knowrob_semantic_map_tools.prolog.queries.Compound import *

from knowrob_semantic_map_tools.prolog.queries.knowrob.ObjectPose import *
from knowrob_semantic_map_tools.prolog.queries.knowrob.ObjectDimensions \
  import *
from knowrob_semantic_map_tools.prolog.queries.knowrob.MapObjectStamp import *

class MapObjectInfo(Compound):
  def __init__(self, identifiers, identifier = "Identifier", type = "Type",
      label = "Label", frame = "Frame", stamp = "Stamp", pose = "Pose",
      width = "Width", height = "Height", depth = "Depth", parent = "Parent"):
    Compound.__init__(self, "map_object_info",
      ["[%s]" % ", ".join(str(identifier) for identifier in identifiers),
      "[%s, %s, %s, %s, %s, %s, [%s, %s, %s], %s]" % (identifier, type, label,
      frame, stamp, pose, width, height, depth, parent)])
    
    self._identifiers = identifiers
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
    return {
      "identifier": IRI(solution[self._identifier]),
      "type": IRI(solution[self._type]),
      "label": solution[self._label],
      "frame": solution[self._frame],
      "stamp": MapObjectStamp(stamp = self._stamp).solutionToStamp(solution),
      "pose": ObjectPose(pose = self._pose).solutionToMessage(solution),
      "dimensions": ObjectDimensions(width = self._width,
        height = self._height, depth = self._depth).
        solutionToDimensions(solution),
      "parent": IRI(solution[self._parent])
    }
