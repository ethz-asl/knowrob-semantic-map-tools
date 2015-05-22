import numpy

from tf_conversions import *

from knowrob_semantic_map_tools.prolog.IRI import *
from knowrob_semantic_map_tools.prolog.queries.Compound import *

from knowrob_semantic_map_tools.prolog.queries.knowrob.ObjectPose import *
from knowrob_semantic_map_tools.prolog.queries.knowrob.ObjectDimensions \
  import *

class VisualizationObjectInfo(Compound):
  def __init__(self, identifiers, identifier = "Identifier", type = "Type",
      label = "Label", pose = "Pose", width = "Width", height = "Height",
      depth = "Depth", modelPath = "ModelPath"):
    Compound.__init__(self, "visualization_object_info",
      ["[%s]" % ", ".join(str(identifier) for identifier in identifiers),
      "[%s, %s, %s, %s, [%s, %s, %s], %s]" % (identifier, type, label,
      pose, width, height, depth, modelPath)])
    
    self._identifiers = identifiers
    self._identifier = identifier
    self._type = type
    self._label = label
    self._pose = pose
    self._width = width
    self._height = height
    self._depth = depth
    self._modelPath = modelPath

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
      "tf": ObjectPose(pose = self._pose).solutionToTf(solution),
      "dimensions": ObjectDimensions(width = self._width,
        height = self._height, depth = self._depth).
        solutionToDimensions(solution),
      "model": solution[self._modelPath]
    }
