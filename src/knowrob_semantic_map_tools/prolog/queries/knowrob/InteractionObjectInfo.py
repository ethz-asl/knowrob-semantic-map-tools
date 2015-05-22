import numpy

from tf_conversions import *

from knowrob_semantic_map_tools.prolog.IRI import *
from knowrob_semantic_map_tools.prolog.queries.Compound import *

from knowrob_semantic_map_tools.prolog.queries.knowrob.ObjectPose import *

class InteractionObjectInfo(Compound):
  def __init__(self, identifiers, identifier = "Identifier",
      type = "Type", label = "Label", pose = "Pose",
      handlePath = "HandlePath", actions = "Actions"):
    Compound.__init__(self, "interaction_object_info",
      ["[%s]" % ", ".join(str(identifier) for identifier in identifiers),
      "[%s, %s, %s, %s, %s, %s]" % (identifier, type, label, pose,
      handlePath, actions)])
    
    self._identifiers = identifiers
    self._identifier = identifier
    self._type = type
    self._pose = pose
    self._label = label
    self._handlePath = handlePath
    self._actions = actions

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
    actions = []
    for action in solution[self._actions]:
      actions.append({
        "identifier": IRI(action[0]),
        "title": action[1]
      })
    
    return {
      "identifier": IRI(solution[self._identifier]),
      "type": IRI(solution[self._type]),
      "label": solution[self._label],
      "tf": ObjectPose(pose = self._pose).solutionToTf(solution),
      "handle": solution[self._handlePath],
      "actions": actions
    }
