import rospy
from rospy.rostime import *

from knowrob_semantic_map_tools.prolog.IRI import *
from knowrob_semantic_map_tools.prolog.queries.Compound import *

class MapObjectStamp(Compound):
  def __init__(self, object, stamp = "Stamp"):
    Compound.__init__(self, "map_object_stamp", [str(object), stamp])
    
    self._stamp = stamp

  def getStamp(self):
    solution = self.solution
    
    if solution:
      return Time(secs = float(solution[self._stamp]))
    
    return None
    
  stamp = property(getStamp)
