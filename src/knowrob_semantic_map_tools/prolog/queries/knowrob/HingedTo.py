from knowrob_semantic_map_tools.prolog.queries.knowrob.ObjectProperty import *

class HingedTo(ObjectProperty):
  def __init__(self, subject = "Subject", object = "Object"):
    ObjectProperty.__init__(self, "hinged_to", subject, object)
  
    print str(self)