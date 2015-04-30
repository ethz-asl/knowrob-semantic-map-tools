from knowrob_semantic_map_tools.prolog.Query import *

class OwlParsed(Query):
  def __init__(self):
    Query.__init__(self, "owl_parsed")

  def getResult(self):
    if not self.finished:
      return self.solution is not None
    else:
      return False
    
  result = property(getResult)
  