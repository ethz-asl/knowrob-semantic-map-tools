from knowrob_semantic_map_tools.prolog.queries.Compound import *

class GetHandlePath(Compound):
  def __init__(self, identifier, path = "Path"):
    Compound.__init__(self, "get_handle_path", [str(identifier), path])
    
    self._identifier = identifier
    self._path = path

  def getPath(self):
    solution = self.solution

    if solution:
      return solution[self._path]
    
    return None
    
  path = property(getPath)
