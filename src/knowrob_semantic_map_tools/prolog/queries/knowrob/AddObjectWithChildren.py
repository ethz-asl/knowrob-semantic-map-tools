import re

from knowrob_semantic_map_tools.prolog.queries.Compound import *

class AddObjectWithChildren(Compound):
  def __init__(self, identifier, time = None):
    if time:
      Compound.__init__(self, "add_object_with_children", [identifier, time])
    else:
      Compound.__init__(self, "add_object_with_children", [identifier])
  