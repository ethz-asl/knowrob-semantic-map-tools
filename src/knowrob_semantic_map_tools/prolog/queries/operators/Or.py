from knowrob_semantic_map_tools.prolog.queries.Operator import *

class Or(Operator):
  def __init__(self, queries = []):
    Operator.__init__(self, ";", queries)
  