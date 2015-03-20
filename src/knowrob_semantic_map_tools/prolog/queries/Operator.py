from knowrob_semantic_map_tools.prolog.Query import *

class Operator(Query):
  def __init__(self, operator, queries = []):
    Query.__init__(self, operator.join(str(query) for query in queries))
  