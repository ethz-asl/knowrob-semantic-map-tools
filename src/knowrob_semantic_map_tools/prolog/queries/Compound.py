from knowrob_semantic_map_tools.prolog.Query import *

class Compound(Query):
  def __init__(self, functor, args = []):
    if args:
      query = "%s(%s)" % (functor, ",".join(str(arg) for arg in args))
    else:
      query = functor
    
    Query.__init__(self, query)
  