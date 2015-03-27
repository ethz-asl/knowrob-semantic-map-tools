from knowrob_semantic_map_tools.prolog.queries.Compound import *

class RegisterPrefix(Compound):
  def __init__(self, prefix, uri, keep = True):
    Compound.__init__(self, "rdf_register_prefix", [prefix, "'%s'" % uri,
      "[keep(%s)]" % str(keep).lower()])
  