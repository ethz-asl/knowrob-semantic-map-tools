import re

from knowrob_semantic_map_tools.prolog.queries.Compound import *

class OwlParseString(Compound):
  def __init__(self, owl):
    owl = re.sub("^\s*", "", owl, flags = re.MULTILINE)
    owl = re.sub("^\s*$", "", owl)
    owl = re.sub("\n", "", owl)
    
    Compound.__init__(self, "owl_parse_string", ["'%s'" % owl])
  