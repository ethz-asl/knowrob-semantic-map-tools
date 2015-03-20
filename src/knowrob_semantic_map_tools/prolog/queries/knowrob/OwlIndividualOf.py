import re

from knowrob_semantic_map_tools.prolog.queries.Compound import *

class OwlIndividualOf(Compound):
  def __init__(self, resource, description):
    Compound.__init__(self, "owl_individual_of", [resource, description])
  