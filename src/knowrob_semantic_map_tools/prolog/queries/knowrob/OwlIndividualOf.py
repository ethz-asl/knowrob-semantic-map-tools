from knowrob_semantic_map_tools.prolog.IRI import *
from knowrob_semantic_map_tools.prolog.queries.Compound import *

class OwlIndividualOf(Compound):
  def __init__(self, type, individual = "Individual"):
    Compound.__init__(self, "owl_individual_of", [str(individual), type])

    self._individual = individual
  
  def getIndividual(self):
    solution = self.solution
    
    if solution:
      return IRI(solution[self._individual])
    
    return None
    
  individual = property(getIndividual)
  
  def getIndividuals(self):
    for solution in self.solutions:
      yield IRI(solution[self._individual])
    
  individuals = property(getIndividuals)
  