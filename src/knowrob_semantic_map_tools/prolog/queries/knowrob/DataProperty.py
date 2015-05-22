from knowrob_semantic_map_tools.prolog.IRI import *
from knowrob_semantic_map_tools.prolog.queries.Compound import *

class DataProperty(Compound):
  def __init__(self, property = "Property", subject = "Subject",
      value = "Value"):
    Compound.__init__(self, "data_property",
      [str(subject), str(property), value])
    
    self._subject = subject
    self._property = property
    self._value = value
    
  def getSubject(self):
    solution = self.solution
    
    if solution:
      return IRI(solution[self._subject])
  
  subject = property(getSubject)

  def getSubjects(self):
    for solution in self.solutions:
      yield IRI(solution[self._subject])
  
  subjects = property(getSubjects)

  def getValue(self):
    solution = self.solution
    
    if solution:
      return solution[self._value]
  
  value = property(getValue)

  def getValues(self):
    for solution in self.solutions:
      yield solution[self._value]
  
  values = property(getValues)
  