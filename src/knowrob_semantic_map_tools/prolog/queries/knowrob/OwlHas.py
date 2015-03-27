from knowrob_semantic_map_tools.prolog.IRI import *
from knowrob_semantic_map_tools.prolog.queries.Compound import *

class OwlHas(Compound):
  def __init__(self, predicate, subject = "Subject", object = "Object"):
    Compound.__init__(self, "owl_has",
      [str(subject), predicate, str(object)])
    
    self._subject = subject
    self._predicate = predicate
    self._object = object
    
  def getSubject(self):
    solution = self.solution
    
    if solution:
      return IRI(solution[self._subject])
  
  subject = property(getSubject)

  def getSubjects(self):
    for solution in self.solutions:
      yield IRI(solution[self._subject])
  
  subjects = property(getSubjects)

  def getObject(self):
    solution = self.solution
    
    if solution:
      return IRI(solution[self._object])
  
  object = property(getObject)

  def getObjects(self):
    for solution in self.solutions:
      yield IRI(solution[self._object])
  
  objects = property(getObjects)
  