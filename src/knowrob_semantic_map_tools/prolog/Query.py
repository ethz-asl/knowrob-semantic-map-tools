import json
import uuid

class Query(object):
  def __init__(self, query = None):
    self.id = uuid.uuid1()
    self.query = query
    
    self.client = None

  def execute(self, client):  
    client.query(self)
    return self
      
  def getSolution(self):
    try:
      return self.getSolutions().next()
    except StopIteration:
      return None
    finally:
      if not self.finished:
        self.client.finish(self)
  
  solution = property(getSolution)
  
  def getSolutions(self):
    if not self.finished:
      try:
        for solution in self.client.nextSolution(self):
          yield solution
      finally:
        if not self.finished:
          self.client.finish(self)
  
  solutions = property(getSolutions)
  
  def finish(self):  
    if not self.finished:
      self.client.finish(self)
  
  def getFinished(self):
    return self.client == None
  
  finished = property(getFinished)
  
  def __str__(self):
    return self.query
  
  def __and__(self, query):
    from knowrob_semantic_map_tools.prolog.queries.operators import And
    return And([self, query])

  def __iand__(self, query):
    from knowrob_semantic_map_tools.prolog.queries.operators import And
    return And([self, query])

  def __or__(self, query):
    from knowrob_semantic_map_tools.prolog.queries.operators import Or
    return Or([self, query])

  def __ior__(self, query):
    from knowrob_semantic_map_tools.prolog.queries.operators import Or
    return Or([self, query])
  