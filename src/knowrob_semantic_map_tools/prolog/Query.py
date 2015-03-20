import json
import uuid

class Query(object):
  def __init__(self, query = None):
    self.id = uuid.uuid1()    
    self.query = query

  def execute(self, client):  
    client.query(self)
      
  def getSolution(self, client):
    try:
      return self.getSolutions(client).next()
    except StopIteration:
      return None
    finally:
      client.finish(self)
  
  def getSolutions(self, client):
    try:
      for solution in client.nextSolution(self):
        yield solution
    finally:
      client.finish(self)
  
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
  