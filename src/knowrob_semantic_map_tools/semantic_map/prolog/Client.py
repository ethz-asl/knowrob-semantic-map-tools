import rospy
from rospy.rostime import *

from knowrob_semantic_map_tools.prolog.Client import Client as Base
from knowrob_semantic_map_tools.prolog import *

from knowrob_semantic_map_tools.prolog.queries import *

from Exception import Exception

class Client(Base):
  def __init__(self):
    Base.__init__(self)
    
  def waitForOwlParsed(self, timeout = None, rate = 0.1):
    owlParsedQuery = knowrob.OwlParsed()
    owlParsedQueryResult = False
    
    if timeout:
      timeout = Time.now()+Duraction(timeout)
    
    while not owlParsedQueryResult:
      owlParsedQuery.execute(self)      
      owlParsedQueryResult = owlParsedQuery.result
      
      if not owlParsedQueryResult:
        if timeout and Time.now() > timeout:
          return False
        else:
          rospy.sleep(rate)

    return True
  