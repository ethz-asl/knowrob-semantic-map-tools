import rospy
import json

from json_prolog_msgs.srv import PrologQuery
from json_prolog_msgs.srv import PrologNextSolution, PrologNextSolutionResponse
from json_prolog_msgs.srv import PrologFinish

from Query import *
from Exception import *

class Client(object):
  def __init__(self):
    self.prologQueryService = rospy.get_param(
      "~clients/prolog_query/service",
      "/json_prolog/simple_query")
    self.prologNnextSolutionService = rospy.get_param(
      "~clients/prolog_next_solution/service",
      "/json_prolog/next_solution")
    self.prologFinishService = rospy.get_param(
      "~clients/prolog_finish/service",
      "/json_prolog/finish_query")
    
  def query(self, query):
    if isinstance(query, str):
      query = Query(query = query)
    
    rospy.wait_for_service(self.prologQueryService)
    request = rospy.ServiceProxy(self.prologQueryService, PrologQuery)    
    
    try:
      response = request(id = str(query.id), query = str(query))
    except rospy.ServiceException, exception:
      raise Exception("PrologQuery service request failed: %s" % exception)
    
    if not response.ok:
      raise Exception("Prolog query failed: %s" % response.message)

  def nextSolution(self, query):
    rospy.wait_for_service(self.prologNnextSolutionService)
    request = rospy.ServiceProxy(self.prologNnextSolutionService,
      PrologNextSolution)    

    try:
      response = request(id = str(query.id))
    except rospy.ServiceException, exception:
      raise Exception(
        "PrologNextSolution service request failed: %s" % exception)
    
    if response.status == PrologNextSolutionResponse.OK:
      yield json.loads(response.solution)
    elif response.status == PrologNextSolutionResponse.NO_SOLUTION:
      return
    elif response.status == PrologNextSolutionResponse.WRONG_ID:
      raise Exception("Prolog query id is invalid: "+
        "Another process may have terminated our query")
    elif response.status == PrologNextSolutionResponse.QUERY_FAILED:
      raise Exception("Prolog query failed: %s" % response.solution)
    else:
      raise Exception("Prolog query status unknown: %d", response.status)

  def finish(self, query):
    rospy.wait_for_service(self.prologFinishService)
    request = rospy.ServiceProxy(self.prologFinishService, PrologFinish)    
    
    try:
      response = request(id = str(query.id))
    except rospy.ServiceException, exception:
      raise Exception("PrologFinish service request failed: %s" % exception)
    