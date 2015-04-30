import rospy

from knowrob_semantic_map_msgs.msg import *
from knowrob_semantic_map_msgs.srv import *

import Exception

class Client(object):
  def __init__(self, map):
    self.generateSemanticMapOwlService = rospy.get_param(
      "~clients/generate_semantic_map_owl/service",
      "semantic_map_to_owl/generate_owl_map")
    
    self.map = map
  
  def getOwl(self):
    rospy.wait_for_service(self.generateSemanticMapOwlService)
    
    try:
      request = rospy.ServiceProxy(self.generateSemanticMapOwlService,
        GenerateSemanticMapOWL)
      response = request(map = self.map)
    except rospy.ServiceException, exception:
      raise Exception(
        "GenerateSemanticMapOWL service request failed: %s" % exception)
      
    return response.owlmap
  
  owl = property(getOwl)
  