import rospy

from knowrob_semantic_map_msgs.msg import *
from knowrob_semantic_map_msgs.srv import *

import Exception

class Client(object):
  def __init__(self):
    self.getSemanticMapService = rospy.get_param(
      "~clients/get_semantic_map/service",
      "semantic_map_server/get_map")
    
  def getMap(self):
    rospy.wait_for_service(self.getSemanticMapService)
    
    try:
      request = rospy.ServiceProxy(self.getSemanticMapService,
        GetSemanticMap)
      response = request()
    except rospy.ServiceException, exception:
      raise Exception(
        "GetSemanticMap service request failed: %s" % exception)
      
    return response.map
  
  map = property(getMap)
  