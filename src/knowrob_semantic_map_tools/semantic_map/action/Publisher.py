import rospy

from knowrob_semantic_map_msgs.msg import *

from knowrob_semantic_map_tools.prolog.queries import *

import Exception

class Publisher(object):
  def __init__(self, prologClient):
    self.semanticMapActionTopic = rospy.get_param(
      "~publishers/semantic_map_action/topic", "~actions")
    self.semanticMapActionQueueSize = rospy.get_param(
      "~publishers/semantic_map_action/queue_size", 1)
    self.semanticMapActionLatch = rospy.get_param(
      "~publishers/semantic_map_action/latch", False)

    self.prologClient = prologClient

    self.publisher = rospy.Publisher(self.semanticMapActionTopic,
      SemMapAction, queue_size = self.semanticMapActionQueueSize,
      latch = self.semanticMapActionLatch)
  
  def publishSemanticMapAction(self, feedback, action, object):
    message = SemMapAction()
    
    message.header = feedback.header
    
    message.id = action.iri
    
    mapObjectInfoQuery = knowrob.MapObjectInfo([object])
    mapObjectInfoQuery.execute(self.prologClient)
    objectInfo = mapObjectInfoQuery.info
    
    message.object_acted_on.id = objectInfo["identifier"].iri
    message.object_acted_on.type = objectInfo["type"].iri
    message.object_acted_on.size.x = objectInfo["dimensions"]["x"]
    message.object_acted_on.size.y = objectInfo["dimensions"]["y"]
    message.object_acted_on.size.z = objectInfo["dimensions"]["z"]
    message.object_acted_on.pose = objectInfo["pose"]
    message.object_acted_on.part_of = objectInfo["parent"].iri

    try:
      self.publisher.publish(message)
    except rospy.ROSSerializationException, exception:
      raise Exception(
        "SemanticMapAction publication failed: %s" % exception)
  