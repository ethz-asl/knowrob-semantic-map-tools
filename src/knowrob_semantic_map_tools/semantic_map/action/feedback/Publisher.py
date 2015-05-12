import rospy

from knowrob_semantic_map_msgs.msg import *

import Exception

class Publisher(object):
  def __init__(self):
    self.semanticMapActionFeedbackTopic = rospy.get_param(
      "~publishers/semantic_map_action_feedback/topic", "~actions")
    self.semanticMapActionFeedbackQueueSize = rospy.get_param(
      "~publishers/semantic_map_action_feedback/queue_size", 1)
    self.semanticMapActionFeedbackLatch = rospy.get_param(
      "~publishers/semantic_map_action_feedback/latch", False)

    self.publisher = rospy.Publisher(
      self.semanticMapActionFeedbackTopic, SemMapActionFeedback,
      queue_size = self.semanticMapActionFeedbackQueueSize,
      latch = self.semanticMapActionFeedbackLatch)
  
  def publishSemanticMapActionFeedback(self, feedback, action, object):
    message = SemMapActionFeedback()
    
    message.header = feedback.header
    
    message.id = action.iri
    message.object_acted_on = object.iri
    
    try:
      self.publisher.publish(message)
    except rospy.ROSSerializationException, exception:
      raise Exception(
        "SemMapActionFeedback publication failed: %s" % exception)
  