import rospy

from knowrob_semantic_map_msgs.msg import *

from knowrob_semantic_map_tools.prolog.IRI import *

import Exception

class Subscriber(object):
  def __init__(self, actionHandlers = {}):
    self.semanticMapActionTopic = rospy.get_param(
      "~subscribers/semantic_map_action/topic", "~actions")
    self.semanticMapActionQueueSize = rospy.get_param(
      "~subscribers/semantic_map_action/queue_size", 1)

    self.actionHandlers = actionHandlers

    self.subscriber = rospy.Subscriber(self.semanticMapActionTopic,
      SemMapAction, queue_size = self.semanticMapActionQueueSize,
      callback = self.semanticMapActionCallback)
  
  def addActionHandler(self, action, handler):
    if isinstance(action, IRI):
      self.actionHandlers[IRI(action).fragment] = handler
      return True
    elif isinstance(action, str):
      self.actionHandlers[action] = handler
      return True
    else:
      return False
  
  def semanticMapActionCallback(self, message):
    action = IRI(message.id).fragment
    
    if action in self.actionHandlers:
      self.actionHandlers[action](message)
  