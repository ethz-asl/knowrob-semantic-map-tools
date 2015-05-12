import re

import rospy

from knowrob_semantic_map_msgs.msg import *

from knowrob_semantic_map_tools.prolog.IRI import *

import Exception

class Subscriber(object):
  def __init__(self, actionHandlers = {}):
    self.semanticMapActionFeedbackTopic = rospy.get_param(
      "~subscribers/semantic_map_action_feedback/topic", "~actions")
    self.semanticMapActionFeedbackQueueSize = rospy.get_param(
      "~subscribers/semantic_map_action_feedback/queue_size", 1)

    self.actionHandlers = actionHandlers

    self.subscriber = rospy.Subscriber(
      self.semanticMapActionFeedbackTopic, SemMapActionFeedback,
      queue_size = self.semanticMapActionFeedbackQueueSize,
      callback = self.semanticMapActionFeedbackCallback)
  
  def addActionHandler(self, action, handler):
    if isinstance(action, IRI):
      regex = re.compile(re.escape(str(action)))
      self.actionHandlers[regex] = handler
      return True
    elif isinstance(action, str):
      regex = re.compile(re.escape(action))
      self.actionHandlers[regex] = handler
      return True
    elif isinstance(action, type(re.compile("x"))):
      self.actionHandlers[action] = handler
      return True
    else:
      return False
  
  def semanticMapActionFeedbackCallback(self, message):
    action = message.id

    handlers = [
      self.actionHandlers[regex] for regex in self.actionHandlers
      if regex.match(action)
    ]
    
    for handler in handlers:
      handler(message)
  