#!/usr/bin/env python

import sys
import argparse
import re

import rospy
import roslib; roslib.load_manifest("knowrob_semantic_map_tools")

from knowrob_semantic_map_tools import *

def handleOpeningADoor(feedback):
  print "Opening door %s" % feedback.object_acted_on

def handleClosingADoor(feedback):
  print "Closing door %s" % feedback.object_acted_on

if __name__ == '__main__':
  rospy.init_node("action_test")
    
  parser = argparse.ArgumentParser(
    description = "Semantic map action testing")
  args = parser.parse_args(rospy.myargv()[1:])
  
  actionSubscriber = semantic_map.action.Subscriber()
  actionSubscriber.addActionHandler(re.compile(".*#OpeningADoor.*"),
    handleOpeningADoor)
  actionSubscriber.addActionHandler(re.compile(".*#ClosingADoor.*"),
    handleClosingADoor)
  
  rospy.spin()
  