#!/usr/bin/env python

import sys
import argparse

import rospy
import roslib; roslib.load_manifest("knowrob_semantic_map_tools")

from knowrob_semantic_map_tools import *

def handleOpeningADoor(action):
  print "Opening door %s" % action.object_acted_on.id

def handleClosingADoor(action):
  print "Closing door %s" % action.object_acted_on.id

if __name__ == '__main__':
  rospy.init_node("action_test")
    
  parser = argparse.ArgumentParser(
    description = "Semantic map action testing")
  args = parser.parse_args(rospy.myargv()[1:])
  
  actionSubscriber = semantic_map.action.Subscriber()
  actionSubscriber.addActionHandler("OpeningADoor", handleOpeningADoor)
  actionSubscriber.addActionHandler("ClosingADoor", handleClosingADoor)
  
  rospy.spin()
  