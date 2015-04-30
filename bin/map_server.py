#!/usr/bin/env python

import sys
import argparse

import rospy
import roslib; roslib.load_manifest("knowrob_semantic_map_tools")

from knowrob_semantic_map_tools import *

if __name__ == '__main__':
  rospy.init_node("map_server")
  
  parser = argparse.ArgumentParser(
    description = "Semantic map server")
  args = parser.parse_args(rospy.myargv()[1:])

  semanticMapServer = semantic_map.Server()

  rospy.spin()
  