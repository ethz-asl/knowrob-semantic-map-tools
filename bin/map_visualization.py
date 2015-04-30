#!/usr/bin/env python

import sys
import argparse

import rospy
import roslib; roslib.load_manifest("knowrob_semantic_map_tools")

from knowrob_semantic_map_tools import *

if __name__ == '__main__':
  rospy.init_node("map_visualization")
  
  parser = argparse.ArgumentParser(
    description = "Visualize semantic map using markers")
  args = parser.parse_args(rospy.myargv()[1:])
  
  prologClient = semantic_map.prolog.Client()
  prologClient.waitForOwlParsed()
  
  owlIndividualOfQuery = knowrob.OwlIndividualOf(
    "knowrob:'SemanticEnvironmentMap'")
  map = owlIndividualOfQuery.execute(prologClient).individual
  
  semanticMapVisualizationPublisher = semantic_map.visualization.Publisher(
    prologClient, [map])
    
  rospy.spin()
  