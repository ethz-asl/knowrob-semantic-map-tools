#!/usr/bin/env python

import sys
import argparse

import rospy
import roslib; roslib.load_manifest("knowrob_semantic_map_tools")

from knowrob_semantic_map_tools import *
from knowrob_semantic_map_tools.prolog.queries import *

if __name__ == '__main__':
  rospy.init_node("spawn_owl")
  
  parser = argparse.ArgumentParser(description = "Spawn semantic map OWL")
  parser.add_argument("-o", "--output-file", metavar = "FILE",
    dest = "output", help = "OWL output filename or '-' for stdout")
  print rospy.myargv()
  args = parser.parse_args(rospy.myargv()[1:])
  
  semanticMapToOwlClient = semantic_map_to_owl.Client()  
  owl = semanticMapToOwlClient.owl
  
  if args.output:
    if args.output == "-":
      sys.stdout.write(owl)
    else:
      outputFile = open(args.output, "w");
      outputFile.write(owl)
      outputFile.close()
  
  prologClient = prolog.Client()
  owlParseStringQuery = knowrob.OwlParseString(owl)
  owlParseStringQuery.execute(prologClient)
  
  registerRosPackageQuery = rosprolog.RegisterRosPackage("knowrob_vis")
  registerRosPackageQuery.execute(prologClient)
  visualizationServerQuery = knowrob.VisualizationServer()
  visualizationServerQuery.execute(prologClient)
  
  addObjectsQuery = knowrob.OwlIndividualOf("MapObjects",
    "knowrob:'SemanticEnvironmentMap'")
  addObjectsQuery &= knowrob.AddObjectWithChildren("MapObjects")
  addObjectsQuery.execute(prologClient)
  