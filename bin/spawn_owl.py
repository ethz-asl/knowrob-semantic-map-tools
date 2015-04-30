#!/usr/bin/env python

import sys
import argparse

import rospy
import roslib; roslib.load_manifest("knowrob_semantic_map_tools")

from knowrob_semantic_map_tools import *
from knowrob_semantic_map_tools.prolog.queries import *

if __name__ == '__main__':
  rospy.init_node("spawn_owl")
  
  parser = argparse.ArgumentParser(
    description = "Spawn semantic map ontology in OWL format")
  parser.add_argument("-o", "--output-file", metavar = "FILE",
    dest = "output", help = "OWL output filename or '-' for stdout")
  args = parser.parse_args(rospy.myargv()[1:])

  semanticMapClient = semantic_map.Client()
  map = semanticMapClient.map
  
  semanticMapGenerationClient = semantic_map.generation.Client(map)
  owl = semanticMapGenerationClient.owl
  
  if args.output:
    if args.output == "-":
      sys.stdout.write(owl)
    else:
      outputFile = open(args.output, "w");
      outputFile.write(owl)
      outputFile.close()
  
  prologClient = prolog.Client()
  
  owlParseStringQuery = knowrob.OwlParseString(owl)
  owlParseStringQuery.execute(prologClient).finish()

  registerPrefixQuery = rdf.RegisterPrefix("map", map.namespace+"#")
  registerPrefixQuery.execute(prologClient).finish()

  rospy.spin()
  