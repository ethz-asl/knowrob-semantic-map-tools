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
  owlParseStringQuery.execute(prologClient).finish()
  
  owlIndividualOfQuery = knowrob.OwlIndividualOf("knowrob:'Cupboard'")
  owlIndividualOfQuery.execute(prologClient)
  print "Individuals of type knowrob:'Cupboard':"
  for individual in owlIndividualOfQuery.individuals:
    print individual

  registerPrefixQuery = rdf.RegisterPrefix("map",
    "http://asl.ethz.ch/example/semantic_map.owl#")
  registerPrefixQuery.execute(prologClient).finish()

  objectDimensionsQuery = knowrob.ObjectDimensions("map:'Cupboard1'")
  objectDimensionsQuery.execute(prologClient)
  print "Dimensions of map:'Cupboard1':"
  print objectDimensionsQuery.dimensions

  objectPoseQuery = knowrob.ObjectPose("map:'Cupboard1'")
  objectPoseQuery.execute(prologClient)
  print "Pose of map:'Cupboard1':"
  print objectPoseQuery.pose

  hingedToQuery = knowrob.ObjectProperty("knowrob:hingedTo",
    object = "map:'Cupboard1'")
  hingedToQuery.execute(prologClient)
  print "Subject of object property knowrob:hingedTo for object map:'Cupboard1':"
  print hingedToQuery.subject
  
  hingedToQuery = knowrob.ObjectProperty("knowrob:hingedTo",
    subject = "map:'Door1'")
  hingedToQuery.execute(prologClient)
  print "Object of object property knowrob:hingedTo for subject map:'Door1':"
  print hingedToQuery.object

  purchasePriceQuery = knowrob.DataProperty("knowrob:purchasePrice",
    "map:'Cupboard1'")
  purchasePriceQuery.execute(prologClient)
  print "Value of data property knowrob:purchasePrice for subject map:'Cupboard1':"
  print purchasePriceQuery.value
  