#!/usr/bin/env python

import sys
import time
import argparse

import rospy
import roslib; roslib.load_manifest("knowrob_semantic_map_tools")

from knowrob_semantic_map_tools import *
from knowrob_semantic_map_tools.prolog.queries import *

if __name__ == '__main__':
  rospy.init_node("prolog_test")
    
  parser = argparse.ArgumentParser(
    description = "Semantic map prolog query testing")
  args = parser.parse_args(rospy.myargv()[1:])
  
  prologClient = semantic_map.prolog.Client()
  prologClient.waitForOwlParsed()

  owlIndividualOfQuery = knowrob.OwlIndividualOf("knowrob:'Cupboard'")
  owlIndividualOfQuery.execute(prologClient)
  print "Individuals of type knowrob:'Cupboard':"
  for individual in owlIndividualOfQuery.individuals:
    print individual

  objectDimensionsQuery = knowrob.ObjectDimensions("map:'Cupboard1'")
  objectDimensionsQuery.execute(prologClient)
  print "Dimensions of map:'Cupboard1':"
  print objectDimensionsQuery.dimensions

  objectPoseQuery = knowrob.ObjectPose("map:'Cupboard1'")
  objectPoseQuery.execute(prologClient)
  print "Pose of map:'Cupboard1':"
  print objectPoseQuery.pose

  handlePathQuery = knowrob.GetHandlePath("map:'Door1'")
  handlePathQuery.execute(prologClient)
  print "Handle path of map:'Door1':"
  print handlePathQuery.path

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

  doorActionQuery = knowrob.ActionProperty("knowrob:objectActedOn",
    object = "knowrob:'Door'")
  doorActionQuery.execute(prologClient)
  print "Actions of action property knowrob:objectActedOn with value knowrob:'Door':"
  for action in doorActionQuery.actions:
    print action

  purchasePriceQuery = knowrob.DataProperty("knowrob:purchasePrice",
    "map:'Cupboard1'")
  purchasePriceQuery.execute(prologClient)
  print "Value of data property knowrob:purchasePrice for subject map:'Cupboard1':"
  print purchasePriceQuery.value

  frameQuery = knowrob.MapObjectFrame("map:'Cupboard1'")
  frameQuery.execute(prologClient)
  print "Tf frame of object map:'Cupboard1':"
  print frameQuery.frame

  stampQuery = knowrob.MapObjectStamp("map:'Cupboard1'")
  stampQuery.execute(prologClient)
  print "Timestamp of object map:'Cupboard1':"
  print time.ctime(stampQuery.stamp.secs)
  