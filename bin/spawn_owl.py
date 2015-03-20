#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("knowrob_semantic_map_tools")

from knowrob_semantic_map_tools import *
from knowrob_semantic_map_tools.prolog.queries import *

if __name__ == '__main__':
  rospy.init_node("spawn_owl")
  
  semanticMapToOwlClient = semantic_map_to_owl.Client()
  prologClient = prolog.Client()
  
  owlParseStringQuery = knowrob.OwlParseString(semanticMapToOwlClient.owl)
  owlParseStringQuery.execute(prologClient)
  
  registerRosPackageQuery = rosprolog.RegisterRosPackage("knowrob_vis")
  registerRosPackageQuery.execute(prologClient)
  visualizationServerQuery = knowrob.VisualizationServer()
  visualizationServerQuery.execute(prologClient)
  
  addObjectsQuery = knowrob.OwlIndividualOf("MapObjects",
    "knowrob:'SemanticEnvironmentMap'")
  addObjectsQuery &= knowrob.AddObjectWithChildren("MapObjects")
  addObjectsQuery.execute(prologClient)
  