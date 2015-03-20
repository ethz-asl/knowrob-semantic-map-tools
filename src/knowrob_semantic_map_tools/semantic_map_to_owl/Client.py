import rospy
from rospy.rostime import *

import time

from tf_conversions.posemath import *

from geometry_msgs.msg import Pose

from knowrob_semantic_map_msgs.msg import SemMap, SemMapObject
from knowrob_semantic_map_msgs.srv import GenerateSemanticMapOWL

import Exception

class Client(object):
  def __init__(self):
    self.generateSemanticMapOWLService = rospy.get_param(
      "~clients/generate_semantic_map_owl/service",
      "/knowrob_semantic_map_to_owl/generate_owl_map")
    
    map = rospy.get_param("~map")
    
    self.map = {}
    if "frame_id" in map:
      self.map["frame_id"] = map["frame_id"]
    else:
      self.map["frame_id"] = "http://www.example.com/map.owl#"

    if "stamp" in map:
      self.map["stamp"] = Time.from_sec(time.mktime(
        time.strptime(map["stamp"].value, "%Y%m%dT%H:%M:%S")))
    else:
      self.map["stamp"] = Time.now()

    self.map["objects"] = []
    if "objects" in map:
      objects = map["objects"]
      
      for obj in objects:
        object = {}
        
        object["id"] = obj["id"]
        object["type"] = obj["type"]
        
        if "size" in obj:
          object["size"] = [obj["size"]["x"], 
                            obj["size"]["y"],
                            obj["size"]["z"]]
        else:
          object["size"] = [0, 0, 0]

        position = Point()
        if "position" in obj:
          position.x = obj["position"]["x"];
          position.y = obj["position"]["y"];
          position.z = obj["position"]["z"];
        else:
          position.x = 0
          position.y = 0
          position.z = 0
          
        orientation = Quaternion()
        if "orientation" in obj:
          orientation.w = obj["orientation"]["w"]
          orientation.x = obj["orientation"]["x"]
          orientation.y = obj["orientation"]["y"]
          orientation.z = obj["orientation"]["z"]
        else:
          orientation.w = 1
          orientation.x = 0
          orientation.y = 0
          orientation.z = 0

        pose = Pose()
        pose.position = position
        pose.orientation = orientation
        
        frame = fromMsg(pose)
        object["pose"] = toMatrix(frame).tolist()
        
        if "part_of" in obj:
          object["part_of"] = obj["part_of"]
        else:
          object["part_of"] = 0
          
        self.map["objects"].append(object)
  
  def getOwl(self):
    map = SemMap()

    map.header.frame_id = self.map["frame_id"]
    map.header.stamp = self.map["stamp"]
    
    for object in self.map["objects"]:
      obj = SemMapObject()
      
      obj.id = object["id"]
      obj.type = object["type"]
      
      obj.depth = object["size"][1]
      obj.width = object["size"][0]
      obj.height = object["size"][2]
    
      obj.pose = [T_ij for T_i in object["pose"] for T_ij in T_i]
      obj.partOf = object["part_of"]
      
      map.objects.append(obj)
      
    rospy.wait_for_service(self.generateSemanticMapOWLService)
    
    try:
      request = rospy.ServiceProxy(self.generateSemanticMapOWLService,
        GenerateSemanticMapOWL)
      response = request(map = map)
    except rospy.ServiceException, exception:
      raise Exception(
        "GenerateSemanticMapOWL service request failed: %s" % exception)
      
    return response.owlmap
  
  owl = property(getOwl)
  