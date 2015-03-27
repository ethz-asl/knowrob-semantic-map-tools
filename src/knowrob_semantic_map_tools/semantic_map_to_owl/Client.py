import rospy
from rospy.rostime import *

import time

from geometry_msgs.msg import Pose

from knowrob_semantic_map_msgs.msg import *
from knowrob_semantic_map_msgs.srv import *

import Exception

class Client(object):
  def __init__(self):
    self.generateSemanticMapOWLService = rospy.get_param(
      "~clients/generate_semantic_map_owl/service",
      "/knowrob_semantic_map_to_owl/generate_owl_map")
    
    map = rospy.get_param("~map")
    
    self.map = SemMap()
    if "frame_id" in map:
      self.map.header.frame_id = map["frame_id"]
    else:
      self.map.header.frame_id = "/map"

    if "stamp" in map:
      self.map.header.stamp = Time.from_sec(time.mktime(
        time.strptime(map["stamp"].value, "%Y%m%dT%H:%M:%S")))
    else:
      self.map.header.stamp = Time.now()

    if "namespace" in map:
      self.map.namespace = map["namespace"]
    else:
      self.map.namespace = "http://asl.ethz.ch/example/semantic_map.owl"

    if "prefixes" in map:
      prefixes = map["prefixes"]

      for pref in prefixes:
        prefix = SemMapPrefix()
        
        prefix.name = pref["name"]
        prefix.prefix = pref["prefix"]
        
        self.map.prefixes.append(prefix)
        
    if "imports" in map:
      self.map.imports = map["imports"]

    if "address" in map:
      address = map["address"]
      
      if "room_nr" in address:
        self.map.address.room_nr = str(address["room_nr"])
      if "floor_nr" in address:
        self.map.address.floor_nr = str(address["floor_nr"])
      if "street_nr" in address:
        self.map.address.street_nr = str(address["street_nr"])
      if "street_name" in address:
        self.map.address.street_name = address["street_name"]
      if "city_name" in address:
        self.map.address.city_name = address["city_name"]

    self.object_properties = {}
    if "object_properties" in map:
      object_properties = map["object_properties"]
      
      for key in object_properties:
        value = object_properties[key]
        self.object_properties[key] = value["id"]

    self.data_properties = {}
    if "data_properties" in map:
      data_properties = map["data_properties"]
      
      for key in data_properties:
        value = data_properties[key]
        self.data_properties[key] = value["id"]
        
    if "objects" in map:
      objects = map["objects"]
      
      for obj in objects:
        object = SemMapObject()
        
        object.id = str(obj["id"])
        object.type = obj["type"]
        
        if "size" in obj:
          object.size.x = obj["size"]["x"]
          object.size.y = obj["size"]["y"]
          object.size.z = obj["size"]["z"]

        if "position" in obj:
          object.pose.position.x = obj["position"]["x"];
          object.pose.position.y = obj["position"]["y"];
          object.pose.position.z = obj["position"]["z"];
          
        if "orientation" in obj:
          object.pose.orientation.w = obj["orientation"]["w"]
          object.pose.orientation.x = obj["orientation"]["x"]
          object.pose.orientation.y = obj["orientation"]["y"]
          object.pose.orientation.z = obj["orientation"]["z"]

        if "part_of" in obj:
          object.part_of = str(obj["part_of"])
          
        self.map.objects.append(object)
        
        for key in obj:
          if key in self.object_properties:
            if not isinstance(obj[key], list):
              obj[key] = [obj[key]]
            
            for prop_obj in obj[key]:
              object_property = SemMapObjectProperty()

              object_property.id = self.object_properties[key]
              object_property.subject = object.id
              object_property.object = str(prop_obj)
              
              self.map.object_properties.append(object_property)
          if key in self.data_properties:
            data_property = SemMapDataProperty()

            data_property.id = self.data_properties[key]
            data_property.subject = object.id
            
            if isinstance(obj[key], bool):
              data_property.value_type = SemMapDataProperty.VALUE_TYPE_BOOL
            elif isinstance(obj[key], float):
              data_property.value_type = SemMapDataProperty.VALUE_TYPE_FLOAT
            elif isinstance(obj[key], int):
              data_property.value_type = SemMapDataProperty.VALUE_TYPE_INT
            else:
              data_property.value_type = SemMapDataProperty.VALUE_TYPE_STRING
            
            data_property.value = str(obj[key])
            
            self.map.data_properties.append(data_property)
  
  def getOwl(self):
    rospy.wait_for_service(self.generateSemanticMapOWLService)
    
    try:
      request = rospy.ServiceProxy(self.generateSemanticMapOWLService,
        GenerateSemanticMapOWL)
      response = request(map = self.map)
    except rospy.ServiceException, exception:
      raise Exception(
        "GenerateSemanticMapOWL service request failed: %s" % exception)
      
    return response.owlmap
  
  owl = property(getOwl)
  