import rospy
from rospy.rostime import *

import time
import uuid

from geometry_msgs.msg import Pose

from knowrob_semantic_map_msgs.msg import *
from knowrob_semantic_map_msgs.srv import *

from knowrob_semantic_map_tools.prolog.IRI import *

import Exception

class Server(object):
  def __init__(self):
    self.getSemanticMapService = rospy.get_param(
      "~servers/get_semantic_map/service", "~get_map")
    
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
      self.map.namespace = "http://asl.ethz.ch/knowrob/example_semantic_map.owl"

    if "id" in map:
      self.map.id = map["id"]
    else:
      self.map.id = "ExampleSemanticMap"
      
    prefix = SemMapPrefix()
    prefix.name = "map"
    prefix.prefix = self.map.namespace+"#"
    self.map.prefixes.append(prefix)

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
        
        object.type = obj["type"]
        
        if "id" in obj:
          object.id = str(obj["id"])
        else:
          object.id = "%s_%s" % (IRI(object.type).shortName,
            str(uuid.uuid1()))
        
        if "frame_id" in obj:
          object.header.frame_id = obj["frame_id"]

        if "stamp" in obj:
          object.header.stamp = Time.from_sec(time.mktime(
            time.strptime(obj["stamp"].value, "%Y%m%dT%H:%M:%S")))
        else:
          object.header.stamp = self.map.header.stamp
        
        if "size" in obj:
          if "x" in obj["size"]:
            object.size.x = obj["size"]["x"]
          if "y" in obj["size"]:
            object.size.y = obj["size"]["y"]
          if "z" in obj["size"]:
            object.size.z = obj["size"]["z"]

        if "position" in obj:
          if "x" in obj["position"]:
            object.pose.position.x = obj["position"]["x"]
          if "y" in obj["position"]:
            object.pose.position.y = obj["position"]["y"]
          if "z" in obj["position"]:
            object.pose.position.z = obj["position"]["z"]
          
        if "orientation" in obj:
          if "w" in obj["orientation"]:
            object.pose.orientation.w = obj["orientation"]["w"]
          if "x" in obj["orientation"]:
            object.pose.orientation.x = obj["orientation"]["x"]
          if "y" in obj["orientation"]:
            object.pose.orientation.y = obj["orientation"]["y"]
          if "z" in obj["orientation"]:
            object.pose.orientation.z = obj["orientation"]["z"]

        if "actions" in obj:
          for act in obj["actions"]:
            action = SemMapAction()
            
            action.type = act["type"]
            
            if "id" in act:
              action.id = str(act["id"])
            else:
              action.id = "%s_%s" % (IRI(action.type).shortName,
                str(uuid.uuid1()))
            
            action.object_acted_on = object.id
            
            self.map.actions.append(action)

            self.addObjectProperties(act, action.id)
            self.addDataProperties(act, action.id)
            
        if "parts" in obj:
          for part in obj["parts"]:
            part["part_of"] = object.id
          
          objects.extend(obj["parts"])
          del obj["parts"]

        if "part_of" in obj:
          object.part_of = str(obj["part_of"])
          
        self.map.objects.append(object)

        self.addObjectProperties(obj, object.id)
        self.addDataProperties(obj, object.id)
    
    self.getSemanticMapServer = rospy.Service(self.getSemanticMapService,
      GetSemanticMap, self.getSemanticMap)
  
  def addObjectProperties(self, properties, subject):
    for key in properties:
      if key in self.object_properties:
        if not isinstance(properties[key], list):
          properties[key] = [properties[key]]
        
        for prop_obj in properties[key]:
          object_property = SemMapObjectProperty()

          object_property.id = self.object_properties[key]
          object_property.subject = subject
          object_property.object = str(prop_obj)
          
          self.map.object_properties.append(object_property)
  
  def addDataProperties(self, properties, subject):
    for key in properties:
      if key in self.data_properties:
        data_property = SemMapDataProperty()

        data_property.id = self.data_properties[key]
        data_property.subject = subject
        
        if isinstance(properties[key], bool):
          data_property.value_type = SemMapDataProperty.VALUE_TYPE_BOOL
        elif isinstance(properties[key], float):
          data_property.value_type = SemMapDataProperty.VALUE_TYPE_FLOAT
        elif isinstance(properties[key], int):
          data_property.value_type = SemMapDataProperty.VALUE_TYPE_INT
        else:
          data_property.value_type = SemMapDataProperty.VALUE_TYPE_STRING
        
        data_property.value = str(properties[key])
        
        self.map.data_properties.append(data_property)
      
  def getSemanticMap(self, request):
    return GetSemanticMapResponse(self.map)
  