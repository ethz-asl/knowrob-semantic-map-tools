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
      for prefix in map["prefixes"]:
        self.addPrefix(prefix)
        
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
      for key in map["object_properties"]:
        self.object_properties[key] = map["object_properties"][key]["id"]

    self.data_properties = {}
    if "data_properties" in map:
      for key in map["data_properties"]:
        self.data_properties[key] = map["data_properties"][key]["id"]
        
    if "objects" in map:
      for object in map["objects"]:
        self.addObject(object)
    
    self.getSemanticMapServer = rospy.Service(self.getSemanticMapService,
      GetSemanticMap, self.getSemanticMap)
  
  def addPrefix(self, prefix):
    msg = SemMapPrefix()
    
    msg.name = prefix["name"]
    msg.prefix = prefix["prefix"]
    
    self.map.prefixes.append(msg)
    
    return msg
  
  def addObject(self, object):
    msg = SemMapObject()
    
    msg.type = object["type"]
    
    if "id" in object:
      msg.id = str(object["id"])
    else:
      msg.id = "%s_%s" % (IRI(msg.type).shortName, str(uuid.uuid1()))
    
    if "frame_id" in object:
      msg.header.frame_id = object["frame_id"]

    if "stamp" in object:
      msg.header.stamp = Time.from_sec(time.mktime(
        time.strptime(object["stamp"].value, "%Y%m%dT%H:%M:%S")))
    else:
      msg.header.stamp = self.map.header.stamp
    
    if "size" in object:
      if "x" in object["size"]:
        msg.size.x = object["size"]["x"]
      if "y" in object["size"]:
        msg.size.y = object["size"]["y"]
      if "z" in object["size"]:
        msg.size.z = object["size"]["z"]

    if "position" in object:
      if "x" in object["position"]:
        msg.pose.position.x = object["position"]["x"]
      if "y" in object["position"]:
        msg.pose.position.y = object["position"]["y"]
      if "z" in object["position"]:
        msg.pose.position.z = object["position"]["z"]
      
    if "orientation" in object:
      if "w" in object["orientation"]:
        msg.pose.orientation.w = object["orientation"]["w"]
      if "x" in object["orientation"]:
        msg.pose.orientation.x = object["orientation"]["x"]
      if "y" in object["orientation"]:
        msg.pose.orientation.y = object["orientation"]["y"]
      if "z" in object["orientation"]:
        msg.pose.orientation.z = object["orientation"]["z"]

    if "tasks" in object:
      for task in object["tasks"]:
        self.addTask(task, msg.id)
        
    if "actions" in object:
      for action in object["actions"]:
        self.addAction(action, msg.id)
        
    if "part_of" in object:
      msg.part_of = str(object["part_of"])
      
    self.map.objects.append(msg)

    self.addObjectProperties(object, msg.id)
    self.addDataProperties(object, msg.id)
    
    if "parts" in object:
      for part in object["parts"]:
        self.addObject(part).part_of = msg.id

    return msg
          
  def addAction(self, action, object):
    msg = SemMapAction()
    
    if "type" in action:
      msg.type = action["type"]
    else:
      msg.type = "knowrob:Action"
    
    if "id" in action:
      msg.id = str(action["id"])
    else:
      msg.id = "%s_%s" % (IRI(msg.type).shortName, str(uuid.uuid1()))
    
    msg.object_acted_on = object
    
    self.map.actions.append(msg)

    self.addObjectProperties(action, msg.id)
    self.addDataProperties(action, msg.id)
    
    return msg
          
  def addTask(self, task, object):
    msg = SemMapTask()
    
    if "type" in task:
      msg.type = task["type"]
    else:
      msg.type = "knowrob:Action"
    
    if "id" in task:
      msg.id = str(task["id"])
    else:
      msg.id = "%s_%s" % (IRI(msg.type).shortName, str(uuid.uuid1()))
    
    if "quantification" in task:
      if task["quantification"] == "intersection_of":
        msg.quantification = SemMapTask.INTERSECTION_OF
      elif task["quantification"] == "union_of":
        msg.quantification = SemMapTask.UNION_OF
      else:
        raise Exception(
          "Invalid quantification for task [%s]: %s" %
          (msg.id, str(task["quantification"])))
      
    if "ordered" in task:
      msg.ordered = task["ordered"]
    else:
      msg.ordered = True
      
    if "actions" in task:
      for action in task["actions"]:
        msg.actions.append(self.addAction(action, object).id)
    
    self.map.tasks.append(msg)

    self.addObjectProperties(task, msg.id)
    self.addDataProperties(task, msg.id)
    
    return msg
          
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
  