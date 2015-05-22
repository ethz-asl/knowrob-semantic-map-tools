import re

import rospy
from rospy.rostime import *

from tf_conversions import *

from geometry_msgs.msg import *
from visualization_msgs.msg import *

from interactive_markers.interactive_marker_server import *

from knowrob_semantic_map_tools.prolog.queries import *

import Exception

class Server(object):
  def __init__(self, prologClient, identifiers = [], actionHandler = None):
    self.interactiveMarkerTopic = rospy.get_param(
      "~publishers/interactive_marker/service", "~markers")
    self.interactiveMarkerQueueSize = rospy.get_param(
      "~publishers/interactive_marker/queue_size", 1)
    
    self.frameId = rospy.get_param("~map/frame_id", "/map")
    self.stamp = rospy.get_param("~map/stamp", None)
    
    if self.stamp:
      self.stamp = Time.from_sec(time.mktime(time.strptime(
        self.stamp.value, "%Y%m%dT%H:%M:%S")))
    
    self.handlesEnabled = rospy.get_param(
      "~interactive_markers/handles/enabled", True)
    self.handleScale = rospy.get_param(
      "~interactive_markers/handles/scale", 1.0)
    self.handleEmbeddedMaterials = rospy.get_param(
      "~interactive_markers/handles/embedded_materials", True)
    self.handleColor = self.getColorParam(
      "~interactive_markers/handles/color", 0.6, 0.6, 0.6, 1.0)
    
    self.prologClient = prologClient
    self.actionHandler = actionHandler
    
    self.markers = self.createInteractiveMarkers(identifiers)
    
    self.server = InteractiveMarkerServer(self.interactiveMarkerTopic,
      q_size = self.interactiveMarkerQueueSize)
    
    for marker in self.markers:
      self.server.insert(self.markers[marker]["message"], self.processFeedback)
      
    self.server.applyChanges()
  
  def getColorParam(self, namespace, defaultRed = 0.0, defaultGreen = 0.0,
      defaultBlue = 0.0, defaultAlpha = 0.0):
    color = {}
    
    color["r"] = rospy.get_param(namespace+"/r", defaultRed)
    color["g"] = rospy.get_param(namespace+"/g", defaultGreen)
    color["b"] = rospy.get_param(namespace+"/b", defaultBlue)
    color["a"] = rospy.get_param(namespace+"/a", defaultAlpha)
    
    return color
  
  def createInteractiveMarkers(self, identifiers, withChildren = True):
    if withChildren:
      interactionFindChildrenQuery = knowrob.InteractionFindChildren(
        identifiers)
      identifiers = interactionFindChildrenQuery.execute(
        self.prologClient).children

    markers = {}
    self.addInteractiveMarkers(identifiers, markers)
        
    return markers

  def addInteractiveMarkers(self, identifiers, markers):
    interactionObjectInfoQuery = knowrob.InteractionObjectInfo(identifiers)
    interactionObjectInfoQuery.execute(self.prologClient)
    
    for info in interactionObjectInfoQuery.infos:
      identifier = info["identifier"]
      type = info["type"]
      description = info["label"]
      label = info["label"]
      frame = fromTf(info["tf"])
      pose = toMsg(frame)
      handle = info["handle"]
      actions = info["actions"]

      if self.handlesEnabled and handle:
        name = self.camelCase(label).encode('ascii', 'ignore')

        markers[name] = {}
        self.createInteractiveMarker(markers[name], identifier, pose, name,
          description, label, handle, actions, self.camelCase(type.fragment)+
          "_handles")
      
  def createInteractiveMarker(self, marker, identifier, pose, name,
      description, label, handle, actions, namespace):
    message = InteractiveMarker()
    
    message.header.frame_id = self.frameId    
    if self.stamp:
      message.header.stamp = self.stamp
    else:
      message.header.stamp = Time.now()
    
    message.pose.position = pose.position
    message.pose.orientation.x = 0.0
    message.pose.orientation.y = 0.0
    message.pose.orientation.z = 0.0
    message.pose.orientation.w = 1.0
    
    message.name = name
    message.description = description
    message.scale = 1.0

    marker["controls"] = {}
    self.createControls(marker["controls"], name, pose, handle, namespace)
    for control in marker["controls"]:
      message.controls.append(marker["controls"][control]["message"])
    
    marker["menu_entries"] = {}
    self.createMenuEntries(marker["menu_entries"], label, actions)
    for entry in marker["menu_entries"]:
      message.menu_entries.append(marker["menu_entries"][entry]["message"])
    
    marker["identifier"] = identifier
    marker["message"] = message
  
  def createMenuEntries(self, entries, name, actions):
    if actions:
      for action in actions:
        message = MenuEntry()
    
        message.parent_id = 0
        message.id = len(entries)
        
        if action["title"]:
          if "%s" in action["title"]:
            message.title = action["title"] % name
          else:
            message.title = "%s %s" % (action["title"], name)
        else:
          message.title = "Perform %s on %s" % \
            (action["identifier"].fragment, name)
        
        message.command = str(action["identifier"])
        message.command_type = MenuEntry.FEEDBACK
      
        entries[message.id] = {
          "identifier": action["identifier"],
          "message": message
        }
    else:
      message = MenuEntry()
  
      message.parent_id = 0
      message.id = len(entries)
      message.title = "No actions defined for %s" % name
      message.command_type = MenuEntry.FEEDBACK
    
      entries[message.id] = {
        "identifier": None,
        "message": message
      }
      
  def createControls(self, controls, name, pose, handle, namespace):
    message = InteractiveMarkerControl()
    
    message.name = name
    
    message.orientation.x = 0.0
    message.orientation.y = 0.0
    message.orientation.z = 0.0
    message.orientation.w = 1.0
    
    message.orientation_mode = InteractiveMarkerControl.VIEW_FACING
    message.interaction_mode = InteractiveMarkerControl.MENU
    
    message.markers.append(self.createMeshMarker(namespace, pose,
      handle, len(message.markers)))
    
    message.independent_marker_orientation = False
  
    controls[name] = {
      "message": message
    }
  
  def createMeshMarker(self, namespace, pose, resource, id):
    message = self.createMarker(id)
    
    message.ns = namespace
    message.type = Marker.MESH_RESOURCE
    message.action = Marker.ADD
    
    message.pose.position = pose.position
    message.pose.orientation.x = 0.0
    message.pose.orientation.y = 0.0
    message.pose.orientation.z = 1.0
    message.pose.orientation.w = 0.0
    
    message.scale.x = self.handleScale
    message.scale.y = self.handleScale
    message.scale.z = self.handleScale
    
    if not self.handleEmbeddedMaterials:
      message.color.r = self.handleColor["r"]
      message.color.g = self.handleColor["g"]
      message.color.b = self.handleColor["b"]
      message.color.a = self.handleColor["a"]
    
    message.mesh_resource = resource
    message.mesh_use_embedded_materials = self.handleEmbeddedMaterials
    
    return message
  
  def createMarker(self, id):
    message = Marker()
    
    message.header.frame_id = self.frameId
    if self.stamp:
      message.header.stamp = self.stamp
    else:
      message.header.stamp = Time.now()
      
    message.id = id
    
    return message

  def camelCase(self, name):
    sub = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', sub).lower()
  
  def processFeedback(self, feedback):
    if self.actionHandler:
      marker = self.markers[feedback.marker_name]
      
      if marker["menu_entries"][feedback.menu_entry_id]["identifier"]:
        self.actionHandler(feedback,
          marker["menu_entries"][feedback.menu_entry_id]["identifier"],
          marker["identifier"])
  