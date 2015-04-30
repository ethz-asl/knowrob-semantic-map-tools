import re

import rospy
from rospy.rostime import *

from tf_conversions import *

from geometry_msgs.msg import *
from visualization_msgs.msg import *

from knowrob_semantic_map_tools.prolog.queries import *

import Exception

class Publisher(object):
  def __init__(self, prologClient, identifiers = []):
    self.markerArrayTopic = rospy.get_param(
      "~publishers/marker_array/topic", "~markers")
    self.markerArrayQueueSize = rospy.get_param(
      "~publishers/marker_array/queue_size", 1)
    self.markerArrayLatch = rospy.get_param(
      "~publishers/marker_array/latch", True)
    self.markerArrayRate = rospy.get_param(
      "~publishers/marker_array/rate", 1.0)
    
    self.frameId = rospy.get_param("~map/frame_id", "/map")
    self.stamp = rospy.get_param("~map/stamp", None)
    
    if self.stamp:
      self.stamp = Time.from_sec(time.mktime(time.strptime(
        self.stamp.value, "%Y%m%dT%H:%M:%S")))
    
    self.labelsEnabled = rospy.get_param(
      "~markers/labels/enabled", False)
    self.labelScale = rospy.get_param(
      "~markers/labels/scale", 1.0)
    self.labelColor = self.getColorParam(
      "~markers/labels/color", 1.0, 1.0, 1.0, 1.0)
    
    self.axesEnabled = rospy.get_param(
      "~markers/axes/enabled", False)
    
    self.boundingBoxesEnabled = rospy.get_param(
      "~markers/bounding_boxes/enabled", False)
    self.boundingBoxColor = self.getColorParam(
      "~markers/bounding_boxes/color", 1.0, 1.0, 1.0, 0.2)
    
    self.modelsEnabled = rospy.get_param(
      "~markers/models/enabled", True)
    self.modelEmbeddedMaterials = rospy.get_param(
      "~markers/models/embedded_materials", True)
    self.modelColor = self.getColorParam(
      "~markers/models/color", 0.6, 0.6, 0.6, 1.0)
    
    self.prologClient = prologClient  
    self.message = self.createMarkerArray(identifiers)
  
    self.publisher = rospy.Publisher(self.markerArrayTopic, MarkerArray,
      queue_size = self.markerArrayQueueSize, latch = self.markerArrayLatch)
    self.timer = rospy.Timer(rospy.Duration(self.markerArrayRate),
      self.timerEvent)
  
  def getColorParam(self, namespace, defaultRed = 0.0, defaultGreen = 0.0,
      defaultBlue = 0.0, defaultAlpha = 0.0):
    color = {}
    
    color["r"] = rospy.get_param(namespace+"/r", defaultRed)
    color["g"] = rospy.get_param(namespace+"/g", defaultGreen)
    color["b"] = rospy.get_param(namespace+"/b", defaultBlue)
    color["a"] = rospy.get_param(namespace+"/a", defaultAlpha)
    
    return color
  
  def createMarkerArray(self, identifiers, withChildren = True):
    if withChildren:
      visualizationFindChildrenQuery = knowrob.VisualizationFindChildren(
        identifiers)
      identifiers = visualizationFindChildrenQuery.execute(
        self.prologClient).children

    message = MarkerArray()    
    self.addMarkers(identifiers, message.markers)
        
    return message

  def publishMarkerArray(self):
    try:
      self.publisher.publish(self.message)
    except rospy.ROSSerializationException, exception:
      raise Exception(
        "MarkerArray publication failed: %s" % exception)

  def addMarkers(self, identifiers, markers):
    visualizatonObjectInfoQuery = knowrob.VisualizationObjectInfo(identifiers)
    visualizatonObjectInfoQuery.execute(self.prologClient)
    
    for info in visualizatonObjectInfoQuery.infos:
      identifier = info["identifier"]
      label = info["label"]
      type = info["type"]
      frame = fromTf(info["tf"])
      pose = toMsg(frame)
      size = info["dimensions"]
      model = info["model"]
      
      if self.modelsEnabled and model:
        markers.append(self.createMeshMarker(pose,
          "models", model, len(markers)))
      if self.axesEnabled:
        markers.extend(self.createAxesMarkers(pose,
          "axes", len(markers)))
      if self.boundingBoxesEnabled and size["x"] > 0.0 and \
          size["y"] > 0.0 and size["z"] > 0.0:
        markers.append(self.createBoundingBoxMarker(pose,
          "bounding_boxes", size, len(markers)))
      if self.labelsEnabled:
        markers.append(self.createLabelMarker(pose,
          "labels", label, len(markers)))
        
  def createAxesMarkers(self, pose, namespace, id):
    dims = ["x", "y", "z"]
    cols = ["r", "g", "b"]
    
    messages = []
    
    for i in [0, 1, 2]:
      message = self.createMarker(pose, namespace, id+i)
    
      message.type = Marker.ARROW
      message.action = Marker.ADD
      
      startPoint = Point()
      message.points.append(startPoint)
      
      endPoint = Point()
      setattr(endPoint, dims[i], 1.0)
      message.points.append(endPoint)
      
      setattr(message.color, cols[i], 1.0)
      message.color.a = 1.0

      message.scale.x = 0.05
      message.scale.y = 0.1
      message.scale.z = 0.0
      
      messages.append(message)
      
    return messages
  
  def createMeshMarker(self, pose, namespace, resource, id):
    message = self.createMarker(pose, namespace, id)
    
    message.type = Marker.MESH_RESOURCE
    message.action = Marker.ADD
    
    message.scale.x = 1.0
    message.scale.y = 1.0
    message.scale.z = 1.0
    
    if not self.modelEmbeddedMaterials:
      message.color.r = self.modelColor["r"]
      message.color.g = self.modelColor["g"]
      message.color.b = self.modelColor["b"]
      message.color.a = self.modelColor["a"]
    
    message.mesh_resource = resource
    message.mesh_use_embedded_materials = self.modelEmbeddedMaterials
    
    return message
  
  def createBoundingBoxMarker(self, pose, namespace, size, id):
    message = self.createMarker(pose, namespace, id)
    
    message.type = Marker.CUBE
    
    message.scale.x = size["x"]
    message.scale.y = size["y"]
    message.scale.z = size["z"]
    
    message.color.r = self.boundingBoxColor["r"]
    message.color.g = self.boundingBoxColor["g"]
    message.color.b = self.boundingBoxColor["b"]
    message.color.a = self.boundingBoxColor["a"]
    
    return message
  
  def createLabelMarker(self, pose, namespace, text, id):
    message = self.createMarker(pose, namespace, id)
    
    message.type = Marker.TEXT_VIEW_FACING    
    
    message.scale.x = self.labelScale
    message.scale.y = self.labelScale
    message.scale.z = self.labelScale
    
    message.color.r = self.labelColor["r"]
    message.color.g = self.labelColor["g"]
    message.color.b = self.labelColor["b"]
    message.color.a = self.labelColor["a"]
    
    message.text = text
    
    return message

  def createMarker(self, pose, namespace, id):
    message = Marker()
    
    message.header.frame_id = self.frameId    
    if self.stamp:
      message.header.stamp = self.stamp
    else:
      message.header.stamp = Time.now()
    
    message.ns = namespace
    message.id = id
    
    message.pose = pose
    
    return message

  def camelCase(self, name):
    sub = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', sub).lower()
  
  def timerEvent(self, event):
    self.publishMarkerArray()
  