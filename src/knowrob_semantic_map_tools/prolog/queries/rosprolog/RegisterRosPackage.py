from knowrob_semantic_map_tools.prolog.queries.Compound import *

class RegisterRosPackage(Compound):
  def __init__(self, package):
    Compound.__init__(self, "register_ros_package", ["'%s'" % package])
  