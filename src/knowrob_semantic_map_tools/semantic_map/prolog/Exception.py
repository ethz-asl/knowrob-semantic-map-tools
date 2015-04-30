from knowrob_semantic_map_tools.prolog.Exception import Exception as Base

class Exception(Base):
  def __init__(self, message):
    Base.__init__(self, message)
