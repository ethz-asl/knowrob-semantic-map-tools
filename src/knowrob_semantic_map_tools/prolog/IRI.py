class IRI(object):
  def __init__(self, iri):
    self.iri = iri

  def getPrefix(self):
    split = self.iri.split(":")
    
    if len(split) > 0:
      if len(split) > 1:
        if len(split[1]) > 0 and split[1][0] != "/":
          return split[0]
      else:
        return split[0]
    
    return None
  
  prefix = property(getPrefix)

  def getSuffix(self):
    if self.prefix:    
      split = self.iri.split(":")
    
      if len(split) > 1 and len(split[1]) > 0:
        return split[1]
    
    return None
  
  suffix = property(getSuffix)

  def getScheme(self):
    split = self.iri.split(":")
    
    if len(split) > 1 and len(split[1]) > 0 and split[1][0] == "/":
      return split[0]
    
    return None
  
  scheme = property(getScheme)

  def getFragment(self):
    split = self.iri.split("#")
    
    if len(split) > 1 and len(split[1]) > 0:
      return split[1]
    
    return None
  
  fragment = property(getFragment)

  def __str__(self):
    if self.prefix:
      return "%s:'%s'" % (self.prefix, self.suffix)
    else:
      return "'%s'" % self.iri
  