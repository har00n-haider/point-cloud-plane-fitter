import random
import numpy as np
import math
from utils import Normalize
from enum import Enum

class Color():
  red = [1.,0.,0.]
  green = [1.,0.,0.]
  blue = [1.,0.,0.]
  lightgrey = [0.7,0.7,0.7]
  black = [0.,0.,0.]
  white = [1.,1.,1.]
  def Random():
    return [random.random(),random.random(),random.random()] 



class Plane:
  '''
  Values are mapped to the plane equation as:
  ax + by + cz + d = 0
  '''
  def __init__(self, a=0, b=0, c=0, d=0):
    self.normal = Normalize(np.array([a,b,c]))
    self.center = self.normal * -d
    self.a = a
    self.b = b
    self.c = c
    self.d = d
    return
  def FromPointNormal(point, normal):
    # use equation of plane to calculate d substituting p1
    # d = - ax - by - cz
    a = normal[0]
    b = normal[1]
    c = normal[2]
    d = -normal[0]*point[0] -normal[1]*point[1] - normal[2]*point[2]   
    return Plane(a,b,c,d)
  def FromPoints(p1, p2, p3):
    v1 = p1 - p2
    v2 = p1 - p3
    normal = Normalize(np.cross(v1, v2))
    return Plane.FromPointNormal(p1, normal)
  def GetDistance(self, point):
    numerator = abs(self.a*point[0] + self.b*point[1] + self.c*point[2] + self.d)
    denominator = math.sqrt(
      self.a*self.a +
      self.b*self.b + 
      self.c*self.c)
    return numerator/denominator
  def GetProjectedVectorOnPlane(self, vector):
    # project of the vector on the plane normal
    normalProjVector = self.normal * (np.dot(vector,self.normal) / (abs(np.linalg.norm(self.normal)) * abs(np.linalg.norm(self.normal))))
    # subtract projected vec from original to get 
    planeProjVector = vector - normalProjVector
    return planeProjVector
  def GetRandomPointOnPlane(self):
    x = random.random()
    y = random.random()
    z = (-self.a*x -self.b*y - self.d)/self.c
    return np.array([x,y,z])
  def GetQuadMesh(self, center=None, size=10, alignAxis=None):
    # defaults
    alignAxis = np.array([0,1,0]) if alignAxis is None else alignAxis
    center = self.center if center is None else center

    planeUp = Normalize(self.GetProjectedVectorOnPlane(alignAxis))
    planeRight = Normalize(np.cross(self.normal, planeUp))
    planeUpS = planeUp * size * 0.5
    planeRightS = planeRight * size * 0.5
    # 0     1
    # -------
    # |     |
    # |     |
    # -------
    # 2     3
    verts=np.array([
      center + planeUpS - planeRightS, # 0
      center + planeUpS + planeRightS, # 1
      center - planeUpS - planeRightS, # 2
      center - planeUpS + planeRightS, # 3
    ])
    tris=np.array([
      [0, 1, 3], 
      [0, 3, 2], 
    ]).astype(np.int32)
    return (verts, tris)




