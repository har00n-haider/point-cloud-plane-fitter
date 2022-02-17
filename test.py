import numpy as np
import open3d as o3d
from ptypes import Color, Plane
from ransac3d import Ransac3d
import utils as u

def TestPlaneAlignment():
  # create y/z plane
  plane = Plane.FromPoints(
    np.array([0,0,0]),
    np.array([0,1,1]),
    np.array([0,0,1])
    )
  v = np.array([2,5,1])
  pv = plane.GetProjectedVectorOnPlane(v)
  geometryList = [
    u.GetLineSetForVector(v, color=[0,1,0]),
    u.GetLineSetForVector(pv, color=[1,0,0]),
    u.GetO3dTriMesh(*plane.GetQuadMesh(size=2))
  ]
  u.Visualize(geometryList)
  return

def TestRandomPointOnPlane():
  geometryList = []
  plane = Plane.FromPointNormal(
    np.array([0,0,0]), 
    np.array([1,1,1]), 
    )
  for i in range(0,50):
    geometryList.append(u.GetCircleMeshAtPoint(plane.GetRandomPointOnPlane(),radius=0.01))
  geometryList.append(u.GetO3dTriMesh(*plane.GetQuadMesh()))
  u.Visualize(geometryList)



