import random
import open3d as o3d
import numpy as np
import utils
from ptypes import Plane

def Ransac3d(
  points, 
  numberOfIterations = 100,
  tolerance = 1):
  '''
  **** RANSAC 3D IMPLEMENTATION ****
  '''
  maxInlierscount = 0
  for iteration in range(0, numberOfIterations):
    (idx1, idx2, idx3) = utils.GenerateRandomIndicesTuple(points)
    # create a plane from the three points
    currentPlane = Plane.FromPoints(
      points[idx1],
      points[idx2],
      points[idx3]
      )
    # find the count of points lying within tolerance of this plane
    inlierIndices = []
    for i in range(0, len(points)):
      #TODO: do we need to skip the points used to make the plane?
      distance = currentPlane.GetDistance(points[i])
      if distance < tolerance:
        inlierIndices.append(i)
    if len(inlierIndices) > maxInlierscount:
      fitPlane = currentPlane
      fitIndices = inlierIndices
      maxInlierscount = len(inlierIndices)
  return (fitPlane, fitIndices)

