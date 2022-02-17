import numpy as np
from numpy.core.defchararray import array
import open3d as o3d
from ptypes import Color, Plane
from ransac3d import Ransac3d
import utils as u
import test as t
import os

flatRoofPath = R'data\flat_roof.obj'
flatRoofCroppedPath = R'data\flat_roof_cropped.ply'
pitchedRoofPath = R'data\pitched_roof.obj'

def RunRansac3d(
  pcd, 
  outdir='out', 
  planeExtractions=30, 
  ransacIter=700,
  ransacTol=0.04, 
  drawPlanes=False):
  # we only care about segments with centroid 2m above
  # mesh centroid, so remove points below it
  # assumptions:
  # - house pcd is roughly axis orientated 
  # - ignore case where roof extends below centroid
  meshCentroid = u.GetCenterOfPcd(pcd)
  pcdArr = np.asarray(pcd.points)
  idxs = np.where(pcdArr[:,1] > meshCentroid[1])[0].tolist()
  pcdFiltered = pcd.select_by_index(idxs)
  pcdFiltered.paint_uniform_color(Color.lightgrey)
  geometryList = []
  for i in range(0, planeExtractions):
    geometryListLocal = []
    color = Color.Random()
    # extract a region/plane
    plane, fitIndices = Ransac3d(
      pcdFiltered.points,
      numberOfIterations=ransacIter, 
      tolerance=ransacTol)
    # filter the matched points for the plane
    fitPcd = pcdFiltered.select_by_index(fitIndices)
    fitPcd.paint_uniform_color(color)
    fitPcdCenter = u.GetCenterOfPcd(fitPcd)
    fitPcdCenterMesh = u.GetCircleMeshAtPoint(fitPcdCenter)
    # calculate the data for the matched points
    orientation = abs(np.dot(plane.normal, np.array([0,1,0])))
    # area estimation seems poor, using the orientation test as an alternative
    # fitArea = u.EstimateAreaOfPcdRegion(fitPcd)
    fitArea = 50
    relativeHeight = fitPcdCenter[1] - meshCentroid[1]
    exportRegion = (
      fitArea > 6 and fitArea < 100 and
      relativeHeight > 2 and
      # roughly within 70 degrees of y axis
      orientation > 0.3        
    )
    # print summary
    print(
      f'found plane{i}: {plane.a:0.2f}x + {plane.b:0.2f}y + {plane.c:0.2f}z + {plane.d:0.2f} = 0' +
      f', area = {fitArea:0.2f}, relative height = {relativeHeight:0.2f}, ' +
      f', ori = {orientation:0.2f}, exported = {exportRegion}'
    )
    if(exportRegion):
      # write to file     
      o3d.io.write_point_cloud(os.path.join(outdir, f'segment_{i}.ply'), fitPcd )
      # add to render list
      geometryListLocal = [fitPcd, fitPcdCenterMesh]
      if drawPlanes:
        planeMesh = u.GetO3dTriMesh(*plane.GetQuadMesh(fitPcdCenter), color=color)
        geometryListLocal.append(planeMesh)
      geometryList.extend(geometryListLocal)
      # remove the matched points from the pcd
      pcdFiltered = pcdFiltered.select_by_index(fitIndices, invert=True)
    else:
      # write to file     
      o3d.io.write_point_cloud(os.path.join(outdir, f'reg_segment_{i}.ply'), fitPcd )
  u.Visualize(geometryList)
  return

RunRansac3d(u.LoadPcdFromObj(flatRoofPath))

# # reject investigation
# geometryList = []
# for file in os.listdir('out'):
#   if 'reg' in file:
#     filePath = 'out\\' + file
#     print(f'opening file {filePath}')
#     pcd = o3d.io.read_point_cloud(filePath)
#     geometryList.append(pcd)
#     u.Visualize([pcd])
# # u.Visualize(geometryList)




