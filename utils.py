import random
import numpy as np
import open3d as o3d

def Normalize(v):
  epsilon = np.finfo(float).eps
  norm = np.linalg.norm(v)
  if norm < epsilon:
      return v
  return v / norm

def GetLineSetForVector(vector, pos=None, color=None):
  # defaults
  color = [1, 0, 0] if color is None else color
  pos = [0, 0, 0] if pos is None else pos
  points = [
    pos,
    pos + vector,
  ]
  lines = [
      [0, 1],
  ]
  colors = [color for i in range(len(lines))]
  line_set = o3d.geometry.LineSet(
      points=o3d.utility.Vector3dVector(points),
      lines=o3d.utility.Vector2iVector(lines),
  )
  line_set.colors = o3d.utility.Vector3dVector(colors)
  return line_set

def GetCircleMeshAtPoint(point, radius=0.3, color=None):
  color = [1, 0, 0] if color is None else color
  mesh = o3d.geometry.TriangleMesh.create_sphere(radius, resolution=10)
  mesh.translate((point[0], point[1], point[2]))
  mesh.paint_uniform_color(color)
  return mesh

def GetCenterOfPcd(pcd):

  return np.mean(pcd.points, axis=0)

def GenerateRandomIndicesTuple(points, size=3):
  indices = []
  indices.append(random.choice([x for x in range(0, len(points))]))
  for y in range(1,size):
    indices.append(random.choice([x for x in range(0, len(points)) if x not in indices]))
  return tuple(indices)

def CleanPcd(originalCloud):
  # denoise the point cloud
  print("showing noise removed points")
  cl, denoisedInd = originalCloud.remove_statistical_outlier(nb_neighbors=6, std_ratio=2.0)
  denoisedCloud = originalCloud.select_by_index(denoisedInd)
  noise_cloud = originalCloud.select_by_index(denoisedInd, invert=True)
  # noise_cloud.paint_uniform_color([0, 0, 0])
  # o3d.visualization.draw_geometries([denoised_cloud, noise_cloud])
  return denoisedCloud

def CropGeometry(pcd):
    print("Demo for manual geometry cropping")
    print(
        "1) Press 'Y' twice to align geometry with negative direction of y-axis"
    )
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry and to save it")
    print("5) Press 'F' to switch to freeview mode")
    o3d.visualization.draw_geometries_with_editing([pcd])

def Visualize(geometryList):
  # vis = o3d.visualization.Visualizer()
  # vis.create_window()
  # vis.add_geometry(geometry)
  # vis.run()
  # vis.destroy_window()bm
  axes = o3d.geometry.TriangleMesh.create_coordinate_frame(1.0, [0., 0., 0.])
  geometryList.append(axes)
  o3d.visualization.draw_geometries(geometryList, mesh_show_back_face=True)
  return

def GetO3dTriMesh(verts, tris, color=None):
  color = [0.2, 0.2, 0.2] if color is None else color
  mesh = o3d.geometry.TriangleMesh()
  mesh.vertices = o3d.utility.Vector3dVector(verts)
  mesh.triangles = o3d.utility.Vector3iVector(tris)
  mesh.paint_uniform_color(color)
  return mesh

def LoadPcdFromObj(filePath, noOfPoints=15000, uniform=True):  
  mesh = o3d.io.read_triangle_mesh(filePath)
  # orient with open3d viewer
  R = mesh.get_rotation_matrix_from_xyz((-np.pi / 2, 0, 0))
  mesh.rotate(R, center=(0, 0, 0))
  if(uniform):
    pcd = mesh.sample_points_uniformly(number_of_points=noOfPoints)
  else:
    pcd = mesh.sample_points_poisson_disk(number_of_points=noOfPoints, init_factor=2)
  return pcd

def EstimateAreaOfPcdRegion(pcd):
  mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, 300)
  mesh.compute_vertex_normals()
  return mesh.get_surface_area()/2.2