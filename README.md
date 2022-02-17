# Comments

## Things to improve 
- Would like to try out the Hough algorithm to see how it compares in customisation, robustness to outliers
- Fix the area estimation. Possibilities:
  - pcd triangulation for a 2d polygon in 3d space
  - Using vertical planes to clip roof planes 
- Planes that are on similar enough should be grouped if the are within a certain area (saw the algorithm split one flat section into two). Probably is a way to avoid this happening in the core algorithm though

## General thoughts
- Tweaking the figures on the algorithm to match the specific use case is important. i.e. Ransac threshold to within the flatness of a roof + lidar error margins. 
- Preprocessing the point cloud seems like a very powerful tool. Gives the main algorithm the best chance to shine

# Notes 

## Formats
- PLY - Polygon file format - format to store 3D data
- OBJ - format to store 3D data, supports lines/trimeshes geometry, materials, textures
- gltf/glb - supports animation, scene graph, textures

## Python libraries
- Open3d - library for 3D data processing
- Trimesh - library for maniplating tri based meshes
- PDAL - library for manipulating point cloud data

## Cleaning algorithms
 - KD-Tree statiscical outlier removal (https://www.mdpi.com/1424-8220/21/11/3703/htm)

## Plane fitting algorithms:
- least squares
- RANSAC
- Hough
- Principal component analysis

### Key terms:
- Singular value decomposition
- Covariance matrix

### Refs:
- https://stackoverflow.com/questions/35726134/3d-plane-fitting-algorithms
- https://stackoverflow.com/questions/28731442/detecting-set-of-planes-from-point-cloud
- https://stackoverflow.com/questions/38754668/plane-fitting-in-a-3d-point-cloud
