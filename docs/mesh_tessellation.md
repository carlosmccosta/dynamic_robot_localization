# Generation of a point cloud with uniform density using [MeshLab](http://www.meshlab.net):

1. __Scale the point cloud to the correct units (meters)__
     - Render -> Show box corners
          - To see the range in XYZ of the mesh (to confirm if it is in mm, cm, m, ...)
     - Filters -> Normals, curvatures and Orientation -> Transform: Scale, Normalize
          - To make uniform scaling (0.001 in XYZ to change from mm to m)

2. __Preprocess the mesh to ensure that the meshlab algorithms work correctly:__
     - Filters -> Cleaning and Repairing -> Remove Faces from Non Manifold Edges
     - Filters -> Normals, curvatures and Orientation -> Normalize Vertex Normals

3. __Increase the density of triangles using either:__
     1. Mid point Subdivision and subsampling
          * Achieves better accuracy since the generated mesh triangles are on the same surface of their original triangles (uses subdivision)
          * Mid point division usually does not achieve uniform point density, requiring a subsampling algorithm that loses mesh information and generates a point cloud
          - Mid point Subdivision
               - Filters -> Remeshing, Simplification and Reconstruction -> Subdivision Surfaces: Midpoint
                    - Iterations: 100
                    - Edge Threshold (world unit): 0.002 m (for example, but can be + or - depending on the size of the part and the desired accuracy)
          - Subsampling of the mesh for having +- an uniform density of points using either:
               1. - Poisson-disk Sampling (preferred method since it creates points in the middle of the triangles using a Poison distribution)
                    - Filters -> Sampling -> Poisson-disk Sampling
               2. - Filters -> Sampling -> Clustered Vertex Subsampling
               3. - Filters -> Point Set -> Point Cloud Simplification
     2. Uniform Mesh Resampling
          * Achieves better point density uniformity and keeps the mesh information
          * May generate artifacts which cause the generated mesh surface to diverge slightly from the original mesh
          - Filters -> Remeshing, Simplification and Reconstruction -> Uniform Mesh Resampling
               - Precision (world unit): 0.002 m (for example, but can be + or - depending on the size of the part and the desired accuracy)
               - Clean vertices

3. __Post processing__
     - View -> Show Layer Dialog
     - Hide the original mesh
     - Select the view of the mesh and then the points to confirm that the generate mesh / point cloud is adequate
     - Render -> Show Normal -> For confirming that there are normals for the point cloud and they are correct
     - Filters -> Normals, curvatures and Orientation -> Normalize Vertex Normals

4. __Export__
     - File -> Export Mesh As
        - .ply
        - Save Vert Normal
        - Binary encoding
