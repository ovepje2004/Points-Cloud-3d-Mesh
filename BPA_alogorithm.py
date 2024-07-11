import open3d as o3d
import numpy as np

path = ""
input_path = path + "points_data/"
output_path = path + "output/"
dataname = "sai_point_cloud.ply" #private datafile

pcd = o3d.io.read_point_cloud(input_path + dataname)

num_points = len(pcd.points)
print(f"Number of points in the point cloud: {num_points}")

pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 3 * avg_dist

bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    pcd,
    o3d.utility.DoubleVector([radius, radius * 2])
)

dec_mesh = bpa_mesh.simplify_quadric_decimation(10000000)

dec_mesh.remove_degenerate_triangles()
dec_mesh.remove_duplicated_triangles()
dec_mesh.remove_duplicated_vertices()
dec_mesh.remove_non_manifold_edges()

o3d.io.write_triangle_mesh(output_path + "bpa_mesh.ply", dec_mesh)
o3d.visualization.draw_geometries([dec_mesh], window_name="BPA Mesh")
