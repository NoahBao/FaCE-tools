import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt


# Load the PLY files (assuming they contain point clouds)
def load_ply(file_path):
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd


# Apply ICP to align two point clouds
def apply_icp(source, target, max_iter=50, threshold=0.02):
    # Perform ICP
    reg_icp = o3d.pipelines.registration.registration_icp(
        source,
        target,
        threshold,
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    )

    # Apply the transformation to the source point cloud
    source.transform(reg_icp.transformation)
    return source, reg_icp.transformation


# Compute the difference in normals between two point clouds
def compare_normals(pc1, pc2):
    if len(pc1.points) > len(pc2.points):
        pc1, pc2 = pc2, pc1  # Ensure pc1 is the smaller one
    pc2_tree = o3d.geometry.KDTreeFlann(pc2)
    pc1_normals = np.asarray(pc1.normals)
    pc2_normals = np.asarray(pc2.normals)
    differences = []
    smallPCD1 = []
    smallPCD2 = []

    for i, point in enumerate(pc1.points):
        _, idx, _ = pc2_tree.search_knn_vector_3d(point, 1)
        closest_point_idx = idx[0]
        normal1 = pc1_normals[i] / np.linalg.norm(pc1_normals[i])
        normal2 = pc2_normals[closest_point_idx] / np.linalg.norm(
            pc2_normals[closest_point_idx]
        )
        # Compute dot product and then angle
        dot_product = np.dot(normal1, normal2)
        # Clip dot product to avoid numerical issues (should be between -1 and 1)
        dot_product = np.clip(dot_product, -1.0, 1.0)

        # Compute the angle in radians
        angle = np.arccos(dot_product)
        differences.append(angle)

    return differences


def plot_normal_differences(differences):
    # Plotting the histogram
    plt.figure(figsize=(8, 6))

    # Create histogram with 20 bins, range from 0 to π (angles in radians)
    plt.hist(differences, bins=20, range=(0, np.pi), edgecolor="black", alpha=0.7)

    # Add labels and title
    plt.xlabel("Angle Difference (radians)", fontsize=12)
    plt.ylabel("Frequency", fontsize=12)
    plt.title("Histogram of Normal Angle Differences", fontsize=14)

    # Show grid
    plt.grid(True)

    # Show the plot
    plt.show()


def color_code_differences_on_pcd(pcd, differences):
    norm_differences = [diff / np.pi for diff in differences]
    colors = [[diff, 1 - diff, 0] for diff in norm_differences]
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


# Load PLY files
ply_file_1 = "meshReferences/021_color.ply"
ply_file_2 = "faceOutput/outputFromMesh.ply"

pcd1 = load_ply(ply_file_1)  # Reference
pcd2 = load_ply(ply_file_2)  # Output

# Apply ICP
# not really necessary in the case that we compare points sampled
# from the reference mesh to the output mesh
# since they should already be aligned
aligned_pcd1, transformation = apply_icp(pcd1, pcd2)


print("Comparing normals...")
normal_differences = compare_normals(aligned_pcd1, pcd2)
print("Normal differences (first 10):", normal_differences[:10])

plot_normal_differences(normal_differences)
differencePC = color_code_differences_on_pcd(pcd2, normal_differences)
o3d.visualization.draw_geometries([differencePC], point_show_normal=False)
