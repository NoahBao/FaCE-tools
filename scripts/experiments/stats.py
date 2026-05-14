import open3d as o3d
import numpy as np


# Compute the difference in normals between two point clouds
def compareNormalsGtEst(
    groundTruthPath: str, estimatesPath: str
) -> tuple[list[float], float, float]:
    """
    Compares the normals of the ground truth point cloud with the normals of the estimated point cloud.
    Returns a list of angle differences in radians, the total number of flipped normals, and the percentage of flipped normals.
    """
    groundTruth = o3d.io.read_point_cloud(groundTruthPath)
    estimates = o3d.io.read_point_cloud(estimatesPath)
    print(
        "Ground truth has {} points; estimates has {} points.".format(
            len(groundTruth.points), len(estimates.points)
        )
    )
    # if len(pc1.points) > len(pc2.points):
    #     pc1, pc2 = pc2, pc1  # Ensure pc1 is the smaller one

    gt_tree = o3d.geometry.KDTreeFlann(groundTruth)
    gt_normals = np.asarray(groundTruth.normals)
    es_normals = np.asarray(estimates.normals)
    differences = []

    total_flipped = 0

    for i, point in enumerate(estimates.points):
        _, idx, _ = gt_tree.search_knn_vector_3d(point, 1)
        closest_point_idx = idx[0]
        gt_normal = gt_normals[closest_point_idx] / np.linalg.norm(
            gt_normals[closest_point_idx]
        )
        es_normal = es_normals[i] / np.linalg.norm(es_normals[i])
        # Compute dot product and then angle
        dot_product = np.dot(gt_normal, es_normal)
        dot_product = np.clip(
            dot_product, -1.0, 1.0
        )  # should be unnecessary but just in case

        # Compute the angle in radians
        angle = np.arccos(dot_product)
        if angle > np.pi / 2:
            total_flipped += 1
        differences.append(angle)

    print("Total flipped normals: {}".format(total_flipped))
    print(
        "Percentage of flipped normals: {:.2f}%".format(
            total_flipped / len(groundTruth.points) * 100
        )
    )

    return differences, total_flipped, total_flipped / len(groundTruth.points) * 100
