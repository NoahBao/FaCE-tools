import argparse
from util import *
import open3d as o3d


def extract_vertices_from_mesh(file_path):
    """
    Extracts vertices from a mesh file and returns them as a list of lists.
    """
    # Load the mesh file
    mesh = o3d.io.read_triangle_mesh(file_path)

    # Check if the mesh is successfully loaded
    if not mesh.has_vertices():
        raise ValueError(
            "The mesh file does not contain any vertices or failed to load."
        )

    # Extract vertices as a list of lists
    vertices = [list(vertex) for vertex in mesh.vertices]
    return vertices


# Example usage
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Converts meshes to point clouds.")
    parser.add_argument("-i", "--input", required=True, help="input mesh file name")
    parser.add_argument("-o", "--output", required=True, help="output obj file name")
    parser.add_argument(
        "-r",
        "--rate",
        help="rate at which to randomly sample points",
        default=0.01,
        type=float,
    )
    args = parser.parse_args()
    inputFilename = args.input
    outputFilename = args.output

    # Convert HDF5 to list and then to OBJ
    print(f"Reading {inputFilename} and converting to point cloud list...")
    pointcloud = extract_vertices_from_mesh(inputFilename)
    print(f"Number of point clouds in file: {len(pointcloud)}")

    # Downsample the point cloud
    sample_rate = args.rate
    if sample_rate <= 0 or sample_rate > 1:
        print("Rate must be between 0 (exclusive) and 1 (inclusive).")
        print("Setting rate to default value of 0.01")
        sample_rate = 0.01
    pointcloud = randomPointSample(pointcloud, sample_rate)
    print("Number of sampled vertices:", len(pointcloud))

    pointCloudToObj(pointcloud, outputFilename)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud)
    o3d.visualization.draw_geometries([pcd])
