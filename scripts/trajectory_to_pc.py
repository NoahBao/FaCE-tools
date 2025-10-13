from util import *
import numpy as np
import argparse
import open3d as o3d


def trajectoryStringToMatrix(matrix_string: str) -> np.array:
    """
    Converts a single trajectory matrix in string format to a 4x4 numpy array.
    """
    lines = matrix_string.strip().split("\n")
    matrix = []
    for line in lines[1:5]:  # Skip the first line (which is just a frame index)
        row = list(map(float, line.split()))
        matrix.append(row)
    return np.array(matrix)


def trajectoryLogToMatrices(filename: str) -> list[np.array]:
    """
    Takes in a trajectory filename and returns the contents as a matrix list.
    """
    matrices = []
    with open(filename, "r") as file:
        lines = file.readlines()
        # each matrix has a header line and 4 rows of data
        # so go 5 lines at a time
        for mi in range(0, len(lines), 5):
            matrixList = []
            # go through each row, skipping the first
            for r in range(1, 5):
                line = lines[mi + r]
                # Split the line into individual string numbers
                str_numbers = line.split()
                # Convert string numbers to floats and append to trajectory list
                float_numbers = [float(num) for num in str_numbers]
                matrixList.append(float_numbers)
            matrix = np.array(matrixList)
            matrices.append(matrix)
    return matrices


def trajectoryLogToPointCloud(filename: str, skip: int) -> list[np.array]:
    """
    Takes in a trajectory filename and returns the translation components as a list of points.
    """
    trajectoryMatrices = trajectoryLogToMatrices(filename)
    points = []
    for i in range(0, len(trajectoryMatrices), skip):
        matrix = trajectoryMatrices[i]
        point = matrix[0:3, 3]  # Extract the translation component (last column)
        points.append(point)
    return points


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Converts trajectory log to point cloud."
    )
    parser.add_argument(
        "-i", "--input", required=True, help="input trajectory log file name"
    )
    parser.add_argument(
        "-o", "--output", required=True, help="output OBJ file name for point cloud"
    )
    parser.add_argument(
        "-s",
        "--skip",
        help="how often to sample points from trajectory (e.g. 3 means every 3rd point)",
        default=1,
        type=int,
    )
    args = parser.parse_args()
    inputFilename = args.input
    outputFilename = args.output
    skip = args.skip

    print(f"Reading trajectory log from {inputFilename}...")
    pointCloud = trajectoryLogToPointCloud(inputFilename, skip)
    print(f"Number of points in trajectory: {len(pointCloud)}")

    print(f"Writing point cloud to {outputFilename}...")
    pointCloudToObj(pointCloud, outputFilename)
    print("Done.")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointCloud)
    o3d.visualization.draw_geometries([pcd])
