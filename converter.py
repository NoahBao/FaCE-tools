import h5py
import os
import sys
import open3d
import numpy as np


def h5pyToList(filename: str) -> list:
    """
    Takes in a .hdf5 filename and returns the contents as a Python list.
    """
    # REFERENCE:
    # https://stackoverflow.com/questions/28170623/how-to-read-hdf5-files-in-python
    with h5py.File(filename, "r") as f:
        # Print all root level object names (aka keys)
        # these can be group or dataset names
        # print(f"Keys: {f.keys()}")  # PRINTS "Keys: <KeysViewHDF5 ['coords']>"
        # get first object name/key; may or may NOT be a group
        coords_key = list(f.keys())[0]

        # get the object type for coords_key: usually group or dataset
        # print(type(f[coords_key]))  # PRINTS "<class 'h5py._hl.dataset.Dataset'>"

        # this gets the dataset values and returns as a list
        data = list(f[coords_key])
        # # preferred methods to get dataset values:
        # ds_obj = f[coords_key]  # returns as a h5py dataset object
        # ds_arr = f[coords_key][()]  # returns as a numpy array
        return data


def readTrajectoryFile(filename: str) -> list[np.array]:
    """
    Takes in a trajectory filename and returns the contents as a matrix list.
    """
    matrices = []
    with open(filename, "r") as file:
        lines = file.readlines()
        # each matrix has a header line and 4 rows of data
        # so go by 5 lines at a time
        for fi in range(0, len(lines), 5):
            matrixList = []
            # go through each row
            for r in range(1, 5):
                line = lines[fi + r]
                # Split the line into individual string numbers
                str_numbers = line.split()
                # Convert string numbers to floats and append to trajectory list
                float_numbers = [float(num) for num in str_numbers]
                matrixList.append(float_numbers)
            matrix = np.array(matrixList)
            matrices.append(matrix)
    return matrices


def listToObj(points: list, outputFilename: str) -> None:
    """
    Takes in a Python list and writes it to a .obj file.
    """
    with open(outputFilename, "w") as f:
        for vertex in points:
            f.write(f"v {' '.join(map(str, vertex))}\n")
    print(f"Data written to {outputFilename}")


if __name__ == "__main__":
    # Read and verify command line arguments
    if len(sys.argv) not in [3, 4]:
        print(
            "Usage: python converter.py <hdf5 filename> <output obj filename> [trajectory filename]"
        )
        sys.exit(1)
    inputFilename = sys.argv[1]
    outputFilename = sys.argv[2]

    if not os.path.isfile(inputFilename):
        print(f"File not found: {inputFilename}")
        sys.exit(1)

    # Convert HDF5 to list and then to OBJ
    print(f"Reading {inputFilename} and converting to point cloud list...")
    pointcloudList = h5pyToList(inputFilename)
    print(f"Number of point clouds in file: {len(pointcloudList)}")

    doTransform = False
    trajectoryMatrices = []
    if len(sys.argv) == 4:
        trajectoryFilename = sys.argv[3]
        print(f"Reading trajectory file {trajectoryFilename}...")
        trajectoryMatrices = readTrajectoryFile(trajectoryFilename)
        print(f"Number of trajectory matrices in file: {len(trajectoryMatrices)}")
        doTransform = True
    else:
        print("No trajectory file provided, skipping transformation step.")

    # Initialize final point cloud list
    finalPointCloud = []
    numCloudsToProcess = (
        min(len(pointcloudList), len(trajectoryMatrices))
        if doTransform
        else len(pointcloudList)
    )

    if doTransform:
        print("Transforming point clouds according to camera position and combining...")
    else:
        print("Combining point clouds...")

    for i in range(numCloudsToProcess):
        pointcloud = pointcloudList[i]

        if doTransform:
            # Transform point cloud according to camera position
            for point in pointcloud:
                point_homogeneous = np.append(
                    point, 1
                )  # Convert to homogeneous coordinates
                transformed_point = trajectoryMatrices[i] @ point_homogeneous
                point[:] = transformed_point[
                    :3
                ]  # Update point with transformed coordinates
        # Add transformed points to final point cloud
        finalPointCloud.extend(pointcloud)
        # check whether we passed a 10% mark
        passedMark = ((i + 1) / numCloudsToProcess * 100) % 10 < (
            i / numCloudsToProcess * 100
        ) % 10
        if passedMark:
            percent = int((i + 1) / numCloudsToProcess * 10) * 10
            print(f"Processed {percent}% of point clouds...")
    print(f"Total number of points in final point cloud: {len(finalPointCloud)}")

    listToObj(finalPointCloud, outputFilename)
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(finalPointCloud)
    open3d.visualization.draw_geometries([pcd])
