import argparse
import h5py
import open3d as o3d
import numpy as np

from util import *


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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Converts HDF5 files to point clouds.")
    parser.add_argument("-i", "--input", required=True, help="input hdf5 file name")
    parser.add_argument("-o", "--output", required=True, help="output obj file name")
    parser.add_argument("-t", "--trajectory", help="trajectory log file name")
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
    pointcloudList = h5pyToList(inputFilename)
    print(f"Number of point clouds in file: {len(pointcloudList)}")

    doTransform = False
    trajectoryMatrices = []
    # if len(sys.argv) == 4:
    #     trajectoryFilename = sys.argv[3]
    #     print(f"Reading trajectory file {trajectoryFilename}...")
    #     trajectoryMatrices = readTrajectoryFile(trajectoryFilename)
    #     print(f"Number of trajectory matrices in file: {len(trajectoryMatrices)}")
    #     doTransform = True
    # else:
    #     print("No trajectory file provided, skipping transformation step.")

    # Initialize final point cloud list
    finalPointCloud = []
    numCloudsToProcess = (
        min(len(pointcloudList), len(trajectoryMatrices))
        if doTransform
        else len(pointcloudList)
    )

    print(f"Combining {len(pointcloudList)} point clouds...")
    if doTransform:
        print(
            "Transforming point clouds according to camera position before combining..."
        )

    for i in range(numCloudsToProcess):
        # for i in [0]:
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

    # Optional: Randomly sample points to reduce size
    sampleRate = args.rate
    print(f"Randomly sampling {int(sampleRate * 100)}% of points to reduce size...")
    finalPointCloud = randomPointSample(finalPointCloud, sampleRate)
    print(f"Number of points after sampling: {len(finalPointCloud)}")

    # Write final point cloud to OBJ file and display
    listToObj(finalPointCloud, outputFilename)
    pcd = o3d.geometry.PointCloud()
    # 0.732859 0.0771988 -0.679487 -1.01705
    # -0.032094 -1.00156 -0.0467065 1.14307
    # -0.680724 0.0955677 -0.727367 -0.912924
    # 0.000000 0.000000 0.000000 1.000000
    cameraPoint = [-1.01705, 1.14307, -0.912924]
    finalPointCloud.extend([cameraPoint])
    pcd.points = o3d.utility.Vector3dVector(finalPointCloud)
    o3d.visualization.draw_geometries([pcd])
