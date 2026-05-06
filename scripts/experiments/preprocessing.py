import open3d as o3d
import numpy as np
import argparse
import os
import subprocess
import shlex

def addGaussianNoise(pc: o3d.geometry.PointCloud, alpha: float = 0) -> o3d.geometry.PointCloud:
    """
    Add Gaussian noise to every point in `pc`. Distribution is scaled by the size of the point cloud's bounding box and by `alpha`.
    """
    # get bounding box diagonal length
    boundingBox = pc.get_axis_aligned_bounding_box()
    corners = boundingBox.get_box_points()
    cArray = np.asarray(corners)
    diag = cArray[0] - cArray[4]
    lenDiag = np.sqrt(np.dot(diag, diag))
    print("bounding box:", cArray)
    print("len diag=", lenDiag)

    # show point cloud before adding noise
    o3d.visualization.draw_geometries([pc], point_show_normal=False)
    # add noise to each point using Gaussian distribution scaled by diagonal length and alpha
    for p in pc.points:
        for d in range(3):
            p[d] += np.random.normal(0, alpha * lenDiag)
    # show noisy point cloud
    o3d.visualization.draw_geometries([pc], point_show_normal=False)
    return pc

def addWhiteNoise(pc: o3d.geometry.PointCloud, numPoints: int) -> o3d.geometry.PointCloud:
    # getting bounding box minimums and maximums
    boundingBox = pc.get_axis_aligned_bounding_box()
    corners = boundingBox.get_box_points()
    cArray = np.asarray(corners)

    # generate white noise points within bounding box
    whiteNoise = np.ndarray((3, numPoints))
    for d in range(3):
        low = min(cArray[0][d], cArray[4][d])
        high = max(cArray[0][d], cArray[4][d])
        whiteNoise[d] = np.random.uniform(low, high, numPoints)
    whiteNoise = whiteNoise.transpose()

    # add white noise points to point cloud and show the result
    allPoints = np.vstack((np.asarray(pc.points), whiteNoise))
    pc.points = o3d.utility.Vector3dVector(allPoints)
    o3d.visualization.draw_geometries([pc], point_show_normal=False)
    return pc

def processDirectoryFiles(dirPath: str, alpha: float, numWhiteNoise: int) -> None:
    for file in os.listdir(dirPath):
        pc = o3d.io.read_point_cloud(file)
        noisyPC = o3d.geometry.PointCloud(pc)
        if alpha > 0:
            noisyPC = addGaussianNoise(noisyPC, alpha)
        if numWhiteNoise > 0:
            noisyPC = addWhiteNoise(pc, numWhiteNoise)
        o3d.io.write_point_cloud(file[:-4] + "_noisy.ply", noisyPC) # [:-4] to remove ".ply" at end of original file name

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Runs all point cloud orientation methods on all provided inputs."
    )
    parser.add_argument(
        "-i",
        "--input_root",
        required=True,
        help="root directory for all input point cloud files",
    )
    parser.add_argument(
        "-o",
        "--output_root",
        required=True,
        help="root directory for all processed point cloud files",
    )
    parser.add_argument(
        "-a",
        "--alpha",
        help="alpha scale for noise (set as 0 to disable)",
        default=0,
        type=float
    )
    parser.add_argument(
        "-w",
        "--white_noise",
        help="num white noise points to add (set as 0 to disable)",
        default=0,
        type=int
    )
    args = parser.parse_args()
    inputDir = args.input_root
    outputDir = args.output_root
    alpha = args.alpha
    numWhiteNoise = args.white_noise

    os.chdir(inputDir)
    copyCmd = "find . -type d -not -path \"*/.*\" -exec mkdir -p %s{} \;" % (outputDir) # command to recursively copy all directories in input root to output root
    cmdArgs = shlex.split(copyCmd, posix=False)
    cmdArgs[6] = "\"*/.*\""
    proc = subprocess.run(cmdArgs)
    # proc = subprocess.run([
    #     "find", ".", "-type", "d",
    #     "-not", "-path", "\"*/.*\"",
    #     "-exec", "mkdir", "-p", f"{outputDir}/{{}}", ";"
    # ])
    print(proc)