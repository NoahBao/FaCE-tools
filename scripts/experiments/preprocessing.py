import open3d as o3d
import numpy as np
import argparse
import os
import fileSystem as fs


def addGaussianNoise(
    pc: o3d.geometry.PointCloud, alpha: float = 0
) -> o3d.geometry.PointCloud:
    """
    Add Gaussian noise to every point in `pc`. Distribution is scaled by the size of the point cloud's bounding box and by `alpha`.
    """
    # get bounding box diagonal length
    boundingBox = pc.get_axis_aligned_bounding_box()
    corners = boundingBox.get_box_points()
    cArray = np.asarray(corners)
    diag = cArray[0] - cArray[4]
    lenDiag = np.sqrt(np.dot(diag, diag))
    # print("bounding box:", cArray)
    # print("len diag=", lenDiag)

    # add noise to each point using Gaussian distribution scaled by diagonal length and alpha
    for p in pc.points:
        for d in range(3):
            p[d] += np.random.normal(0, alpha * lenDiag)
    return pc


def addWhiteNoise(
    pc: o3d.geometry.PointCloud, numPoints: int
) -> o3d.geometry.PointCloud:
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
    return pc


def processDirectoryFiles(
    inputRoot: str, outputRoot: str, alpha: float, numWhiteNoise: int, vis: bool
) -> None:
    """
    Goes through all directories (recursively) in `inputRoot` and processes all .ply files by adding noise to them, then saves the processed point clouds in the same directory structure in `outputRoot`. Noise is added according to `alpha` and `numWhiteNoise` parameters. If `vis` is true, shows the point clouds before and after processing.
    """
    print("Processing files in directory ", inputRoot)
    print("Beginning processing...")
    for root, dirs, files in os.walk(inputRoot):
        for d in dirs:
            src_path = os.path.join(root, d)
            rel_path = os.path.relpath(src_path, inputRoot)
            dest_path = os.path.join(outputRoot, rel_path)

            os.makedirs(dest_path, exist_ok=True)
        for file in files:
            # check extension is .ply
            # splittext returns a tuple of (filename without extension, extension)
            if os.path.splitext(file)[1] == ".ply":
                inputFilePath = os.path.join(root, file)
                relPath = os.path.relpath(inputFilePath, inputRoot)
                print("Processing file: ", relPath)
                pc = o3d.io.read_point_cloud(inputFilePath)
                # show point cloud before adding noise
                if vis:
                    o3d.visualization.draw_geometries([pc], point_show_normal=False)

                noisyPC = o3d.geometry.PointCloud(pc)
                if alpha > 0:
                    noisyPC = addGaussianNoise(noisyPC, alpha)
                if numWhiteNoise > 0:
                    noisyPC = addWhiteNoise(noisyPC, numWhiteNoise)
                # show noisy point cloud
                if vis:
                    o3d.visualization.draw_geometries(
                        [noisyPC], point_show_normal=False
                    )

                relPathWithoutExt = os.path.splitext(relPath)[0]
                outputFilePath = os.path.join(
                    outputRoot, relPathWithoutExt + "_noisy.ply"
                )
                o3d.io.write_point_cloud(outputFilePath, noisyPC)
    print("Finished processing files.")


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
        type=float,
    )
    parser.add_argument(
        "-w",
        "--white_noise",
        help="num white noise points to add (set as 0 to disable)",
        default=0,
        type=int,
    )
    parser.add_argument(
        "-v",
        "--vis",
        help="whether to visualize point clouds before and after processing",
        default=False,
        type=bool,
    )
    args = parser.parse_args()
    inputDir = args.input_root
    outputDir = args.output_root
    alpha = args.alpha
    numWhiteNoise = args.white_noise
    vis = args.vis

    processDirectoryFiles(
        os.path.abspath(inputDir), os.path.abspath(outputDir), alpha, numWhiteNoise, vis
    )
