# create all necessary output folders
# loop through all input files
# format input into terminal command to run methods
#   ours, FaCE25, WNNC, SNO, DACPO, DWG
# run each method
#   for closed: WNNC, DWG, FaCE25
#   for open: DACPO, SNO
# save output to correct folder

import argparse
import os
import json
import subprocess
import shlex
import setup
import stats

# All formats should have the order of:
# 1. executable/script
# 2. input directory
# 3. input file
# 4. output file path (dir + filename)
# Executable/script path should be specified as absolute path in config.
commandFormatsClosed: dict[str, str] = {
    "WNNC": "python %s %s%s --out_dir %s --width_config l0 --tqdm",
    "DWG": "%s --in_path %s --in_name %s --out_path %s",
    "FaCE": "%s %s%s --h --o %s --i",
}

commandFormatsOpen: dict[str, str] = {
    "DACPO": "%s --input_data_root %s --in %s --out %s --ipsr_type 2 --config_folder conf/default/",
    "SNO": "%s %s%s --out %s -prepare -estimateNormals -r 0.05 -stream",
}


def runMethodsOnFile(
    inputDir: str,
    inputFile: str,
    outputDir: str,
    confFile: str,
    isClosed: bool,
) -> None:
    methodsConf = json.load(open(confFile))["methods"]
    for method in methodsConf.keys():
        if methodsConf[method]["skip"] or methodsConf[method]["isClosed"] != isClosed:
            continue
        exec = methodsConf[method]["path"]
        cmdFormat = methodsConf[method]["command_format"]
        outputFile = (
            os.path.splitext(inputFile)[0]
            + methodsConf[method]["output_suffix"]
            + ".ply"
        )
        outputArg = outputDir + outputFile if method == "FaCE" else outputDir

        cmd = cmdFormat % ('"' + exec + '"', inputDir, inputFile, outputArg)
        print("\tRunning command: ", cmd)
        cmdArgs = shlex.split(cmd)
        proc = subprocess.run(cmdArgs)

        inputFilePath = inputDir + inputFile
        outputFilePath = outputDir + outputFile
        # diff, flippedCount, flippedPercent = stats.compareNormalsGtEst(
        #     inputFilePath, outputFilePath
        # )
        # angleDiffs[method] += [diff]
        # flippedCounts[method] += [flippedCount]


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
        help="root directory for all output orientation files",
    )
    parser.add_argument(
        "-c",
        "--config",
        required=True,
        help="path to the configuration file",
    )
    args = parser.parse_args()
    inputRoot = args.input_root
    outputRoot = args.output_root
    configFile = args.config
    inputDirs = setup.setup(inputRoot)
    outputDirs = setup.setup(outputRoot)

    angleDiffs: dict[str, list[list[float]]] = {}
    flippedCounts: dict[str, list[float]] = {}
    for iDir, oDir in zip(inputDirs, outputDirs):
        iDirRelative = os.path.relpath(iDir, inputRoot)  # for cleaner print statements
        isClosed = "closed" in iDir.split("/")
        isOpen = "open" in iDir.split("/")

        if isClosed and isOpen:
            print(
                "Warning: directory",
                iDirRelative,
                "is labeled as both closed and open. Skipping...",
            )
            continue
        elif not isClosed and not isOpen:
            print(
                "Warning: directory",
                iDirRelative,
                "is labeled as neither closed nor open. Skipping...",
            )
            continue
        elif isClosed:
            print("Running closed surface methods for all inputs in:", iDirRelative)
        elif isOpen:
            print("Running open surface methods for all inputs in:", iDirRelative)

        for input in os.listdir(iDir):
            if os.path.splitext(input)[1] != ".ply":
                continue
            print("\tUsing input:", input)
            runMethodsOnFile(iDir, input, oDir, configFile, isClosed)
