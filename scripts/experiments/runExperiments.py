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
import subprocess
import shlex
import setup

# all formats should have the order of:
# 1. executable/script
# 2. input file
# 3. output file
commandFormatsClosed: dict[str, str] = {
    "WNNC": "python main_wnnc.py %s --out_dir %s --width_config l0 --tqdm",
    "DWG": "./DWG_CUDA --in_path %s --in_name %s --out_path %s",
    "FaCE": "./face %s%s --h --o %s --i"
}

commandFormatsOpen: dict[str, str] = {
    "DACPO": "./DACPO --input_data_root %s --in %s --out %s --ipsr_type 2 --config_folder conf/default/",
    "SNO": "./StreamingNormalOrientation %s%s --out %s -prepare -estimateNormals -r 0.05 -stream"
}

def runMethodsOnFile(commandFormats: dict[str, str], inputDir: str, inputFile: str, outputDir: str) -> None:
    for method in commandFormats.keys():
        cmd = commandFormats[method] % (inputDir, inputFile, outputDir)
        cmdArgs = shlex.split(cmd)
        proc = subprocess.run(cmdArgs)

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
    args = parser.parse_args()
    inputRoot = args.input_root
    outputRoot = args.output_root
    inputDirs = setup.setup(inputRoot)
    outputDirs = setup.setup(outputRoot)
    for (iDir, oDir) in zip(inputDirs, outputDirs):
        isClosed = "closed" in iDir.split("/")
        isOpen = "open" in iDir.split("/")
        methods: dict[str, str] = {}
        if isClosed:
            methods = commandFormatsClosed
            print("Running closed surface methods for all inputs in:")
        elif isOpen:
            methods = commandFormatsOpen
            print("Running open surface methods for all inputs in:")
        else:
            print("Warning: directory", iDir, "is not labeled as closed or open. Skipping...")
            continue
        print(iDir)

        for input in os.listdir(iDir):
            print("Using input:", input)
            runMethodsOnFile(methods, iDir, input, oDir)