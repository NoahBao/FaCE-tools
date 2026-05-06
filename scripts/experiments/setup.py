import argparse
import fileSystem as fs

def setup(root: str) -> list[str]:
    print("Creating input directories with following tree:")
    fs.printDirectoryTrees()

    if not root.endswith("/"):
        root += "/"
    leaves = fs.createDirectories(root, fs.DIRECTORY_TREE_ROOT)
    finalLeaves = []
    while len(leaves) > 0:
        newLeaves = []
        for leaf in leaves:
            finalFolderName = leaf.split("/")[-2]
            subTree = fs.DIRECTORY_TREES.get(finalFolderName)
            if subTree is not None:
                newLeaves += fs.createDirectories(leaf, subTree)
            else:
                finalLeaves.append(leaf)
        leaves = newLeaves
    return finalLeaves


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Creates the input directory tree in the provided root directory."
    )
    parser.add_argument(
        "-r",
        "--root_path",
        required=True,
        help="root directory for all orientation input files",
    )
    args = parser.parse_args()
    root = args.root_path
    leaves = setup(root)

    print("===== DIRECTORY TREE RESULT =====")
    fs.printExistingTree(root)