import os
from textwrap import indent

DIRECTORY_TREE_ROOT: str = """closed
open
preprocessed"""

DIRECTORY_TREE_CLOSED: str = """GCNO
ModelNet
Thingi10k
RibbonBrush
    original
    sparsified
synthetic_scans"""

DIRECTORY_TREE_OPEN: str = """SceneNN
ScanNet
faces"""

DIRECTORY_TREE_PREPROCESSED: str = """closed
open
noisy
    alpha_0.0025
    alpha_0.0050
    alpha_0.0100
extra_500
extra_1000"""

# KEY NAMES HAVE TO EXACTLY MATCH THE DIRECTORY NAMES USED IN TREES

DIRECTORY_TREES = {
    "root": DIRECTORY_TREE_ROOT,
    "closed": DIRECTORY_TREE_CLOSED,
    "open": DIRECTORY_TREE_OPEN,
    "preprocessed": DIRECTORY_TREE_PREPROCESSED,
}


def createDirectories(root: str, directoryTree: str) -> list[str]:
    """
    Creates the input directory tree in the specified root folder. Returns a list of paths to all leaf node folders.
    """
    prevRoot = os.path.abspath(os.curdir)
    os.chdir(root)

    directories = directoryTree.split("\n")
    curIndent = 0
    leafNodeFolders: list[str] = []  # paths for all leaf nodes in directory tree
    pathStack: list[str] = (
        []
    )  # stack for which folders we're in (essentially a live-updated working path)

    for i, folder in enumerate(directories):
        folderName = folder.strip()
        indent: int = (len(folder) - len(folderName)) // 4  # 4 spaces per indent

        if i > 0 and indent <= curIndent:
            # if we're on the same level or a lower level than before,
            # then the previous folder is a leaf node in the directory tree
            prevFolderName = directories[i - 1].strip()
            leafNodeFolders.append(pathStackToPath(pathStack + [prevFolderName], root))

        while indent < curIndent:
            # keep on exiting directories until curIndent matches new folder's indent
            os.chdir("..")
            pathStack.pop()
            curIndent -= 1
        if i == len(directories) - 1:
            # the last line is also always a leaf node
            leafNodeFolders.append(pathStackToPath(pathStack + [folderName], root))

        if indent > curIndent + 1:
            # can't go up multiple indents at once
            raise ValueError(
                "Invalid indentation for line %d in directories string: has indent level %d, but previous line was indent level %d."
                % (i, indent, curIndent)
            )
        elif indent == curIndent + 1:
            # if indent level goes up, it means we entered the previous directory
            prevDir = directories[i - 1].strip()
            os.chdir(prevDir)
            pathStack.append(prevDir)
            curIndent += 1

        try:
            os.mkdir(folderName)
        except:
            print(
                "Warning: folder ",
                pathStackToPath(pathStack + [folderName], root),
                "already exists; skipping.",
            )
    os.chdir(prevRoot)
    print("Returning to previous working directory: ", prevRoot)
    return leafNodeFolders


def copyDirectories(srcRoot: str, destRoot: str) -> None:
    """
    Copies the directory structure from the source to the destination without copying files.
    """
    for root, dirs, _ in os.walk(srcRoot):
        for d in dirs:
            src_path = os.path.join(root, d)
            rel_path = os.path.relpath(src_path, srcRoot)
            dest_path = os.path.join(destRoot, rel_path)

            os.makedirs(dest_path, exist_ok=True)


def pathStackToPath(pathStack: list[str], root: str = "/") -> str:
    """
    Creates a path based on the stack provided. The `root` path is prepended to the final path if provided.
    """
    path = root
    for dir in pathStack:
        path += dir + "/"
    return path


def printDirectoryTrees() -> None:
    for k in DIRECTORY_TREES.keys():
        print("=====", k.upper(), "DIRECTORY TREE =====")
        print(DIRECTORY_TREES[k])
        print()


# Source - https://stackoverflow.com/a/9728478
# Posted by dhobbs, modified by community. See post 'Timeline' for change history
# Retrieved 2026-05-02, License - CC BY-SA 3.0
def printExistingTree(path: str):
    for root, dirs, files in os.walk(path):
        level = root.replace(path, "").count(os.sep)
        indent = " " * 4 * (level)
        print("{}{}/".format(indent, os.path.basename(root)))
        subindent = " " * 4 * (level + 1)
        for f in files:
            print("{}{}".format(subindent, f))
