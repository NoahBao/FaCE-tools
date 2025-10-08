import random


def listToObj(points: list, outputFilename: str) -> None:
    """
    Takes in a Python list and writes it to a .obj file.
    """
    with open(outputFilename, "w") as f:
        f.write("# OBJ file\n")
        for vertex in points:
            f.write(f"v {' '.join(map(str, vertex))}\n")
    print(f"Data written to {outputFilename}")


def randomPointSample(points: list, rate: float) -> list:
    """
    Takes in a Python list and returns a random sample of it at the given rate.
    """
    if rate <= 0 or rate > 1:
        raise ValueError("Rate must be between 0 (exclusive) and 1 (inclusive).")
    sampleSize = int(len(points) * rate)
    sampledPoints = random.sample(points, sampleSize)
    return sampledPoints
