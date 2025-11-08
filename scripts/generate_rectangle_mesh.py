import argparse


def generate_triangulated_obj(width, height, resolution_x, resolution_y):
    """
    Generate a high resolution flat rectangular mesh (triangulated) in OBJ format.

    Parameters:
        width (float): The width of the rectangle.
        height (float): The height of the rectangle.
        resolution_x (int): The number of subdivisions along the width.
        resolution_y (int): The number of subdivisions along the height.

    Returns:
        str: OBJ formatted mesh data.
    """

    vertices = []
    faces = []

    # Generate vertices for the mesh
    for y in range(resolution_y + 1):
        for x in range(resolution_x + 1):
            vertices.append((x * width / resolution_x, y * height / resolution_y, 0))

    # Generate triangulated faces for the mesh (using two triangles for each quad)
    for y in range(resolution_y):
        for x in range(resolution_x):
            # Define the four vertices of the quad
            v1 = y * (resolution_x + 1) + x + 1
            v2 = v1 + 1
            v3 = v1 + (resolution_x + 1)
            v4 = v3 + 1

            # First triangle (v1, v2, v3)
            faces.append((v1, v2, v3))

            # Second triangle (v2, v4, v3)
            faces.append((v2, v4, v3))

    # Prepare OBJ format data
    obj_data = []

    # Adding vertices to OBJ
    for v in vertices:
        obj_data.append(f"v {v[0]} {v[1]} {v[2]}")

    # Adding faces to OBJ
    for f in faces:
        obj_data.append(f"f {f[0]} {f[1]} {f[2]}")

    return "\n".join(obj_data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Converts meshes to point clouds.")
    parser.add_argument("-o", "--output", required=True, help="output obj file name")
    parser.add_argument(
        "-l",
        "--length",
        help="length of the rectangle",
        default=2.0,
        type=float,
    )
    parser.add_argument(
        "-w",
        "--width",
        help="width of the rectangle",
        default=1.0,
        type=float,
    )
    parser.add_argument(
        "-r",
        "--resolution",
        help="resolution of the rectangle",
        default=10,
        type=int,
    )
    args = parser.parse_args()
    outputFilename = args.output
    length = args.length
    width = args.width
    resolution = args.resolution
    obj_output = generate_triangulated_obj(width, length, resolution, resolution)
    with open(outputFilename, "w") as file:
        file.write(obj_output)
    print(f"Equilateral triangular mesh OBJ file generated: {outputFilename}")
