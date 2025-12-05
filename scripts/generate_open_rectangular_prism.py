import numpy as np


def generate_open_rectangular_prism(width, depth, height, output_file, grid_size):
    """
    Generate a rectangular prism with an open top base and save it as a .obj file.

    :param width: Width of the rectangular prism
    :param depth: Depth of the rectangular prism
    :param height: Height of the rectangular prism
    :param output_file: Output .obj file name
    :param grid_size: Size of the grid for subdividing faces
    """
    vertices = []
    faces = []

    # Generate grid points for each face
    def generate_grid(x_range, y_range, z_range):
        grid_vertices = []
        # if a dimension has no range (i.e., min == max), just use that single value
        x_vals = (
            np.arange(x_range[0], x_range[1], grid_size)
            if x_range[0] != x_range[1]
            else [x_range[0]]
        )
        y_vals = (
            np.arange(y_range[0], y_range[1], grid_size)
            if y_range[0] != y_range[1]
            else [y_range[0]]
        )
        z_vals = (
            np.arange(z_range[0], z_range[1], grid_size)
            if z_range[0] != z_range[1]
            else [z_range[0]]
        )
        for x in x_vals:
            for y in y_vals:
                for z in z_vals:
                    grid_vertices.append((x, y, z))
        return grid_vertices

    # Bottom face
    bottom_vertices = generate_grid((0, width), (0, 0), (0, depth))
    vertices.extend(bottom_vertices)

    # Front face
    front_vertices = generate_grid((0, width), (0, height), (0, 0))
    vertices.extend(front_vertices)

    # Right face
    right_vertices = generate_grid((width, width), (0, height), (0, depth))
    vertices.extend(right_vertices)

    # Back face
    back_vertices = generate_grid((0, width), (0, height), (depth, depth))
    vertices.extend(back_vertices)

    # Left face
    left_vertices = generate_grid((0, 0), (0, height), (0, depth))
    vertices.extend(left_vertices)

    # Write to .obj file
    with open(output_file, "w") as f:
        for v in vertices:
            f.write(f"v {v[0]} {v[1]} {v[2]}\n")


if __name__ == "__main__":
    width = 4.0
    depth = 3.0
    height = 2.0
    grid_size = 0.2
    output_file = "open_rectangular_prism.obj"

    generate_open_rectangular_prism(width, depth, height, output_file, grid_size)
    print(f"Rectangular prism with open top base saved to {output_file}")
