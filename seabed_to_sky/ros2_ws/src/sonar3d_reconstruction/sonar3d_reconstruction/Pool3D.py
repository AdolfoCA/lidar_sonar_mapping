import numpy as np

def get_pool_structure():
    stone_to_wall = -0.045 # meters
    pole_to_water = -0.28
    bottom_z = -3 + pole_to_water
    stone_width = 0.288
    pool_width = 3.98
    inner_pool_length = 6.99 - stone_width + stone_to_wall

    ## CYLINDER 1
    cylinder_position = np.array([2.054, 1.85, bottom_z])

    # Define cylinder parameters
    radius = 0.1
    height = 3
    num_points = 50
    theta = np.linspace(0, 2 * np.pi, num_points)
    z = np.linspace(0, height, num_points)
    theta, z = np.meshgrid(theta, z)

    # Convert cylindrical coordinates to Cartesian coordinates
    x_cylinder = radius * np.cos(theta) + cylinder_position[0]
    y_cylinder = radius * np.sin(theta) + cylinder_position[1]
    z_cylinder = z + cylinder_position[2]

    ## CYLINDER 2
    cylinder2_position = np.array([pool_width - 2*(stone_width) - 0.77, 3.78, bottom_z])

    # Convert cylindrical coordinates to Cartesian coordinates
    x_cylinder2 = radius * np.cos(theta) + cylinder2_position[0]
    y_cylinder2 = radius * np.sin(theta) + cylinder2_position[1]
    z_cylinder2 = z + cylinder2_position[2]

    # Left wall
    left_upper_point = np.array([stone_to_wall, stone_to_wall, 0.0])
    left_lower_point = np.array([stone_to_wall, stone_to_wall + inner_pool_length, bottom_z])
    left_wall = np.vstack((left_upper_point, left_lower_point))

    # Right wall
    origo_to_inner_pool_right_side = (pool_width-(2*stone_width)-stone_to_wall)
    right_upper_point = np.array([origo_to_inner_pool_right_side, stone_to_wall, 0.0])
    right_lower_point = np.array([origo_to_inner_pool_right_side, stone_to_wall + inner_pool_length, bottom_z])
    right_wall = np.vstack((right_upper_point, right_lower_point))

    # Floor
    floor = np.array([
        [stone_to_wall, stone_to_wall, bottom_z],
        [stone_to_wall, stone_to_wall + inner_pool_length, bottom_z],
        [origo_to_inner_pool_right_side, stone_to_wall + inner_pool_length, bottom_z],
        [origo_to_inner_pool_right_side, stone_to_wall, bottom_z]
    ])

    end_wall = np.array([
        [stone_to_wall, stone_to_wall + inner_pool_length , 0.0],
        [stone_to_wall, stone_to_wall + inner_pool_length, bottom_z],
        [origo_to_inner_pool_right_side, stone_to_wall + inner_pool_length, bottom_z],
        [origo_to_inner_pool_right_side, stone_to_wall + inner_pool_length, 0.0]
    ])
    home_wall = np.array([
        [stone_to_wall, stone_to_wall, 0.0],
        [stone_to_wall, stone_to_wall, bottom_z],
        [origo_to_inner_pool_right_side, stone_to_wall, bottom_z],
        [origo_to_inner_pool_right_side, stone_to_wall, 0.0]
    ])

    # Define the vertices for the walls
    left_wall_vertices = [
        [left_upper_point, [left_upper_point[0], left_upper_point[1], left_lower_point[2]], left_lower_point, [left_lower_point[0], left_lower_point[1], left_upper_point[2]]]
    ]

    right_wall_vertices = [
        [right_upper_point, [right_upper_point[0], right_upper_point[1], right_lower_point[2]], right_lower_point, [right_lower_point[0], right_lower_point[1], right_upper_point[2]]]
    ]

    # Define the vertices for the floor
    floor_vertices = [
        [floor[0], floor[1], floor[2], floor[3]]
    ]

    end_wall_vertices = [
        [end_wall[0], end_wall[1], end_wall[2], end_wall[3]]
    ]
    
    home_wall_vertices = [
        [home_wall[0], home_wall[1], home_wall[2], home_wall[3]]
    ]

    return {
        "cylinder1": [x_cylinder, y_cylinder, z_cylinder],
        "cylinder2": [x_cylinder2, y_cylinder2, z_cylinder2],
        "left_wall": left_wall,
        "right_wall": right_wall,
        "floor": floor,
        "end_wall": end_wall,
        "home_wall": home_wall,
    }, {"left_wall": left_wall_vertices, 
        "right_wall": right_wall_vertices, 
        "floor": floor_vertices, 
        "end_wall": end_wall_vertices,
        "home_wall": home_wall_vertices}

def generate_grid_based_point_cloud(environment, grid_spacing=0.01):
    """Point cloud generation code adapted with the help of ChatGPT by OpenAI

    Args:
        environment (dict): Environment definition
        grid_spacing (float): Grid spacing for sampling points from surfaces. Given in meters.
    Returns:
        np.ndarray: Point cloud of shape (3, N) where N is the number of points in the point cloud
    
    """
    point_cloud = []

    # Unpack the environment
    cylinder1 = environment["cylinder1"]
    cylinder2 = environment["cylinder2"]
    left_wall = environment["left_wall"]
    right_wall = environment["right_wall"]
    floor = environment["floor"]
    end_wall = environment["end_wall"]
    home_wall = environment["home_wall"]

    # Sample points from the cylindrical surfaces using a grid
    def sample_cylinder_grid(x, y, z, grid_spacing):
        sampled_points = []
        for i in range(x.shape[0]):
            for j in range(x.shape[1]):
                if i % int(1 / grid_spacing) == 0 and j % int(1 / grid_spacing) == 0:
                    sampled_points.append([x[i, j], y[i, j], z[i, j]])
        return np.array(sampled_points)

    point_cloud.append(sample_cylinder_grid(cylinder1[0], cylinder1[1], cylinder1[2], grid_spacing))
    point_cloud.append(sample_cylinder_grid(cylinder2[0], cylinder2[1], cylinder2[2], grid_spacing))

    # Sample points uniformly from a quad defined by 4 vertices using a grid
    def sample_quad_grid(quad, grid_spacing):
        v0, v1, v2, v3 = [np.array(v) for v in quad]
        # Determine grid size based on surface area and grid spacing
        width_vec = v1 - v0
        height_vec = v3 - v0
        width = np.linalg.norm(width_vec)
        height = np.linalg.norm(height_vec)

        num_x = int(width / grid_spacing)
        num_y = int(height / grid_spacing)

        samples = []
        for i in range(num_x + 1):
            for j in range(num_y + 1):
                a = i / num_x
                b = j / num_y
                sample = (1 - a) * (1 - b) * v0 + a * (1 - b) * v1 + a * b * v2 + (1 - a) * b * v3
                samples.append(sample)
        return np.array(samples)

    # Define quads for the walls and floor
    left_wall_quad = [left_wall[0], [left_wall[0][0], left_wall[0][1], left_wall[1][2]], left_wall[1], [left_wall[1][0], left_wall[1][1], left_wall[0][2]]]
    right_wall_quad = [right_wall[0], [right_wall[0][0], right_wall[0][1], right_wall[1][2]], right_wall[1], [right_wall[1][0], right_wall[1][1], right_wall[0][2]]]
    floor_quad = floor
    end_wall_quad = end_wall
    home_wall_quad = home_wall

    # Sample points for each wall and floor
    point_cloud.append(sample_quad_grid(left_wall_quad, grid_spacing))
    point_cloud.append(sample_quad_grid(right_wall_quad, grid_spacing))
    point_cloud.append(sample_quad_grid(floor_quad, grid_spacing))
    point_cloud.append(sample_quad_grid(end_wall_quad, grid_spacing))
    point_cloud.append(sample_quad_grid(home_wall_quad, grid_spacing))

    # Combine all sampled points into one point cloud array
    point_cloud = np.vstack(point_cloud)

    return point_cloud.T

def point_to_plane_distance(point, plane_point, normal):
    """Compute the signed distance from a point to a plane."""
    return np.abs(np.dot(point - plane_point, normal) / np.linalg.norm(normal))

def point_to_cylinder_distance(point, axis_point, axis_dir, radius):
    """Compute the shortest distance from a point to a cylindrical surface."""
    w = point - axis_point
    proj = w - np.dot(w, axis_dir) * axis_dir
    radial_distance = np.linalg.norm(proj)
    return np.abs(radial_distance - radius)

def compute_point_error(point, environment):
    errors = []
    pole_to_water = -0.28
    bottom_z = -3 + pole_to_water
    stone_width = 0.288
    pool_width = 3.98


    # Define planes (point on plane and normal vector)
    planes = [
        (environment["left_wall"][0], np.array([1, 0, 0])),  # Left wall
        (environment["right_wall"][0], np.array([-1, 0, 0])), # Right wall
        (environment["floor"][0], np.array([0, 0, 1])),       # Floor
        (environment["end_wall"][0], np.array([0, -1, 0])),   # End wall
        (environment["home_wall"][0], np.array([0, 1, 0]))    # Home wall
    ]

    # Compute distances to planes
    for plane_point, normal in planes:
        errors.append(point_to_plane_distance(point, plane_point, normal))

    # Define cylinders (axis point, direction vector, radius)
    cylinders = [
        (np.array([2.054, 1.85, bottom_z]), np.array([0, 0, 1]), 0.1), # Cylinder 1
        (np.array(np.array([pool_width - 2*(stone_width) - 0.77, 3.78, bottom_z])), np.array([0, 0, 1]), 0.1)    # Cylinder 2
    ]

    # Compute distances to cylinders
    for axis_point, axis_dir, radius in cylinders:
        errors.append(point_to_cylinder_distance(point, axis_point, axis_dir, radius))

    # Return the minimum error
    return min(errors)

def compute_point_cloud_error(point_cloud, environment):
    """Compute the error for a point cloud in the
    environment.

    Args:
        point_cloud (np.ndarray): Point cloud of shape (3, N)
        environment (dict): Environment definition
    Returns:
    """
    errors = []
    for point in point_cloud.T:
        errors.append(compute_point_error(point, environment))
    return errors
