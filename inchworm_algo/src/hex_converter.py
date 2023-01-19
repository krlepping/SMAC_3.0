def cube_to_evenr(cube):
    q = cube[0]
    r = cube[1]
    col = q + (r + (r&1)) / 2
    row = r
    return [col, row]

def evenr_to_cube(hex_coord):
    q = hex_coord[0] - (hex_coord[1] + (int(hex_coord[1])&1)) / 2
    r = hex_coord[1]
    s = -q-r
    return [q, r, s]
    


def cube_subtract(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]

def cube_distance(a, b):
    vec = cube_subtract(a, b)
    return (abs(vec[0]) + abs(vec[1]) + abs(vec[2])) / 2



cube_direction_vectors = [
    [+1, 0, -1], [+1, -1, 0], [0, -1, +1], 
    [-1, 0, +1], [-1, +1, 0], [0, +1, -1]
]

def cube_direction(direction):
    return cube_direction_vectors[direction]

def cube_add(hex_coord, vec):
    return [hex_coord[0] + vec[0], hex_coord[1] + vec[1], hex_coord[2] + vec[2]]

def cube_neighbor(hex_coord, direction):
    return cube_add(hex_coord, cube_direction(direction))

def get_all_neighbors(coord):
    neighbors = []
    hex_coord = evenr_to_cube(coord)
    for vec in cube_direction_vectors:
        n = cube_to_evenr(cube_neighbor(hex_coord, vec))
        if n[0] < 0 and n[1] < 0:
            neighbors.append(n)
    return neighbors
    
def evenr_distance(x, y):
    x_cube = evenr_to_cube(x)
    y_cube = evenr_to_cube(y)
    return cube_distance(x_cube, y_cube)