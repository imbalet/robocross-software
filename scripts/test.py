def coords_to_inds(coords, shape):
    row = int(shape[0] // 2 - coords[1])
    col = int(shape[1] // 2 + coords[0])
    return row, col


def inds_to_coords(inds, shape):
    y = float(shape[0] // 2 - inds[0])
    x = float(-shape[1] // 2 + inds[1])
    return x, y

print()