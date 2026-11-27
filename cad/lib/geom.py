"""Small geometry helpers: axis-aligned boxes and cylinders (in world coordinates)."""
import build123d as bd


def box(x0, x1, y0, y1, z0, z1):
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(x1 - x0, y1 - y0, z1 - z0)


def ycyl(r, y0, y1, x=0.0, z=0.0):
    """Cylinder with its axis along Y."""
    return bd.Pos(x, (y0 + y1) / 2, z) * bd.Cylinder(r, y1 - y0, rotation=(90, 0, 0))


def zcyl(r, z0, z1, x=0.0, y=0.0):
    """Cylinder with its axis along Z."""
    return bd.Pos(x, y, (z0 + z1) / 2) * bd.Cylinder(r, z1 - z0)


def xcyl(r, x0, x1, y=0.0, z=0.0):
    return bd.Pos((x0 + x1) / 2, y, z) * bd.Cylinder(r, x1 - x0, rotation=(0, 90, 0))


def stadium_y(half_w, z0, z1, y0, y1, x=0.0):
    """Vertical stadium (oval) profile in the XZ plane, extruded y0..y1 along Y."""
    r = half_w
    if z1 - z0 <= 2 * r:
        return ycyl(r, y0, y1, x=x, z=(z0 + z1) / 2)
    return (box(x - r, x + r, y0, y1, z0 + r, z1 - r)
            + ycyl(r, y0, y1, x=x, z=z0 + r) + ycyl(r, y0, y1, x=x, z=z1 - r))


def fillet_edges(shape, edges, r):
    """Edge fillet; raises if OCCT fails (does not silently skip)."""
    return shape.fillet(r, edges)
