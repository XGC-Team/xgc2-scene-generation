"""Bounded backend-only polygon-envelope samples of the Euclidean ring surface.

The centerline chord error is at most min(.5 mm, 1% of tube, 1% of hole).
This is not a bound of .5 mm on the entire sampled surface. Radial inner
halfplanes avoid internal sheets even beyond the ellipse curvature radius.
"""
import math


def surface_points(radii, radius):
    a, b = radii
    hole = min(a, b) - radius
    if not all(math.isfinite(x) and x > 0 for x in (a, b, radius)) or hole <= 0:
        raise ValueError('Ring requires positive radii and tube radius smaller than both radii')
    tolerance = min(.0005, .01*radius, .01*hole)
    n = max(32, 4*math.ceil(math.ceil(math.pi*math.sqrt(max(a, b)/(2*tolerance)))/4))
    m = max(16, 4*math.ceil(math.ceil(math.pi*math.sqrt(radius/(2*tolerance)))/4))
    if n > 8192 or m > 2048 or n*m > 4000000:
        raise ValueError('Ring surface sampling exceeds backend capacity')
    points = [(a*math.cos(2*math.pi*i/n), b*math.sin(2*math.pi*i/n)) for i in range(n)]
    edges = []
    for i, (x, y) in enumerate(points):
        qx, qy = points[(i+1) % n]
        ex, ey = qx-x, qy-y
        length = math.hypot(ex, ey)
        nx, ny = ey/length, -ex/length
        edges.append((x, y, ex, ey, nx, ny, nx*x+ny*y))
    result = []
    for x, y in points:
        base = math.hypot(x, y)
        dx, dy = x/base, y/base
        for j in range(m):
            v = 2*math.pi*j/m
            cosine = math.cos(v)
            s = radius*abs(cosine)
            rho = base
            if j not in (m//4, 3*m//4):
                if cosine < 0:
                    rho = min((support-s)/(nx*dx+ny*dy)
                              for _, _, _, _, nx, ny, support in edges if nx*dx+ny*dy > 0)
                else:
                    for px, py, ex, ey, nx, ny, support in edges:
                        discriminant = s*s-(px*dy-py*dx)**2
                        if discriminant >= 0:
                            rho = max(rho, px*dx+py*dy+math.sqrt(discriminant))
                        dot = nx*dx+ny*dy
                        if dot > 0:
                            candidate = (support+s)/dot
                            t = ((candidate*dx-px)*ex+(candidate*dy-py)*ey)/(ex*ex+ey*ey)
                            if 0 <= t <= 1:
                                rho = max(rho, candidate)
            result.append((rho*dx, rho*dy, radius*math.sin(v)))
    return result
