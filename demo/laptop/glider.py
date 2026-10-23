"""Procedural glider geometry and quaternion-based orientation.

Model coordinates:
    nose: -X
    left wing: -Y
    top: +Z

The default mapping implements the supplied description literally:
    IMU +X -> nose (-model X)
    IMU +Y -> left (-model Y)
    IMU +Z -> bottom (-model Z)

That mapping has determinant -1. It is a coordinate relabeling, not a
physically realizable rotation between two right-handed frames. Verify
the actual sensor axes before presenting.
"""

import math


IDENTITY = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)

DEFAULT_AXES = "-x,-y,-z"
VIEW_AZ_DEG = 20.0
VIEW_EL_DEG = 25.0
BACKGROUND = "#f2f4f8"


def matmul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3))
              for j in range(3))
        for i in range(3)
    )


def transpose(m):
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


def apply(m, p):
    return tuple(sum(m[i][j] * p[j] for j in range(3)) for i in range(3))


def sub(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def unit(v):
    length = math.sqrt(dot(v, v))
    if length < 1e-12:
        return None
    return tuple(x / length for x in v)


def determinant(m):
    return dot(m[0], cross(m[1], m[2]))


def axis_matrix(spec):
    """Rows define model X/Y/Z in terms of signed sensor components."""
    words = [part.strip().lower() for part in spec.split(",")]
    if len(words) != 3:
        raise ValueError("axes must contain three entries, e.g. -x,-y,-z")

    rows = []
    used = set()

    for word in words:
        if len(word) == 1:
            sign, axis = 1.0, word
        elif len(word) == 2 and word[0] in "+-":
            sign = -1.0 if word[0] == "-" else 1.0
            axis = word[1]
        else:
            raise ValueError("each axis must be x, y, z, +x, -x, etc.")

        if axis not in "xyz" or axis in used:
            raise ValueError("use each of x, y, z exactly once")

        used.add(axis)
        index = "xyz".index(axis)
        rows.append(tuple(sign if i == index else 0.0 for i in range(3)))

    return tuple(rows)


def quaternion_matrix(q, inverse=False):
    """Hamilton quaternion, scalar first; default is an active rotation."""
    if len(q) != 4 or not all(math.isfinite(value) for value in q):
        raise ValueError("invalid quaternion")

    length = math.sqrt(sum(value * value for value in q))
    if not 0.5 <= length <= 1.5:
        raise ValueError("invalid quaternion magnitude")

    w, x, y, z = (value / length for value in q)

    result = (
        (1 - 2 * (y * y + z * z),
         2 * (x * y - w * z),
         2 * (x * z + w * y)),
        (2 * (x * y + w * z),
         1 - 2 * (x * x + z * z),
         2 * (y * z - w * x)),
        (2 * (x * z - w * y),
         2 * (y * z + w * x),
         1 - 2 * (x * x + y * y)),
    )

    return transpose(result) if inverse else result


def relative_rotation(reference, current, mapping, inverse=False):
    """Express motion from the captured sensor reference in model axes."""
    r0 = quaternion_matrix(reference, inverse)
    r1 = quaternion_matrix(current, inverse)
    relative = matmul(transpose(r0), r1)
    return matmul(matmul(mapping, relative), transpose(mapping))


def view_matrix(azimuth=VIEW_AZ_DEG, elevation=VIEW_EL_DEG):
    azimuth = math.radians(azimuth)
    elevation = math.radians(elevation)
    ca, sa = math.cos(azimuth), math.sin(azimuth)
    ce, se = math.cos(elevation), math.sin(elevation)

    rz = ((ca, -sa, 0.0), (sa, ca, 0.0), (0.0, 0.0, 1.0))
    rx = ((1.0, 0.0, 0.0), (0.0, ce, -se), (0.0, se, ce))
    return matmul(rx, rz)


def rgb(color):
    return tuple(int(color[i:i + 2], 16) for i in (1, 3, 5))


class Glider:
    """Small opaque mesh, triangulated and drawn using painter ordering."""

    def __init__(self, azimuth=VIEW_AZ_DEG, elevation=VIEW_EL_DEG):
        self.faces = []
        self.view = view_matrix(azimuth, elevation)
        self.light = unit((-0.35, -0.6, 0.7))
        self.shift_x = 0.15
        self._build()

    def _face(self, vertices, color, inside):
        vertices = [
            (x - self.shift_x, y, z) for x, y, z in vertices
        ]
        inside = (
            inside[0] - self.shift_x, inside[1], inside[2]
        )

        # Triangulation reduces, but does not eliminate, painter-sort errors.
        for i in range(1, len(vertices) - 1):
            triangle = (vertices[0], vertices[i], vertices[i + 1])
            center = tuple(sum(p[k] for p in triangle) / 3.0
                           for k in range(3))
            normal = cross(sub(triangle[1], triangle[0]),
                           sub(triangle[2], triangle[0]))
            normal = unit(normal)

            if normal is None:
                continue

            if dot(normal, sub(center, inside)) < 0:
                normal = tuple(-value for value in normal)

            self.faces.append((triangle, center, normal, rgb(color)))

    @staticmethod
    def _ring(x, radius, sides=12):
        return [
            (
                x,
                radius * math.cos(2 * math.pi * k / sides),
                radius * math.sin(2 * math.pi * k / sides),
            )
            for k in range(sides)
        ]

    def _tube(self, rings, color):
        for (x0, r0), (x1, r1) in zip(rings, rings[1:]):
            a, b = self._ring(x0, r0), self._ring(x1, r1)
            inside = ((x0 + x1) / 2, 0.0, 0.0)

            for k in range(len(a)):
                j = (k + 1) % len(a)
                self._face((a[k], a[j], b[j], b[k]), color, inside)

    def _box(self, center, half_size, color):
        cx, cy, cz = center
        dx, dy, dz = half_size
        x0, x1 = cx - dx, cx + dx
        y0, y1 = cy - dy, cy + dy
        z0, z1 = cz - dz, cz + dz

        faces = (
            ((x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)),
            ((x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0)),
            ((x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)),
            ((x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0)),
            ((x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0)),
            ((x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)),
        )

        for face in faces:
            self._face(face, color, center)

    def _wings(self, x0, x1, y0, y1, top, bottom, strips=3):
        thickness = 0.018

        for sign in (-1, 1):
            for i in range(strips):
                ya = sign * (y0 + (y1 - y0) * i / strips)
                yb = sign * (y0 + (y1 - y0) * (i + 1) / strips)
                lo, hi = min(ya, yb), max(ya, yb)
                inside = ((x0 + x1) / 2, (lo + hi) / 2, 0.0)

                self._face(
                    ((x0, lo, thickness), (x1, lo, thickness),
                     (x1, hi, thickness), (x0, hi, thickness)),
                    top, inside,
                )
                self._face(
                    ((x0, lo, -thickness), (x0, hi, -thickness),
                     (x1, hi, -thickness), (x1, lo, -thickness)),
                    bottom, inside,
                )

                for xa in (x0, x1):
                    self._face(
                        ((xa, lo, -thickness), (xa, hi, -thickness),
                         (xa, hi, thickness), (xa, lo, thickness)),
                        bottom, inside,
                    )

                if i == strips - 1:
                    outer = yb
                    self._face(
                        ((x0, outer, -thickness),
                         (x1, outer, -thickness),
                         (x1, outer, thickness),
                         (x0, outer, thickness)),
                        bottom, inside,
                    )

    def _build(self):
        dome = [(-1.0, 0.0)]

        for angle in (75, 55, 35, 15, 0):
            a = math.radians(angle)
            dome.append((-0.75 - 0.25 * math.sin(a),
                         0.25 * math.cos(a)))

        self._tube(dome, "#d01010")
        self._tube([(-0.75, 0.25), (-0.10, 0.25)], "#b058c8")
        self._tube([(-0.10, 0.25), (0.20, 0.25)], "#4a2fbd")
        self._tube(
            [(0.20, 0.25), (0.45, 0.25), (0.80, 0.08),
             (1.20, 0.07), (1.28, 0.11), (1.36, 0.09), (1.40, 0.0)],
            "#6fba2c",
        )

        self._wings(-0.50, -0.12, 0.24, 1.05,
                    "#d4c400", "#14b5b5")
        self._wings(1.12, 1.32, 0.10, 0.62,
                    "#4f9a1a", "#2f6d10", strips=2)

        # Bottoms are above the maximum hull radius of 0.25.
        # No viewer-directed depth bias: underside occlusion stays meaningful.
        self._box((-0.73, 0.0, 0.30),
                  (0.05, 0.05, 0.04), "#d2640a")
        self._box((-0.34, 0.0, 0.31),
                  (0.07, 0.07, 0.05), "#1f78c8")

    def draw(self, canvas, width, height, rotation):
        matrix = matmul(self.view, rotation)
        scale = min(width / 3.7, height / 3.4)
        cx, cy = width / 2, height / 2 + 0.04 * height
        visible = []

        for triangle, center, normal, base in self.faces:
            normal_view = apply(matrix, normal)

            # Viewer looks toward +Y; visible outward normals face -Y.
            if normal_view[1] >= -1e-9:
                continue

            vertices = [apply(matrix, p) for p in triangle]
            depth = apply(matrix, center)[1]
            shade = 0.45 + 0.55 * max(0.0, dot(normal_view, self.light))
            color = "#%02x%02x%02x" % tuple(
                max(0, min(255, int(component * shade)))
                for component in base
            )
            visible.append((depth, vertices, color))

        visible.sort(key=lambda face: -face[0])

        for _, vertices, color in visible:
            points = []
            for x, _, z in vertices:
                points.extend((cx + x * scale, cy - z * scale))

            canvas.create_polygon(
                points, fill=color, outline=color, width=1
            )
            