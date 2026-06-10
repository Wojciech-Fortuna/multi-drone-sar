from __future__ import annotations

import os
import numpy as np


def export_terrain_to_webots_proto(
    terrain,
    output_path="protos/GeneratedTerrain.proto",
    x_min=None,
    x_max=None,
    y_min=None,
    y_max=None,
    resolution=None,
):
    x_min = terrain.x_min if x_min is None else x_min
    x_max = terrain.x_max if x_max is None else x_max
    y_min = terrain.y_min if y_min is None else y_min
    y_max = terrain.y_max if y_max is None else y_max
    resolution = terrain.resolution if resolution is None else resolution

    xs = np.arange(x_min, x_max + 0.5 * resolution, resolution)
    ys = np.arange(y_min, y_max + 0.5 * resolution, resolution)

    nx = len(xs)
    ny = len(ys)

    points = []

    for y in ys:
        for x in xs:
            height = float(terrain.get_height(x, y))
            points.append((x, y, height))

    coord_indices = []

    def idx(i, j):
        return j * nx + i

    for j in range(ny - 1):
        for i in range(nx - 1):
            a = idx(i, j)
            b = idx(i + 1, j)
            c = idx(i, j + 1)
            d = idx(i + 1, j + 1)

            # Face winding chosen so the surface normal points upward.
            coord_indices.append((a, b, d, -1))
            coord_indices.append((a, d, c, -1))

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    with open(output_path, "w", encoding="utf-8") as file:
        file.write("#VRML_SIM R2025a utf8\n\n")
        file.write("PROTO GeneratedTerrain [\n")
        file.write("]\n")
        file.write("{\n")
        file.write("  Solid {\n")
        file.write('    name "terrain_3d"\n')
        file.write("    children [\n")
        file.write("      Shape {\n")
        file.write("        appearance PBRAppearance {\n")
        file.write("          baseColor 0.18 0.45 0.22\n")
        file.write("          roughness 1\n")
        file.write("        }\n")
        file.write("        geometry IndexedFaceSet {\n")
        file.write("          coord Coordinate {\n")
        file.write("            point [\n")

        for x, y, z in points:
            file.write(f"              {x:.4f} {y:.4f} {z:.4f},\n")

        file.write("            ]\n")
        file.write("          }\n")
        file.write("          coordIndex [\n")

        for face in coord_indices:
            file.write(
                "            "
                + " ".join(str(v) for v in face)
                + ",\n"
            )

        file.write("          ]\n")
        file.write("        }\n")
        file.write("      }\n")
        file.write("    ]\n")
        file.write("  }\n")
        file.write("}\n")

    print(f"[Webots terrain export] Saved terrain PROTO to: {output_path}")
