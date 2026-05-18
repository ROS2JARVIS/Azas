#!/usr/bin/env python3
"""Generate simple OBJ/MTL models from Azas wiki tumbler/dispenser specs.

Units are meters. The models are lightweight digital-twin primitives for RViz,
planning-scene discussion, and Isaac Sim import experiments. They are not
calibrated collision geometry for real robot execution.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

SEGMENTS = 64


@dataclass
class Mesh:
    name: str
    vertices: list[tuple[float, float, float]] = field(default_factory=list)
    faces: list[tuple[str, list[int]]] = field(default_factory=list)

    def add_vertex(self, x: float, y: float, z: float) -> int:
        self.vertices.append((x, y, z))
        return len(self.vertices)

    def add_face(self, material: str, indices: list[int]) -> None:
        self.faces.append((material, indices))

    def add_cylinder(self, material: str, radius: float, height: float, center: tuple[float, float, float], segments: int = SEGMENTS, radius_top: float | None = None, cap_top: bool = True, cap_bottom: bool = True) -> None:
        radius_top = radius if radius_top is None else radius_top
        cx, cy, cz = center
        bottom = []
        top = []
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            ca = math.cos(a)
            sa = math.sin(a)
            bottom.append(self.add_vertex(cx + radius * ca, cy + radius * sa, cz - height / 2.0))
            top.append(self.add_vertex(cx + radius_top * ca, cy + radius_top * sa, cz + height / 2.0))
        for i in range(segments):
            j = (i + 1) % segments
            self.add_face(material, [bottom[i], bottom[j], top[j], top[i]])
        if cap_bottom:
            self.add_face(material, list(reversed(bottom)))
        if cap_top:
            self.add_face(material, top)

    def add_box(self, material: str, size: tuple[float, float, float], center: tuple[float, float, float]) -> None:
        sx, sy, sz = size
        cx, cy, cz = center
        pts = [
            (cx - sx/2, cy - sy/2, cz - sz/2),
            (cx + sx/2, cy - sy/2, cz - sz/2),
            (cx + sx/2, cy + sy/2, cz - sz/2),
            (cx - sx/2, cy + sy/2, cz - sz/2),
            (cx - sx/2, cy - sy/2, cz + sz/2),
            (cx + sx/2, cy - sy/2, cz + sz/2),
            (cx + sx/2, cy + sy/2, cz + sz/2),
            (cx - sx/2, cy + sy/2, cz + sz/2),
        ]
        idx = [self.add_vertex(*p) for p in pts]
        for face in ([1,2,3,4], [5,8,7,6], [1,5,6,2], [2,6,7,3], [3,7,8,4], [4,8,5,1]):
            self.add_face(material, [idx[i-1] for i in face])

    def add_torus(self, material: str, major_radius: float, minor_radius: float, center: tuple[float, float, float], major_segments: int = SEGMENTS, minor_segments: int = 12) -> None:
        cx, cy, cz = center
        grid = []
        for i in range(major_segments):
            a = 2.0 * math.pi * i / major_segments
            ca = math.cos(a)
            sa = math.sin(a)
            row = []
            for j in range(minor_segments):
                b = 2.0 * math.pi * j / minor_segments
                rb = major_radius + minor_radius * math.cos(b)
                row.append(self.add_vertex(cx + rb * ca, cy + rb * sa, cz + minor_radius * math.sin(b)))
            grid.append(row)
        for i in range(major_segments):
            ni = (i + 1) % major_segments
            for j in range(minor_segments):
                nj = (j + 1) % minor_segments
                self.add_face(material, [grid[i][j], grid[ni][j], grid[ni][nj], grid[i][nj]])

    def translated(self, name: str, dx: float, dy: float, dz: float) -> "Mesh":
        return Mesh(name, [(x + dx, y + dy, z + dz) for x, y, z in self.vertices], list(self.faces))

    def write_obj(self, directory: Path, mtl_name: str) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        obj_path = directory / f"{self.name}.obj"
        with obj_path.open("w", encoding="utf-8") as f:
            f.write("# Generated from Azas wiki specs. Units: meters.\n")
            f.write(f"mtllib {mtl_name}\n")
            f.write(f"o {self.name}\n")
            for x, y, z in self.vertices:
                f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
            current = None
            for material, indices in self.faces:
                if material != current:
                    f.write(f"usemtl {material}\n")
                    current = material
                f.write("f " + " ".join(str(i) for i in indices) + "\n")


def merge(name: str, meshes: list[Mesh]) -> Mesh:
    out = Mesh(name)
    offset = 0
    for mesh in meshes:
        out.vertices.extend(mesh.vertices)
        for material, face in mesh.faces:
            out.faces.append((material, [i + offset for i in face]))
        offset += len(mesh.vertices)
    return out


def tumbler_mesh(name: str = "azas_tumbler_shaker") -> Mesh:
    m = Mesh(name)
    body_height = 0.140
    lidded_height = 0.170
    bottom_radius = 0.032
    top_radius = 0.0375
    m.add_cylinder("tumbler_clear_body", bottom_radius, body_height, (0, 0, body_height / 2), radius_top=top_radius)
    m.add_cylinder("tumbler_lid_band", top_radius + 0.003, 0.018, (0, 0, body_height + 0.009), radius_top=top_radius + 0.003)
    m.add_cylinder("tumbler_lid_dome", top_radius * 0.82, 0.012, (0, 0, body_height + 0.024), radius_top=top_radius * 0.55)
    m.add_torus("tumbler_rim", top_radius + 0.002, 0.002, (0, 0, body_height + 0.018))
    m.add_box("tumbler_lid_tab", (0.030, 0.014, 0.006), (0.020, 0, lidded_height - 0.003))
    m.add_cylinder("axis_marker_blue", 0.003, 0.020, (0, 0, lidded_height + 0.010), segments=24)
    return m


def dispenser_mesh(name: str = "azas_dispenser_single") -> Mesh:
    m = Mesh(name)
    bottle_w = 0.058
    bottle_h = 0.275
    mouth_outer_r = 0.014
    mouth_inner_r = 0.009
    tube_outer_r = 0.00425
    tube_len = 0.205
    pump_head_len = 0.195
    pump_exposed = 0.117
    m.add_box("dispenser_clear_bottle", (bottle_w, bottle_w, bottle_h), (0, 0, bottle_h / 2))
    m.add_cylinder("dispenser_mouth_outer", mouth_outer_r, 0.018, (0, 0, bottle_h + 0.009), segments=48)
    m.add_cylinder("dispenser_mouth_inner_dark", mouth_inner_r, 0.020, (0, 0, bottle_h + 0.011), segments=48)
    m.add_cylinder("dispenser_black_plastic", tube_outer_r, tube_len, (0, 0, bottle_h - tube_len / 2), segments=24)
    stem_z = bottle_h + 0.018 + pump_exposed / 2
    m.add_cylinder("dispenser_black_plastic", tube_outer_r * 1.3, pump_exposed, (0, 0, stem_z), segments=24)
    head_z = bottle_h + 0.018 + pump_exposed
    m.add_box("dispenser_black_plastic", (pump_head_len, 0.018, 0.018), (pump_head_len / 2 - 0.012, 0, head_z))
    m.add_cylinder("dispenser_outlet_blue", 0.005, 0.018, (pump_head_len - 0.020, 0, head_z - 0.018), segments=24)
    m.add_cylinder("axis_marker_blue", 0.004, 0.030, (pump_head_len - 0.020, 0, head_z - 0.040), segments=24)
    return m


def four_dispenser_row() -> Mesh:
    spacing = 0.085
    base = dispenser_mesh("single_template")
    return merge("azas_four_dispenser_row", [base.translated(f"dispenser_{i+1}", 0, (i - 1.5) * spacing, 0) for i in range(4)])


def scene_preview() -> Mesh:
    return merge("azas_tumbler_dispenser_preview", [four_dispenser_row(), tumbler_mesh("cup").translated("cup", 0.12, 0.0, 0.0)])


def write_materials(directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "azas_models.mtl").write_text("""# Azas model materials. Transparency is advisory and importer-dependent.
newmtl tumbler_clear_body
Kd 0.85 0.95 1.00
Ka 0.10 0.10 0.12
d 0.35
illum 2

newmtl tumbler_lid_band
Kd 0.08 0.08 0.08
Ka 0.02 0.02 0.02
d 1.0
illum 2

newmtl tumbler_lid_dome
Kd 0.95 0.95 0.92
Ka 0.08 0.08 0.08
d 0.92
illum 2

newmtl tumbler_rim
Kd 1.00 1.00 1.00
Ka 0.10 0.10 0.10
d 0.75
illum 2

newmtl tumbler_lid_tab
Kd 0.08 0.08 0.08
Ka 0.02 0.02 0.02
d 1.0
illum 2

newmtl dispenser_clear_bottle
Kd 0.90 0.98 1.00
Ka 0.08 0.08 0.10
d 0.28
illum 2

newmtl dispenser_mouth_outer
Kd 0.88 0.88 0.88
Ka 0.10 0.10 0.10
d 1.0
illum 2

newmtl dispenser_mouth_inner_dark
Kd 0.02 0.02 0.02
Ka 0.00 0.00 0.00
d 1.0
illum 2

newmtl dispenser_black_plastic
Kd 0.01 0.01 0.01
Ka 0.00 0.00 0.00
d 1.0
illum 2

newmtl dispenser_outlet_blue
Kd 0.10 0.35 1.00
Ka 0.02 0.03 0.08
d 1.0
illum 2

newmtl axis_marker_blue
Kd 0.10 0.35 1.00
Ka 0.02 0.03 0.08
d 1.0
illum 2
""", encoding="utf-8")


def main() -> None:
    out = Path(__file__).resolve().parents[1] / "models"
    write_materials(out)
    for mesh in [tumbler_mesh(), dispenser_mesh(), four_dispenser_row(), scene_preview()]:
        mesh.write_obj(out, "azas_models.mtl")
    print(f"wrote models to {out}")


if __name__ == "__main__":
    main()
