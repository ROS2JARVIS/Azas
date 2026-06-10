# RG2 Collision Meshes

Prefer simple collision primitives in `urdf/rg2_parametric.xacro`.

If collision meshes are added later, derive them from official CAD and simplify
them for MoveIt planning. Do not use dense visual CAD meshes directly as
collision geometry.
