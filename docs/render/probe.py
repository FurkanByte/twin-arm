from pathlib import Path

import bpy

from cad.blender import load

HERE = Path(__file__).resolve().parents[2] / "build" / "render"   # GLB in, images out
bpy.ops.wm.read_factory_settings(use_empty=True)
model = load(HERE / "twinarm.glb")
print("clips", {k: (v.kind, v.duration) for k, v in model.clips.items()})
print("box", model.box, "center", model.center, "size", model.size)
for m in bpy.data.materials:
    bsdf = m.node_tree.nodes.get("Principled BSDF") if m.use_nodes else None
    col = tuple(round(c, 3) for c in bsdf.inputs["Base Color"].default_value) if bsdf else None
    users = m.users
    print("mat", m.name, col, "metal", bsdf and round(bsdf.inputs["Metallic"].default_value, 2),
          "rough", bsdf and round(bsdf.inputs["Roughness"].default_value, 2), "users", users)
names = sorted({o.name.split(".")[0] for o in bpy.data.objects if o.type == "MESH"})
print(len(names), names[:60])
