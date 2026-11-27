"""TwinArm-5 product photo (studio, Cycles).

    cad model export <id> render/twinarm.glb --tolerance 0.03 --angle 5
    cad run render/render.py            # EEVEE draft   -> render/draft.png
    cad run render/render.py final      # Cycles (GPU)  -> render/hero.png
"""
import math
import sys
from pathlib import Path

import bpy
import bmesh  # noqa: E402  (after bpy)
from mathutils import Vector

from cad.blender import load

HERE = Path(__file__).resolve().parent
FINAL = "final" in sys.argv[1:]
POSE_T = 2.0            # "ready" pose in the demo clip (s)
FPS = 24

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
model = load(HERE / "twinarm.glb", fps=FPS)
if "demo" in model.clips:
    model.play("demo")
    scene.frame_set(int(POSE_T * FPS))
lo, hi = model.bounds()
center = (lo + hi) / 2
size = hi - lo
print("bounds", lo, hi)


# ------------------------------------------------------------------------------------------
# materials: keep the CAD colors, make the surface character realistic
# ------------------------------------------------------------------------------------------
def principled(mat):
    if not mat.use_nodes:
        return None
    return next((n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED"), None)


def add_print_texture(mat, strength=0.12):
    """FDM layer lines: fine wave along Z + light noise (bump)."""
    nt = mat.node_tree
    bsdf = principled(mat)
    coord = nt.nodes.new("ShaderNodeTexCoord")
    wave = nt.nodes.new("ShaderNodeTexWave")
    wave.wave_type, wave.bands_direction = "BANDS", "Z"
    wave.inputs["Scale"].default_value = 260.0          # ~0.2 mm layer (model in metres)
    wave.inputs["Distortion"].default_value = 0.4
    noise = nt.nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 900.0
    mix = nt.nodes.new("ShaderNodeMath")
    mix.operation = "ADD"
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = strength
    bump.inputs["Distance"].default_value = 0.00008
    nt.links.new(coord.outputs["Object"], wave.inputs["Vector"])
    nt.links.new(coord.outputs["Object"], noise.inputs["Vector"])
    nt.links.new(wave.outputs["Fac"], mix.inputs[0])
    nt.links.new(noise.outputs["Fac"], mix.inputs[1])
    nt.links.new(mix.outputs["Value"], bump.inputs["Height"])
    nt.links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])


for mat in list(bpy.data.materials):
    bsdf = principled(mat)
    if bsdf is None:
        continue
    name = mat.name.lower()
    r, g, b, _ = bsdf.inputs["Base Color"].default_value
    bright = max(r, g, b)
    if name.startswith("plastic_fdm"):                   # PETG printed parts
        bsdf.inputs["Roughness"].default_value = 0.42
        if "Coat Weight" in bsdf.inputs:
            bsdf.inputs["Coat Weight"].default_value = 0.06
            bsdf.inputs["Coat Roughness"].default_value = 0.35
        add_print_texture(mat)
    elif name.startswith("steel"):
        if bright < 0.1:                                 # black-oxide screw
            bsdf.inputs["Metallic"].default_value = 0.7
            bsdf.inputs["Roughness"].default_value = 0.4
        else:                                            # bearing steel
            bsdf.inputs["Roughness"].default_value = 0.2
    elif name.startswith("aluminum"):
        bsdf.inputs["Roughness"].default_value = 0.32
    elif name.startswith("brass"):
        bsdf.inputs["Roughness"].default_value = 0.3
    elif name.startswith("plastic"):
        if b > 0.2 and r < 0.05:                         # cable jacket (blue) -> black braid
            bsdf.inputs["Base Color"].default_value = (0.02, 0.022, 0.026, 1)
            bsdf.inputs["Roughness"].default_value = 0.75
        else:                                            # servo body, horn, sensor
            bsdf.inputs["Roughness"].default_value = 0.48

# ------------------------------------------------------------------------------------------
# studio: curved infinity backdrop
# ------------------------------------------------------------------------------------------
def cyclorama(width, depth, height, radius, z0):
    mesh = bpy.data.meshes.new("cyc")
    bm = bmesh.new()
    prof = [(-depth, z0)]
    steps = 24
    for i in range(steps + 1):
        a = math.pi / 2 * i / steps
        prof.append((radius * math.sin(a), z0 + radius * (1 - math.cos(a))))
    prof.append((radius, z0 + height))
    rows = []
    for y, z in prof:
        rows.append([bm.verts.new((x, y, z)) for x in (-width / 2, width / 2)])
    for a, b in zip(rows, rows[1:]):
        bm.faces.new((a[0], a[1], b[1], b[0]))
    bm.to_mesh(mesh)
    bm.free()
    for p in mesh.polygons:
        p.use_smooth = True
    obj = bpy.data.objects.new("cyclorama", mesh)
    mat = bpy.data.materials.new("studio")
    mat.use_nodes = True
    bs = principled(mat)
    bs.inputs["Base Color"].default_value = (0.45, 0.46, 0.48, 1)
    bs.inputs["Roughness"].default_value = 0.6
    obj.data.materials.append(mat)
    scene.collection.objects.link(obj)
    return obj


L = size.length
cyc = cyclorama(width=12 * L, depth=6 * L, height=4 * L, radius=1.2 * L, z0=0.0)   # underside of the base plate
cyc.location.y = center.y + 1.6 * L                     # backdrop behind the arm

# ------------------------------------------------------------------------------------------
# camera
# ------------------------------------------------------------------------------------------
W, H = (2400, 1350) if FINAL else (1280, 720)
scene.render.resolution_x, scene.render.resolution_y = W, H
aim = bpy.data.objects.new("aim", None)
aim.location = center + Vector((0.01, 0.0, 0.01 * L))
camera = bpy.data.objects.new("camera", bpy.data.cameras.new("camera"))
camera.data.lens = 70
view = math.tan(camera.data.angle / 2) * min(1, H / W)
distance = L / 2 / view * 0.78
camera.location = center + distance * Vector((1.0, -1.25, 0.32)).normalized()
camera.constraints.new("TRACK_TO").target = aim
camera.data.dof.use_dof = True
camera.data.dof.focus_object = aim
camera.data.dof.aperture_fstop = 5.6
scene.collection.objects.link(aim)
scene.collection.objects.link(camera)
scene.camera = camera


def area(name, direction, power, size_k, color=(1, 1, 1)):
    light = bpy.data.objects.new(name, bpy.data.lights.new(name, "AREA"))
    light.data.shape = "RECTANGLE"
    light.data.energy = power * distance ** 2
    light.data.size, light.data.size_y = size_k * distance, size_k * distance * 0.6
    light.data.color = color
    light.location = center + distance * Vector(direction).normalized()
    light.constraints.new("TRACK_TO").target = aim
    scene.collection.objects.link(light)


area("key", (-0.8, -1.0, 1.1), 22, 0.9, (1.0, 0.97, 0.93))     # large softbox, front upper left
area("fill", (1.3, -0.6, 0.25), 5, 1.2, (0.93, 0.96, 1.0))     # soft fill from the right
area("rim", (0.4, 1.4, 0.9), 30, 0.35)                         # rim light from behind
area("top", (0.0, 0.1, 1.0), 5, 0.8)

scene.world = bpy.data.worlds.new("world")
scene.world.use_nodes = True
bg = scene.world.node_tree.nodes["Background"]
bg.inputs["Color"].default_value = (0.75, 0.77, 0.8, 1)
bg.inputs["Strength"].default_value = 0.12

scene.view_settings.view_transform = "AgX"
for look in ("AgX - Punchy", "Punchy", "AgX - Medium High Contrast", "Medium High Contrast"):
    try:
        scene.view_settings.look = look
        break
    except TypeError:
        continue

# ------------------------------------------------------------------------------------------
# engine
# ------------------------------------------------------------------------------------------
if FINAL:
    scene.render.engine = "CYCLES"
    prefs = bpy.context.preferences.addons["cycles"].preferences
    for kind in ("OPTIX", "CUDA"):
        try:
            prefs.compute_device_type = kind
        except TypeError:
            continue
        prefs.get_devices()
        if any(d.type == kind for d in prefs.devices):
            break
    for d in prefs.devices:
        d.use = d.type != "CPU"
    scene.cycles.device = "GPU" if any(d.use for d in prefs.devices) else "CPU"
    print("cycles device", scene.cycles.device, [d.name for d in prefs.devices if d.use])
    scene.cycles.samples = 384
    scene.cycles.use_denoising, scene.cycles.denoiser = True, "OPENIMAGEDENOISE"
    scene.cycles.max_bounces = 10
    scene.render.filepath = str(HERE / "hero.png")
else:
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.filepath = str(HERE / "draft.png")
bpy.ops.render.render(write_still=True)
print("wrote", scene.render.filepath)
