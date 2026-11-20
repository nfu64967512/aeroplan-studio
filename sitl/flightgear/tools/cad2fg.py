# -*- coding: utf-8 -*-
"""任意 CAD 檔 -> FlightGear 方向的 OBJ,**方向自動量測**。

用法:
  blender --background --python cad2fg.py -- <in> <out.obj> [tris] [nose=auto|+X|-X|...]

⚠️⚠️ 為什麼要自動量:舊版寫死「+X 機鼻、+Y 上」,那是**那一個** GLB 的實測值。
   換一個 Fusion 匯出、換一次座標系設定,方向就不一樣,而寫死的程式碼
   **不會報錯,只會把飛機畫成橫著飛或倒著飛**。
   所以這裡量,而且把證據印出來讓人能否決。

判準(和 2026-08-22 解 GLB 軸向用的同一招:網格剖面,不是推理):
  * 機身縱軸 = 包圍盒最長的軸
  * 翼展軸   = 次長(而且應該在機身中段出現最大展寬)
  * 上下軸   = 最短
  * 機鼻方向 = 沿縱軸切格後,截面半徑(r95)**較小**的那一端
                並印出兩端比值;比值太接近時大聲說「分不出來」

FlightGear 的模型座標系(由 c172p.ac 反推,見 sitl/flightgear/README.md):
  -X = 機鼻   +Y = 上   +-Z = 翼展
"""
import math
import sys

import bpy
import mathutils

argv = sys.argv[sys.argv.index("--") + 1:]
src, dst = argv[0], argv[1]
target = int(argv[2]) if len(argv) > 2 else 40000
force_nose = argv[3] if len(argv) > 3 else "auto"

bpy.ops.wm.read_factory_settings(use_empty=True)
low = src.lower()
if low.endswith((".glb", ".gltf")):
    bpy.ops.import_scene.gltf(filepath=src)
elif low.endswith(".obj"):
    bpy.ops.wm.obj_import(filepath=src)
elif low.endswith(".stl"):
    try:
        bpy.ops.wm.stl_import(filepath=src)
    except AttributeError:
        bpy.ops.import_mesh.stl(filepath=src)
elif low.endswith(".fbx"):
    bpy.ops.import_scene.fbx(filepath=src)
elif low.endswith(".dae"):
    bpy.ops.wm.collada_import(filepath=src)
elif low.endswith(".ply"):
    bpy.ops.wm.ply_import(filepath=src)
else:
    print("UNSUPPORTED: %s" % src)
    sys.exit(2)

objs = [o for o in bpy.context.scene.objects if o.type == "MESH"]
if not objs:
    print("NO MESH FOUND")
    sys.exit(2)
print("meshes: %d" % len(objs))

# 收集世界座標頂點(抽樣,量方向不需要全部)
P = []
for o in objs:
    mw = o.matrix_world
    vs = o.data.vertices
    stride = max(1, len(vs) // 60000)
    for i in range(0, len(vs), stride):
        w = mw @ vs[i].co
        P.append((w.x, w.y, w.z))
print("sampled verts: %d" % len(P))

names = "XYZ"
lo = [min(p[i] for p in P) for i in range(3)]
hi = [max(p[i] for p in P) for i in range(3)]
ln = [hi[i] - lo[i] for i in range(3)]
print("RAW bbox:")
for i in range(3):
    print("  %s: [%9.4f %9.4f] len %8.4f" % (names[i], lo[i], hi[i], ln[i]))

order = sorted(range(3), key=lambda i: ln[i], reverse=True)
ax_len, ax_span, ax_up = order[0], order[1], order[2]
print("  推定: 縱軸=%s(%.3f)  翼展=%s(%.3f)  上下=%s(%.3f)"
      % (names[ax_len], ln[ax_len], names[ax_span], ln[ax_span],
         names[ax_up], ln[ax_up]))
if ln[ax_len] < 1.15 * ln[ax_span]:
    print("  ⚠️ 縱軸與翼展長度太接近(%.3f vs %.3f),自動判斷不可靠"
          % (ln[ax_len], ln[ax_span]))

# 機鼻:沿縱軸切格,比兩端截面半徑
o1, o2 = [i for i in range(3) if i != ax_len]
N = 14
step = ln[ax_len] / N
prof = []
for k in range(N):
    a0 = lo[ax_len] + k * step
    r = sorted(math.hypot(p[o1], p[o2]) for p in P if a0 <= p[ax_len] < a0 + step)
    prof.append((a0, len(r), r[int(0.95 * (len(r) - 1))] if len(r) >= 20 else float("nan")))
print("  %10s %8s %10s" % ("from", "n", "r95"))
for a0, n, r in prof:
    print("  %10.4f %8d %10.4f" % (a0, n, r))
good = [p for p in prof if p[1] >= 20]
r_lo, r_hi = good[0][2], good[-1][2]
ratio = max(r_lo, r_hi) / max(1e-9, min(r_lo, r_hi))
nose_pos = r_hi < r_lo          # True => 機鼻在縱軸正方向
print("  兩端 r95: 低 %.4f / 高 %.4f  比值 %.2f" % (r_lo, r_hi, ratio))
if force_nose != "auto":
    want = force_nose.strip()
    nose_pos = want.startswith("+")
    print("  ⚠️ 使用者指定機鼻 = %s,略過自動判斷" % want)
elif ratio < 1.3:
    print("  ⚠️⚠️ 兩端粗細太接近(比值 %.2f < 1.3)=> **分不出機鼻**。" % ratio)
    print("     請用第 4 個參數明確指定,例如 +X 或 -X。不要讓它猜。")
    sys.exit(3)
print("  => 機鼻在 %s 的 %s 方向" % (names[ax_len], "正" if nose_pos else "負"))

# 組旋轉:把 (縱軸,上下,翼展) 映到 FG 的 (-X 機鼻, +Y 上, +-Z 翼展)
e = [mathutils.Vector((1, 0, 0)), mathutils.Vector((0, 1, 0)), mathutils.Vector((0, 0, 1))]
nose_vec = e[ax_len] * (1.0 if nose_pos else -1.0)
up_vec = e[ax_up]
# FG: 機鼻 -X, 上 +Y, 翼展 Z(右手系決定)
col_x = -nose_vec
col_y = up_vec
col_z = col_x.cross(col_y)
M = mathutils.Matrix((
    (col_x.x, col_y.x, col_z.x, 0.0),
    (col_x.y, col_y.y, col_z.y, 0.0),
    (col_x.z, col_y.z, col_z.z, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)).inverted()
det = M.to_3x3().determinant()
print("  rotation det = %.4f" % det)
assert abs(det - 1.0) < 1e-6, "不是真旋轉,det=%.4f" % det

for o in objs:
    o.matrix_world = M @ o.matrix_world

tot = 0
for o in objs:
    o.data.calc_loop_triangles()
    tot += len(o.data.loop_triangles)
print("tris before: %d" % tot)
if tot > target:
    ratio_d = float(target) / tot
    for o in objs:
        m = o.modifiers.new("dec", "DECIMATE")
        m.ratio = ratio_d
    bpy.context.view_layer.update()
    for o in objs:
        bpy.context.view_layer.objects.active = o
        bpy.ops.object.modifier_apply(modifier="dec")
    tot = 0
    for o in objs:
        o.data.calc_loop_triangles()
        tot += len(o.data.loop_triangles)
    print("tris after decimate: %d" % tot)

bpy.ops.wm.obj_export(filepath=dst, export_materials=True,
                      export_triangulated_mesh=True,
                      forward_axis="Y", up_axis="Z")
print("wrote %s" % dst)
