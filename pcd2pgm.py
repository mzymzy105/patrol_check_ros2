#!/usr/bin/env python3
"""
点云 PCD -> Nav2 map_server 占据栅格地图 (.pgm + .yaml)

流程：
  1. 读全局点云 + 轨迹
  2. RANSAC 找主地面平面 -> 推 LiDAR 离地高度
  3. 用轨迹点反推「地面高度场」(支持坡道/缓坡)
  4. 每个点按它所在位置的局部地面高度做 z 裁剪
     (去掉地面薄层 / 头顶上层结构 / 地下层)
  5. 投影到 2D 网格(0.05m) -> 输出 PGM + YAML + 预览图

用法:
    python pcd2pgm.py
参数在下方 CONFIG 里改。
"""
import os
import numpy as np
import open3d as o3d
from scipy.interpolate import NearestNDInterpolator
from PIL import Image, ImageDraw

# ---------------- 参数 ----------------
CONFIG = dict(
    pcd_path      = "20260804_211333/GlobalMap.pcd",
    ref_path      = "20260804_211333/filterGlobalMap.pcd",  # 用于快速 RANSAC 找地面
    traj_path     = "20260804_211333/trajectory.pcd",
    resolution    = 0.05,   # 米/像素
    ground_cell   = 0.5,    # 地面高度场网格尺寸(m)
    z_clearance   = 0.15,   # 地面以上多少米开始算障碍物(去掉地面薄层)
    z_height      = 2.50,   # 障碍物最高保留到地面以上多少米(去掉头顶上层)
    min_points    = 1,      # 一个 cell 里至少多少点才判为占据(抗噪)
    out_dir       = "nav2_map",
)

# ---------------- 读取 ----------------
pcd = o3d.io.read_point_cloud(CONFIG["pcd_path"])
pts = np.asarray(pcd.points).astype(np.float64)
del pcd
print(f"原始点数: {len(pts)}")

traj = o3d.io.read_point_cloud(CONFIG["traj_path"])
tpts = np.asarray(traj.points).astype(np.float64)
del traj
print(f"轨迹点数: {len(tpts)}")

# 地图范围用「全部点云」的 xy 范围(包含空旷区，保证地图完整)
x_min, x_max = pts[:, 0].min(), pts[:, 0].max()
y_min, y_max = pts[:, 1].min(), pts[:, 1].max()

# ---------------- RANSAC 找主地面 -> LiDAR 离地高度 ----------------
ref = o3d.io.read_point_cloud(CONFIG["ref_path"])
ref.normals = o3d.utility.Vector3dVector(np.zeros((0, 3)))
lidar_z = np.median(tpts[:, 2])

# 迭代找多个水平面(地面/天花板/上层楼板都可能被先找到)
candidates = []
rest = ref
for _ in range(8):
    model, inliers = rest.segment_plane(distance_threshold=0.10, ransac_n=3, num_iterations=2000)
    a, b, c, d = model
    if abs(c) > 0.85:  # 水平面
        candidates.append((-d / c, len(inliers)))
    rest = rest.select_by_index(inliers, invert=True)
    if len(rest.points) < 1000:
        break
del ref

print("检测到的水平面高度: ", [f"{z:.2f}({n})" for z, n in candidates])

# 地面 = 轨迹下方、最接近轨迹高度的水平面
below = [z for z, _ in candidates if z < lidar_z - 0.1]
if below:
    ground_plane_z = max(below)   # 轨迹下方最近的一个
else:
    ground_plane_z = min(z for z, _ in candidates)  # 兜底
lidar_h = lidar_z - ground_plane_z
if not (0.15 <= lidar_h <= 1.5):
    print(f"  [警告] LiDAR 离地高度 {lidar_h:.2f} m 不合理，请检查地面检测")
print(f"主地面平面 z≈{ground_plane_z:.3f} m, 轨迹中位 z={lidar_z:.3f} m, "
      f"-> LiDAR 离地高度≈{lidar_h:.3f} m")

# ---------------- 地面高度场 (轨迹锚定，支持坡道) ----------------
# 每个轨迹点给出一个「地面样本」: (x, y, z - lidar_h)
samples_xy = tpts[:, :2]
samples_z = tpts[:, 2] - lidar_h
interp = NearestNDInterpolator(samples_xy, samples_z)

gcell = CONFIG["ground_cell"]
Wg = int(np.ceil((x_max - x_min) / gcell))
Hg = int(np.ceil((y_max - y_min) / gcell))
gx = x_min + (np.arange(Wg) + 0.5) * gcell
gy = y_min + (np.arange(Hg) + 0.5) * gcell
GX, GY = np.meshgrid(gx, gy)
ground_grid = interp(GX.ravel(), GY.ravel()).reshape(Hg, Wg)
# 限制外推范围到轨迹实际地面高度区间，防止远离轨迹处离谱
z_lo, z_hi = samples_z.min(), samples_z.max()
ground_grid = np.clip(ground_grid, z_lo, z_hi)
print(f"地面高度场范围: [{z_lo:.2f}, {z_hi:.2f}] m")

# ---------------- 逐点局部 z 裁剪 ----------------
gc = np.floor((pts[:, 0] - x_min) / gcell).astype(np.int64)
gr = np.floor((pts[:, 1] - y_min) / gcell).astype(np.int64)
gc = np.clip(gc, 0, Wg - 1)
gr = np.clip(gr, 0, Hg - 1)
local_ground = ground_grid[gr, gc]

z_min_local = local_ground + CONFIG["z_clearance"]
z_max_local = local_ground + CONFIG["z_height"]
mask = (pts[:, 2] >= z_min_local) & (pts[:, 2] <= z_max_local)
pts = pts[mask]
print(f"局部 z 裁剪后点数: {len(pts)}")

# ---------------- 2D 投影 ----------------
res = CONFIG["resolution"]
W = int(np.ceil((x_max - x_min) / res))
H = int(np.ceil((y_max - y_min) / res))
print(f"地图尺寸: {W} x {H} px  (约 {W*res:.1f}m x {H*res:.1f}m)")

cols = np.floor((pts[:, 0] - x_min) / res).astype(np.int32)
rows = np.floor((y_max - pts[:, 1]) / res).astype(np.int32)  # 图像第0行 = 最高 y
valid = (cols >= 0) & (cols < W) & (rows >= 0) & (rows < H)
cols, rows = cols[valid], rows[valid]

idx = rows * W + cols
counts = np.bincount(idx, minlength=W * H).reshape(H, W)

# 占据栅格: 0=占据(黑), 254=空闲(白), 205=未知(灰)
occ = counts >= CONFIG["min_points"]
img = np.where(occ, 0, 254).astype(np.uint8)
print(f"占据 cell 数: {occ.sum()}  ({occ.sum()/occ.size*100:.1f}%)")

# ---------------- 输出 PGM + YAML ----------------
os.makedirs(CONFIG["out_dir"], exist_ok=True)
pgm_path = os.path.join(CONFIG["out_dir"], "map.pgm")
yaml_path = os.path.join(CONFIG["out_dir"], "map.yaml")

# 手写标准 P5 binary PGM(map_server 最兼容格式)
with open(pgm_path, "wb") as f:
    f.write(f"P5\n{W} {H}\n255\n".encode())
    f.write(img.tobytes())

yaml_text = f"""image: map.pgm
mode: trinary
resolution: {res}
origin: [{x_min:.6f}, {y_min:.6f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
with open(yaml_path, "w") as f:
    f.write(yaml_text)
print(f"已写出: {pgm_path}")
print(f"已写出: {yaml_path}")
print(yaml_text)

# ---------------- 预览图(叠加轨迹) ----------------
preview = Image.fromarray(img, "L").convert("RGB")  # 黑障碍 / 白空闲
draw = ImageDraw.Draw(preview)
for p in tpts:
    c = int(np.floor((p[0] - x_min) / res))
    r = int(np.floor((y_max - p[1]) / res))
    if 0 <= c < W and 0 <= r < H:
        draw.ellipse([c - 2, r - 2, c + 2, r + 2], fill=(255, 0, 0))  # 轨迹=红色
prev_path = os.path.join(CONFIG["out_dir"], "map_preview.png")
preview.save(prev_path)
print(f"已写出预览图: {prev_path} (红色=轨迹)")
