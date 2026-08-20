#!/usr/bin/env python3
"""
把 Nav2 占据栅格地图 (map.pgm) 转成 Gazebo world：
  1. 读地图 + origin/resolution
  2. cv2.findContours 提取障碍物轮廓
  3. 轮廓简化 (approxPolyDP)
  4. trimesh 把每个轮廓挤出成 3D 墙 (z: 0 ~ wall_height)
  5. 合并导出 STL，生成 .world 文件(地面 + 墙 mesh)

这样 Gazebo 的虚拟墙和地图完全一致，虚拟激光雷达扫描能匹配地图，AMCL 可定位。
"""
import os
import re
import numpy as np
import cv2
import trimesh
from shapely.geometry import Polygon, MultiPolygon
from shapely.validation import make_valid

# ---------------- 参数 ----------------
MAP = "nav2_map/map.pgm"
YAML = "nav2_map/map.yaml"
TRAJ = "20260804_211333/trajectory.pcd"
WALL_H = 2.0        # 墙的高度(m)
EPSILON_PX = 1.0    # 轮廓简化精度(像素, 1px = 0.05m @0.05m)
OUT_DIR = "nav2_world"


def read_pgm(path):
    data = open(path, "rb").read()
    assert data[:2] == b"P5", "不是 P5 PGM"
    pos = 2
    vals = []
    while len(vals) < 3:
        while data[pos] in b" \t\r\n#":
            if data[pos] == ord("#"):
                while data[pos] != ord("\n"):
                    pos += 1
            pos += 1
        s = pos
        while data[pos] not in b" \t\r\n":
            pos += 1
        vals.append(int(data[s:pos]))
    W, H, _ = vals
    img = np.frombuffer(data, dtype=np.uint8, offset=pos + 1).reshape(H, W)
    return W, H, img


def read_pcd_xyz(path):
    data = open(path, "rb").read()
    fields = None
    n = 0
    lines = data.split(b"\n", 11)
    hdr = b"\n".join(lines[:-1])
    for ln in hdr.split(b"\n"):
        if ln.startswith(b"FIELDS"):
            fields = ln.split()[1:]
        if ln.startswith(b"POINTS"):
            n = int(ln.split()[1])
    off = len(hdr) + 1
    nf = len(fields)
    arr = np.frombuffer(data, dtype=np.float32, offset=off, count=n * nf).reshape(n, nf)
    ix = {f.decode(): i for i, f in enumerate(fields)}
    return arr[:, [ix["x"], ix["y"], ix["z"]]]


def main():
    # 读 origin
    yaml_txt = open(YAML).read()
    res = float(re.search(r"resolution:\s*([\d.]+)", yaml_txt).group(1))
    origin_x = float(re.search(r"origin:\s*\[([-\d.]+)", yaml_txt).group(1))
    origin_y = float(re.search(r"origin:\s*\[[-\d.]+, ([-\d.]+)", yaml_txt).group(1))

    W, H, img = read_pgm(MAP)
    occ = (img == 0).astype(np.uint8) * 255  # 占据=255(白) 供 findContours
    print(f"地图 {W}x{H}, origin({origin_x},{origin_y}), res={res}")

    contours, _ = cv2.findContours(occ, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"原始轮廓数: {len(contours)}")

    meshes = []
    kept = 0
    for c in contours:
        if len(c) < 3:
            continue
        # 简化
        approx = cv2.approxPolyDP(c, EPSILON_PX, True)
        poly = approx.reshape(-1, 2)  # (col, row)
        # 过滤太小(噪声)
        area = cv2.contourArea(approx)
        if area < 4:  # < 4 px^2 ≈ 0.01 m^2 的噪声块
            continue
        # 像素坐标 -> 世界坐标 (米)
        wx = origin_x + (poly[:, 0] + 0.5) * res
        wy = origin_y + (H - poly[:, 1] - 0.5) * res
        pts = np.column_stack([wx, wy])
        if len(pts) < 3:
            continue
        try:
            poly = Polygon(pts)
            if not poly.is_valid:
                poly = make_valid(poly)  # 修复自交多边形
            if poly.is_empty:
                continue
            # make_valid 可能返回 MultiPolygon，拆开逐个挤出
            geoms = poly.geoms if isinstance(poly, MultiPolygon) else [poly]
            for g in geoms:
                if g.is_empty or g.geom_type != "Polygon":
                    continue
                m = trimesh.creation.extrude_polygon(g, height=WALL_H)
                meshes.append(m)
                kept += 1
        except Exception as e:
            print(f"  轮廓挤出失败({len(pts)}点): {e}")
            continue

    print(f"保留并挤出 {kept} 个轮廓")

    if not meshes:
        print("没有生成任何墙，退出")
        return

    combined = trimesh.util.concatenate(meshes)
    os.makedirs(OUT_DIR, exist_ok=True)
    stl_path = os.path.join(OUT_DIR, "map_walls.stl")
    combined.export(stl_path)
    print(f"已导出 STL: {stl_path}  (顶点 {len(combined.vertices)}, 面 {len(combined.faces)})")

    # 找机器人 spawn 位置：轨迹的中间点(避开起点/终点可能靠墙)
    traj = read_pcd_xyz(TRAJ)
    idx = len(traj) // 2
    sx, sy = float(traj[idx, 0]), float(traj[idx, 1])
    print(f"建议机器人 spawn: world({sx:.2f}, {sy:.2f})  [轨迹中点]")

    # 生成 world 文件
    stl_abs = os.path.abspath(stl_path)
    world_path = os.path.join(OUT_DIR, "world.world")
    world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <!-- 从用户地图挤出的墙 -->
    <model name="map_walls">
      <static>true</static>
      <link name="walls">
        <collision name="collision">
          <geometry><mesh><uri>file://{stl_abs}</uri></mesh></geometry>
        </collision>
        <visual name="visual">
          <geometry><mesh><uri>file://{stl_abs}</uri></mesh></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
"""
    with open(world_path, "w") as f:
        f.write(world)
    print(f"已写出 world: {world_path}")
    print(f"\n提示: 机器人 spawn 位置建议用 world({sx:.2f}, {sy:.2f})")


if __name__ == "__main__":
    main()
