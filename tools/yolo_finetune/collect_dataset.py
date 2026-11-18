"""Collect an auto-labelled YOLO dataset from the running Gazebo world (semih_world).

The robot is teleported (Gazebo set_pose) to sampled poses around the known objects. At each
pose one RGB frame and the matching depth frame are saved. Labels come from geometry, not from
a person: every depth pixel is turned into a world point, and pixels whose point lies inside an
object's footprint (and above the floor) form that object's mask. The box around the mask is the
label, so occlusion by walls / other objects is handled by the depth itself.

usage (sim must be running, e.g. `ros2 launch semih_description spawn_gazebo.launch.py`):
  python3 collect_dataset.py <out_dir> [--n-per-object 400] [--n-random 150] [--seed 0]
Writes <out_dir>/images/*.jpg, <out_dir>/labels/*.txt (YOLO format, COCO class ids),
<out_dir>/meta.csv (pose of each frame) and <out_dir>/preview/*.jpg (every 25th frame boxed).
"""
import argparse
import csv
import math
import random
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# Objects in worlds/semih_world.sdf: name -> (COCO id, x, y, footprint radius m, max height m).
# Radii/heights were checked against the depth point cloud of each object (see README).
OBJECTS = {
    'backpack_1': (24, 3.0, -1.0, 0.32, 0.60),
    'suitcase_1': (28, 5.0, 1.5, 0.42, 0.85),
    'chair_1': (56, 2.0, 1.0, 0.50, 1.10),
}
# walls: (cx, cy, half-size x, half-size y)
WALLS = [(4, -3, 0.05, 3), (4, 3.05, 2, 0.05), (8, 0, 0.05, 3), (4, -6, 4, 0.05), (6, -1.5, 0.05, 1.5)]
CAM_OFF = (0.2, 0.0, 0.095)   # camera_link in base_link (semih.urdf.xacro)
BASE_Z = 0.127                # base_link height above the floor when resting on the wheels
W, H, HFOV = 640, 480, 1.047
FX = (W / 2) / math.tan(HFOV / 2)
CX, CY = W / 2, H / 2
MIN_PIX = 120                 # masks smaller than this are not labelled (too far / mostly hidden)


class Grab(Node):
    def __init__(self):
        super().__init__('yolo_dataset_collector')
        self.rgb = self.depth = None
        self.create_subscription(Image, '/camera/image_raw', lambda m: setattr(self, 'rgb', (time.time(), m)), 2)
        self.create_subscription(Image, '/depth_camera', lambda m: setattr(self, 'depth', (time.time(), m)), 2)

    def fresh(self, after, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.rgb and self.depth and self.rgb[0] > after and self.depth[0] > after:
                return self.rgb[1], self.depth[1]
        return None, None


def set_pose(x, y, yaw):
    req = (f'name: "semih", position: {{x: {x}, y: {y}, z: {BASE_Z + 0.005}}}, '
           f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
    r = subprocess.run(['ign', 'service', '-s', '/world/semih_world/set_pose', '--reqtype', 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', req],
                       capture_output=True, text=True)
    return 'true' in r.stdout


def free(x, y, clear=0.45):
    if not (-1.0 < x < 7.6 and -5.6 < y < 2.7):
        return False
    for _, ox, oy, r, _ in OBJECTS.values():
        if math.hypot(x - ox, y - oy) < r + clear:
            return False
    for cx, cy, hx, hy in WALLS:
        if abs(x - cx) < hx + clear and abs(y - cy) < hy + clear:
            return False
    return True


def sample_poses(n_per, n_rand, rng):
    poses = []
    for name, (_, ox, oy, _, _) in OBJECTS.items():
        k = 0
        while k < n_per:
            d, b = rng.uniform(0.55, 3.5), rng.uniform(-math.pi, math.pi)
            x, y = ox + d * math.cos(b), oy + d * math.sin(b)
            if not free(x, y):
                continue
            yaw = math.atan2(oy - y, ox - x) + math.radians(rng.uniform(-26, 26))
            poses.append((name, x, y, yaw))
            k += 1
    while len(poses) < len(OBJECTS) * n_per + n_rand:
        x, y = rng.uniform(-0.8, 7.4), rng.uniform(-5.4, 2.5)
        if free(x, y):
            poses.append(('random', x, y, rng.uniform(-math.pi, math.pi)))
    rng.shuffle(poses)
    return poses


def to_np(msg, depth=False):
    if depth:
        return np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
    return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else a.copy()


def labels_from_depth(depth, x, y, yaw):
    v, u = np.mgrid[0:H, 0:W]
    z = depth
    ok = np.isfinite(z) & (z > 0.05) & (z < 9.0)
    # optical -> camera_link: forward = z, left = -x_opt, up = -y_opt
    fwd, left, up = z, -(u - CX) * z / FX, -(v - CY) * z / FX
    cx_ = x + math.cos(yaw) * CAM_OFF[0]
    cy_ = y + math.sin(yaw) * CAM_OFF[0]
    wx = cx_ + math.cos(yaw) * fwd - math.sin(yaw) * left
    wy = cy_ + math.sin(yaw) * fwd + math.cos(yaw) * left
    wz = BASE_Z + CAM_OFF[2] + up
    out = []
    for name, (cls, ox, oy, r, h) in OBJECTS.items():
        m = ok & (np.hypot(wx - ox, wy - oy) < r) & (wz > 0.015) & (wz < h)
        m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        if m.sum() < MIN_PIX:
            continue
        ys, xs = np.nonzero(m)
        x1, x2, y1, y2 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
        out.append((cls, name, x1, y1, x2, y2, int(m.sum())))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--n-per-object', type=int, default=400)
    ap.add_argument('--n-random', type=int, default=150)
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    out = Path(a.out)
    for sub in ('images', 'labels', 'preview'):
        (out / sub).mkdir(parents=True, exist_ok=True)
    rng = random.Random(a.seed)
    poses = sample_poses(a.n_per_object, a.n_random, rng)
    rclpy.init()
    g = Grab()
    meta = open(out / 'meta.csv', 'w', newline='')
    wr = csv.writer(meta)
    wr.writerow(['file', 'aim', 'x', 'y', 'yaw', 'labels'])
    t_start, n_ok = time.time(), 0
    for i, (aim, x, y, yaw) in enumerate(poses):
        if not set_pose(x, y, yaw):
            print(f'{i}: set_pose failed', flush=True)
            continue
        t_set = time.time()
        time.sleep(0.35)
        rgb, dep = g.fresh(t_set + 0.3)
        if rgb is None:
            print(f'{i}: no fresh frames', flush=True)
            continue
        img, depth = to_np(rgb), to_np(dep, depth=True)
        labs = labels_from_depth(depth, x, y, yaw)
        stem = f's{a.seed}_{i:05d}'
        cv2.imwrite(str(out / 'images' / f'{stem}.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
        with open(out / 'labels' / f'{stem}.txt', 'w') as f:
            for cls, _, x1, y1, x2, y2, _ in labs:
                f.write(f'{cls} {(x1 + x2) / 2 / W:.6f} {(y1 + y2) / 2 / H:.6f} {(x2 - x1) / W:.6f} {(y2 - y1) / H:.6f}\n')
        wr.writerow([stem, aim, f'{x:.3f}', f'{y:.3f}', f'{yaw:.4f}', ';'.join(f'{n}:{p}' for _, n, *_, p in labs)])
        if i % 25 == 0:
            pv = img.copy()
            for cls, name, x1, y1, x2, y2, _ in labs:
                cv2.rectangle(pv, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(pv, name, (x1, max(14, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imwrite(str(out / 'preview' / f'{stem}.jpg'), pv)
        n_ok += 1
        if i % 50 == 0:
            print(f'{i}/{len(poses)} ok={n_ok} {time.time() - t_start:.0f}s', flush=True)
    meta.close()
    print(f'done: {n_ok} frames in {time.time() - t_start:.0f}s', flush=True)


if __name__ == '__main__':
    main()
