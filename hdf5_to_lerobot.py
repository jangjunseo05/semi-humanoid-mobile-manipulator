#!/usr/bin/env python3
"""
Offline converter: hdf5_collector_node.py output -> LeRobotDataset v3.0.

Deliberately NOT a ROS node and not run inside the ROS Python environment.
`lerobot` pulls in torch/torchvision/diffusers/opencv-python-headless, which
would fight the versions ROS2's cv_bridge and our own YOLO nodes rely on if
installed into the same interpreter. Run this from a dedicated venv instead:

    python3 -m venv ~/lerobot_venv
    source ~/lerobot_venv/bin/activate
    pip install lerobot
    python3 hdf5_to_lerobot.py /path/to/semih_session_XXXXXXXX_XXXXXX.h5 --repo-id local/semih-arm-demo

What this does and does not carry over from the HDF5 file:
  - observation.state <- joint_states/positions (arm_joint1..3, actual measured angles)
  - action            <- action/arm_target_positions (what arm_reach_node commanded).
                         Frames before the first command in the session are NaN in the
                         source file; those are forward-filled from the first valid
                         action, or (if the arm was never commanded at all this session)
                         replaced with observation.state, i.e. treated as a "hold" action.
  - observation.images.front <- camera/images, converted BGR (OpenCV/ROS convention) to
                         RGB (LeRobot/PIL convention) and video-encoded.
  - lidar/*, odom/* are NOT carried over. LeRobotDataset's policy-facing schema doesn't
    have a standard lidar/odom modality, and this project's nav/SLAM evaluation use case
    wants those in their original form anyway -- keep using the .h5 file directly for that.

fps is *not* something this recording ever guaranteed: hdf5_collector_node fires on an
opportunistic ApproximateTimeSynchronizer match, not a fixed timer, so frame spacing has
real jitter (see verify_dataset.py's interval stats for a given file). We derive a single
constant fps from the mean interval, as LeRobotDataset requires, and print how much jitter
that's papering over so it isn't a silent assumption.
"""

import argparse
import sys

import h5py
import numpy as np


def build_task_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('h5_path', help='Path to a semih_session_*.h5 file from hdf5_collector_node')
    parser.add_argument('--repo-id', required=True, help='e.g. local/semih-arm-demo (not pushed to the Hub by this script)')
    parser.add_argument('--root', default=None, help='Output directory; defaults to ~/.cache/huggingface/lerobot/<repo-id>')
    parser.add_argument('--task', default='reach toward target', help='Single-task description stored per frame')
    parser.add_argument('--camera-key', default='front', help='Name used in observation.images.<key>')
    return parser


def load_h5(h5_path):
    with h5py.File(h5_path, 'r') as f:
        if 'camera/images' not in f:
            raise SystemExit(
                f"'{h5_path}' has no camera/images dataset (likely dropped at save time due to a "
                "shape mismatch -- see hdf5_collector_node.py save_and_shutdown). Cannot build a "
                "video-based LeRobotDataset without it."
            )
        images = f['camera/images'][:]  # (N, H, W, 3) uint8, BGR
        sync_timestamps = f['sync_timestamps'][:]
        joint_names = list(f.attrs.get('joint_names', []))
        positions = f['joint_states/positions'][:] if 'joint_states/positions' in f else None
        actions = f['action/arm_target_positions'][:] if 'action/arm_target_positions' in f else None
    return images, sync_timestamps, joint_names, positions, actions


def fill_actions(actions, positions):
    """Forward-fill NaN rows from the first valid action; fall back to the
    corresponding observation.state row (i.e. 'hold') if no action was ever
    commanded during the whole session."""
    actions = actions.copy()
    n = len(actions)
    valid = ~np.isnan(actions).any(axis=1)

    if not valid.any():
        print('WARNING: arm was never commanded during this session -- using observation.state as action for every frame.')
        return positions.copy()

    first_valid = np.argmax(valid)
    if first_valid > 0:
        print(f'INFO: first {first_valid} frame(s) predate the first arm command; forward-filling from frame {first_valid}.')
        actions[:first_valid] = actions[first_valid]

    for i in range(1, n):
        if not valid[i]:
            actions[i] = actions[i - 1]

    return actions


def main():
    args = build_task_arg_parser().parse_args()

    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError:
        raise SystemExit(
            "Could not import lerobot. This script must be run inside the dedicated venv "
            '(see the module docstring), not the ROS2 Python environment.'
        )

    images, sync_timestamps, joint_names, positions, actions = load_h5(args.h5_path)
    n = len(sync_timestamps)
    if positions is None:
        raise SystemExit("No joint_states/positions in this file -- can't build observation.state.")
    if actions is None:
        raise SystemExit(
            "No action/arm_target_positions in this file -- it was recorded before the action-logging "
            'fix was added to hdf5_collector_node.py. Re-record, or add --allow-no-action if you just '
            'want observation.state duplicated as the action (not implemented -- re-record instead).'
        )
    if not joint_names:
        joint_names = [f'arm_joint{i + 1}' for i in range(positions.shape[1])]

    intervals = np.diff(sync_timestamps)
    fps = round(1.0 / intervals.mean())
    jitter_ms = intervals.std() * 1000
    print(f'{n} frames, mean interval {intervals.mean() * 1000:.1f}ms (fps={fps}), '
          f'jitter std={jitter_ms:.1f}ms -- this is being flattened into a constant fps.')

    actions = fill_actions(actions, positions)
    images_rgb = images[..., ::-1]  # BGR -> RGB

    n_joints = positions.shape[1]
    features = {
        'observation.state': {'dtype': 'float32', 'shape': (n_joints,), 'names': joint_names},
        'action': {'dtype': 'float32', 'shape': (n_joints,), 'names': joint_names},
        f'observation.images.{args.camera_key}': {
            'dtype': 'video',
            'shape': images.shape[1:],
            'names': ['height', 'width', 'channels'],
        },
    }

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=fps,
        features=features,
        root=args.root,
        robot_type='semih',
        use_videos=True,
    )

    for i in range(n):
        dataset.add_frame({
            'observation.state': positions[i].astype(np.float32),
            'action': actions[i].astype(np.float32),
            f'observation.images.{args.camera_key}': images_rgb[i],
            'task': args.task,
        })

    dataset.save_episode()
    dataset.finalize()

    print(f'Wrote LeRobotDataset to: {dataset.root}')

    # Round-trip sanity check: load it back the same way training code would.
    reloaded = LeRobotDataset(repo_id=args.repo_id, root=dataset.root)
    print(f'Reloaded OK: {len(reloaded)} frames, features={list(reloaded.features.keys())}')
    sample = reloaded[0]
    print(f'Sample[0] keys: {list(sample.keys())}')


if __name__ == '__main__':
    sys.exit(main())
