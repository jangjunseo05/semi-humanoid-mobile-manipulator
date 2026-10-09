#!/usr/bin/env python3
"""
Verification script for HDF5 datasets produced by hdf5_collector_node.

Checks:
  - Total number of synchronized frames
  - Timestamp interval statistics (mean, min, max, std) between consecutive
    synchronized frames, to spot frame drops / stalls
  - Per-topic timestamp drift relative to the synchronized frame timestamp
    (i.e. how far apart camera/lidar/joint_states/odom actually were within
    each "matched" set) -- large drift means the sync tolerance let through
    poorly-matched data

Usage:
    python3 verify_dataset.py /path/to/semih_session_XXXXXXXX_XXXXXX.h5
"""

import sys
import numpy as np
import h5py


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 verify_dataset.py <path_to_h5_file>")
        sys.exit(1)

    path = sys.argv[1]

    with h5py.File(path, 'r') as f:
        n_frames = f.attrs.get('num_frames', 'unknown')
        created = f.attrs.get('created', 'unknown')
        joint_names = f.attrs.get('joint_names', [])

        print(f"=== Dataset: {path} ===")
        print(f"Created: {created}")
        print(f"Declared frame count: {n_frames}")
        print(f"Joint names: {list(joint_names)}")
        print()

        sync_ts = f['sync_timestamps'][:]
        actual_n = len(sync_ts)
        print(f"Actual synchronized frames stored: {actual_n}")

        if actual_n < 2:
            print("Not enough frames to compute interval statistics.")
            return

        intervals = np.diff(sync_ts)
        print()
        print("--- Sync frame interval statistics (seconds) ---")
        print(f"  mean : {intervals.mean():.4f}")
        print(f"  std  : {intervals.std():.4f}")
        print(f"  min  : {intervals.min():.4f}")
        print(f"  max  : {intervals.max():.4f}")

        expected_interval = intervals.mean()
        drop_threshold = expected_interval * 2.0
        drops = np.where(intervals > drop_threshold)[0]
        if len(drops) > 0:
            print(f"\n  WARNING: {len(drops)} interval(s) more than 2x the average "
                  f"(> {drop_threshold:.3f}s) -- possible frame drops/stalls at indices: {list(drops)}")
        else:
            print("\n  No suspiciously large gaps detected (all intervals within 2x average).")

        print()
        print("--- Per-topic timestamp drift relative to sync frame timestamp ---")
        for topic in ['camera', 'lidar', 'joint_states', 'odom']:
            key = f'{topic}/timestamps'
            if key not in f:
                print(f"  {topic}: no timestamp dataset found, skipping.")
                continue
            topic_ts = f[key][:]
            if len(topic_ts) != actual_n:
                print(f"  {topic}: length mismatch ({len(topic_ts)} vs {actual_n} sync frames)")
                continue
            drift = np.abs(topic_ts - sync_ts)
            print(f"  {topic:14s} drift: mean={drift.mean()*1000:.1f}ms  max={drift.max()*1000:.1f}ms")

        print()
        print("--- Shape sanity check ---")
        for key in ['camera/images', 'lidar/ranges', 'joint_states/positions',
                    'odom/position', 'odom/orientation']:
            if key in f:
                print(f"  {key}: shape={f[key].shape}, dtype={f[key].dtype}")


if __name__ == '__main__':
    main()
