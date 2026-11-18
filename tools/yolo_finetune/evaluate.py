"""Compare models on a held-out auto-labelled set (collect_dataset.py output, different seed).

usage: python3 evaluate.py <test_dir> <model.pt> [<model.pt> ...] [--conf 0.2]
For each labelled object (backpack/suitcase/chair) and each model: recall (a prediction of the
right class with IoU >= 0.5), what the best-overlapping prediction was called when it was missed,
and false positives (predictions of these classes that match no label of that class).
"""
import argparse
from collections import Counter, defaultdict
from pathlib import Path

from ultralytics import YOLO

CLS = {24: 'backpack', 28: 'suitcase', 56: 'chair'}


def iou(a, b):
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    i = max(0, x2 - x1) * max(0, y2 - y1)
    return i / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i + 1e-9)


def load_gt(p):
    out = []
    for line in p.read_text().splitlines():
        c, x, y, w, h = line.split()
        x, y, w, h = float(x) * 640, float(y) * 480, float(w) * 640, float(h) * 480
        out.append((int(c), (x - w / 2, y - h / 2, x + w / 2, y + h / 2)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('test')
    ap.add_argument('models', nargs='+')
    ap.add_argument('--conf', type=float, default=0.2)
    ap.add_argument('--min-area', type=float, default=1500, help='ignore labels smaller than this (px^2)')
    a = ap.parse_args()
    imgs = sorted((Path(a.test) / 'images').glob('*.jpg'))
    for mp in a.models:
        m = YOLO(mp)
        hit, tot, fp, fp_small = Counter(), Counter(), Counter(), Counter()
        missed_as = defaultdict(Counter)
        for p in imgs:
            all_gt = load_gt(Path(a.test) / 'labels' / f'{p.stem}.txt')
            gt = [g for g in all_gt if (g[1][2] - g[1][0]) * (g[1][3] - g[1][1]) >= a.min_area]
            small = [g for g in all_gt if g not in gt]
            r = m(str(p), conf=a.conf, verbose=False)[0]
            pred = [(int(b.cls), float(b.conf), b.xyxy[0].tolist()) for b in r.boxes]
            used = set()
            for c, box in gt:
                tot[c] += 1
                best = max(((iou(box, pb), i) for i, (pc, _, pb) in enumerate(pred) if pc == c), default=(0, -1))
                if best[0] >= 0.5:
                    hit[c] += 1
                    used.add(best[1])
                else:
                    ov = max(((iou(box, pb), m.names[pc]) for pc, _, pb in pred), default=(0, 'nothing'))
                    missed_as[c][ov[1] if ov[0] >= 0.3 else 'nothing'] += 1
            for i, (pc, _, pb) in enumerate(pred):
                if pc in CLS and i not in used:
                    # a hit on a label too small to be scored is not a false positive
                    if any(c == pc and iou(box, pb) >= 0.3 for c, box in small):
                        fp_small[pc] += 1
                    else:
                        fp[pc] += 1
        print(f'== {mp}  (conf >= {a.conf}, {len(imgs)} images, labels >= {a.min_area:.0f} px^2)')
        for c, n in CLS.items():
            print(f'  {n:9s} recall {hit[c]:4d}/{tot[c]:4d} = {hit[c] / max(tot[c], 1):.3f}   false pos {fp[c]:4d} (+{fp_small[c]} on unscored small labels)'
                  f'   missed as: {dict(missed_as[c].most_common(4))}')


if __name__ == '__main__':
    main()
