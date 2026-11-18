"""Fine-tune YOLOv8n on the auto-labelled Gazebo dataset (collect_dataset.py output).

usage: python3 train.py <raw_train_dir> <work_dir> [--epochs 60] [--base yolov8n.pt]
Keeps all 80 COCO class names (labels use COCO ids: backpack 24, suitcase 28, chair 56) so the
node's class names and `target_classes` filter keep working unchanged. Only these three classes
have training data here, so other COCO classes are not maintained by this fine-tune.
Writes <work_dir>/data.yaml, the train/val split, and the run under <work_dir>/runs/.
"""
import argparse
import random
import shutil
from pathlib import Path

from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('raw')
    ap.add_argument('work')
    ap.add_argument('--epochs', type=int, default=60)
    ap.add_argument('--base', default=str(Path.home() / 'semiH_ws' / 'yolov8n.pt'))
    ap.add_argument('--val-frac', type=float, default=0.15)
    a = ap.parse_args()
    raw, work = Path(a.raw), Path(a.work)
    stems = sorted(p.stem for p in (raw / 'images').glob('*.jpg'))
    random.Random(0).shuffle(stems)
    n_val = int(len(stems) * a.val_frac)
    for split, part in (('val', stems[:n_val]), ('train', stems[n_val:])):
        for sub in ('images', 'labels'):
            d = work / split / sub
            if d.exists():
                shutil.rmtree(d)
            d.mkdir(parents=True)
        for s in part:
            shutil.copy(raw / 'images' / f'{s}.jpg', work / split / 'images')
            shutil.copy(raw / 'labels' / f'{s}.txt', work / split / 'labels')
    names = YOLO(a.base).names
    yaml = work / 'data.yaml'
    yaml.write_text(f'path: {work}\ntrain: train/images\nval: val/images\nnames:\n'
                    + ''.join(f'  {i}: {names[i]}\n' for i in range(len(names))))
    print(f'train {len(stems) - n_val}, val {n_val}')
    YOLO(a.base).train(data=str(yaml), epochs=a.epochs, imgsz=640, batch=32, device=0, seed=0,
                       project=str(work / 'runs'), name='ft', exist_ok=True, patience=20, plots=True)


if __name__ == '__main__':
    main()
