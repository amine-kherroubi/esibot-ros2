#!/usr/bin/env python3
"""
prepare_gtsrb.py — Conversion GTSRB → format YOLOv8
=====================================================
Utilise le CSV Train.csv (contient les ROI réels de chaque panneau).

Structure attendue (archive Kaggle GTSRB) :
  archive/
  ├── Train/
  │   ├── 0/  1/  2/ … 42/   ← images classées
  ├── Train.csv               ← Width,Height,Roi.X1,Roi.Y1,Roi.X2,Roi.Y2,ClassId,Path
  ├── Test/
  └── Test.csv

Résultat dans <output_dir>/dataset/ :
  dataset/
  ├── images/train/  val/
  ├── labels/train/  val/
  └── dataset.yaml

Usage :
  python3 prepare_gtsrb.py --src ~/Téléchargements/archive --out ~/esibot_ws/src/esibot_vision
"""

import argparse
import csv
import os
import random
import shutil
import yaml


# ── Classes retenues (GTSRB class_id → label local) ──────────────────────────
SELECTED = {
    1:  0,   # speed_30
    2:  1,   # speed_50
    4:  2,   # speed_70
    5:  3,   # speed_80
    14: 4,   # stop
    35: 5,   # dir_straight
    38: 6,   # dir_right
    39: 7,   # dir_left
}

NAMES = {
    0: "speed_30",
    1: "speed_50",
    2: "speed_70",
    3: "speed_80",
    4: "stop",
    5: "dir_straight",
    6: "dir_right",
    7: "dir_left",
}

VAL_RATIO = 0.15
RANDOM_SEED = 42


def main():
    parser = argparse.ArgumentParser(description="GTSRB → YOLOv8 dataset")
    parser.add_argument("--src", required=True,
                        help="Dossier racine archive GTSRB (contient Train.csv)")
    parser.add_argument("--out", required=True,
                        help="Dossier de sortie (dataset/ sera créé dedans)")
    args = parser.parse_args()

    src = os.path.expanduser(args.src)
    out = os.path.expanduser(args.out)
    ds  = os.path.join(out, "dataset")

    # ── Créer arborescence ────────────────────────────────────────────────
    for split in ("train", "val"):
        os.makedirs(os.path.join(ds, "images", split), exist_ok=True)
        os.makedirs(os.path.join(ds, "labels", split), exist_ok=True)

    print(f"[prepare_gtsrb] Source  : {src}")
    print(f"[prepare_gtsrb] Dataset : {ds}")

    # ── Lire Train.csv ────────────────────────────────────────────────────
    csv_path = os.path.join(src, "Train.csv")
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"Train.csv introuvable : {csv_path}")

    rows_by_class: dict[int, list] = {c: [] for c in SELECTED.values()}

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            gtsrb_id = int(row["ClassId"])
            if gtsrb_id not in SELECTED:
                continue

            img_path = os.path.join(src, row["Path"].replace("/", os.sep))
            if not os.path.isfile(img_path):
                continue

            w    = int(row["Width"])
            h    = int(row["Height"])
            x1   = int(row["Roi.X1"])
            y1   = int(row["Roi.Y1"])
            x2   = int(row["Roi.X2"])
            y2   = int(row["Roi.Y2"])

            # Normaliser en format YOLO (cx, cy, bw, bh)
            cx = ((x1 + x2) / 2.0) / w
            cy = ((y1 + y2) / 2.0) / h
            bw = (x2 - x1) / w
            bh = (y2 - y1) / h

            # Clamp
            cx = max(0.0, min(1.0, cx))
            cy = max(0.0, min(1.0, cy))
            bw = max(0.01, min(1.0, bw))
            bh = max(0.01, min(1.0, bh))

            label = SELECTED[gtsrb_id]
            rows_by_class[label].append((img_path, label, cx, cy, bw, bh))

    # ── Bilan par classe ──────────────────────────────────────────────────
    total = sum(len(v) for v in rows_by_class.values())
    print(f"\n[prepare_gtsrb] Images retenues : {total}")
    for lbl, rows in rows_by_class.items():
        print(f"  {NAMES[lbl]:15s} ({lbl}) : {len(rows)} images")

    # ── Split train / val ─────────────────────────────────────────────────
    random.seed(RANDOM_SEED)
    all_rows = []
    for rows in rows_by_class.values():
        all_rows.extend(rows)
    random.shuffle(all_rows)

    n_val = int(len(all_rows) * VAL_RATIO)
    val_rows   = all_rows[:n_val]
    train_rows = all_rows[n_val:]

    print(f"\n[prepare_gtsrb] Train : {len(train_rows)}  Val : {len(val_rows)}")

    # ── Copier images + labels ────────────────────────────────────────────
    counters = {"train": 0, "val": 0}

    for split, rows in (("train", train_rows), ("val", val_rows)):
        for img_path, label, cx, cy, bw, bh in rows:
            stem = os.path.splitext(os.path.basename(img_path))[0]
            # Éviter les collisions de noms
            new_name = f"{label}_{stem}"

            dst_img = os.path.join(ds, "images", split, new_name + ".png")
            dst_lbl = os.path.join(ds, "labels", split, new_name + ".txt")

            shutil.copy2(img_path, dst_img)
            with open(dst_lbl, "w") as f:
                f.write(f"{label} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")

            counters[split] += 1

    # ── dataset.yaml ─────────────────────────────────────────────────────
    yaml_path = os.path.join(ds, "dataset.yaml")
    dataset_cfg = {
        "path":  ds,
        "train": "images/train",
        "val":   "images/val",
        "nc":    len(NAMES),
        "names": [NAMES[i] for i in range(len(NAMES))],
    }
    with open(yaml_path, "w") as f:
        yaml.dump(dataset_cfg, f, default_flow_style=False, allow_unicode=True)

    print(f"\n[prepare_gtsrb] Dataset YAML : {yaml_path}")
    print(f"[prepare_gtsrb] Terminé — train:{counters['train']}  val:{counters['val']}")


if __name__ == "__main__":
    main()
