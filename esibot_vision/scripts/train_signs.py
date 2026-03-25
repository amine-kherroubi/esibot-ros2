#!/usr/bin/env python3
"""
train_signs.py — Entraînement YOLOv8n sur dataset GTSRB EsiBot
===============================================================
Lance prepare_gtsrb.py si nécessaire, puis entraîne YOLOv8n.

Usage :
  python3 train_signs.py \
      --dataset ~/esibot_ws/src/esibot_vision/dataset/dataset.yaml \
      --output  ~/esibot_ws/src/esibot_vision/models \
      --epochs  50 \
      --batch   16 \
      --imgsz   128

Modèle sauvegardé dans <output>/signs_best.pt
"""

import argparse
import os
import shutil


def main():
    parser = argparse.ArgumentParser(description="Entraîne YOLOv8n sur GTSRB EsiBot")
    parser.add_argument("--dataset", required=True,
                        help="Chemin vers dataset.yaml (généré par prepare_gtsrb.py)")
    parser.add_argument("--output", required=True,
                        help="Dossier de destination pour signs_best.pt")
    parser.add_argument("--epochs",  type=int,   default=50)
    parser.add_argument("--batch",   type=int,   default=16)
    parser.add_argument("--imgsz",   type=int,   default=128,
                        help="Taille images (64|128|160|320). 128 recommandé pour ESP32-CAM")
    parser.add_argument("--workers", type=int,   default=4)
    parser.add_argument("--device",  type=str,   default="",
                        help="Device torch : '' (auto), 'cpu', '0' (GPU 0)")
    args = parser.parse_args()

    dataset = os.path.expanduser(args.dataset)
    output  = os.path.expanduser(args.output)

    if not os.path.isfile(dataset):
        raise FileNotFoundError(
            f"dataset.yaml introuvable : {dataset}\n"
            f"Lancez d'abord prepare_gtsrb.py")

    os.makedirs(output, exist_ok=True)

    # ── Import ultralytics ────────────────────────────────────────────────
    try:
        from ultralytics import YOLO
    except ImportError:
        raise ImportError("Installez ultralytics : pip install ultralytics")

    print(f"[train_signs] Dataset  : {dataset}")
    print(f"[train_signs] Output   : {output}")
    print(f"[train_signs] Epochs   : {args.epochs}")
    print(f"[train_signs] Batch    : {args.batch}")
    print(f"[train_signs] Img size : {args.imgsz}")

    # ── Charger YOLOv8n pré-entraîné ─────────────────────────────────────
    model = YOLO("yolov8n.pt")

    # ── Entraînement ──────────────────────────────────────────────────────
    train_args = dict(
        data     = dataset,
        epochs   = args.epochs,
        batch    = args.batch,
        imgsz    = args.imgsz,
        workers  = args.workers,
        project  = os.path.join(output, "runs"),
        name     = "signs",
        exist_ok = True,
        val      = True,
        plots    = True,
        verbose  = True,
    )
    if args.device:
        train_args["device"] = args.device

    results = model.train(**train_args)

    # ── Copier best.pt vers models/signs_best.pt ──────────────────────────
    best_src = os.path.join(output, "runs", "signs", "weights", "best.pt")
    best_dst = os.path.join(output, "signs_best.pt")

    if os.path.isfile(best_src):
        shutil.copy2(best_src, best_dst)
        print(f"\n[train_signs] Modèle sauvegardé : {best_dst}")
    else:
        print(f"\n[train_signs] ATTENTION : best.pt introuvable à {best_src}")
        print(f"             Cherchez manuellement dans {output}/runs/signs/weights/")

    print("[train_signs] Entraînement terminé.")


if __name__ == "__main__":
    main()
