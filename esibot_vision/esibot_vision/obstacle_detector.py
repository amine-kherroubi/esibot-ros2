"""
obstacle_detector.py — Détection d'obstacles avec YOLOv8n (COCO)
==================================================================
Filtre temporel (hystérésis) :
  - Confirmer : CONFIRM_FRAMES détections consécutives
  - Retirer   : REMOVE_FRAMES absences consécutives
  → élimine le clignotement
"""

import os
import cv2
import numpy as np

from esibot_vision.config import COLOR_RED, COLOR_ORANGE, COLOR_GREEN, COLOR_WHITE


_IGNORE_CLASSES = {
    60,  # dining table
    63,  # laptop
    64,  # mouse
    65,  # remote
    66,  # keyboard
    67,  # cell phone
    72,  # refrigerator
}

_MIN_WIDTH_RATIO = 0.05
_CONFIRM_FRAMES  = 4    # frames consécutives pour confirmer
_REMOVE_FRAMES   = 10   # frames absentes pour retirer


class ObstacleDetector:

    def __init__(self,
                 model_path:      str,
                 conf_threshold:  float = 0.45,
                 roi_ratio:       float = 0.55,
                 lane_width_ratio: float = 0.50):
        self.conf_threshold   = conf_threshold
        self.roi_ratio        = roi_ratio
        self.lane_width_ratio = lane_width_ratio
        self._model = None

        # Filtre temporel : {label: (frames_vues, frames_perdues, last_det)}
        self._counters: dict[str, tuple[int, int, dict]] = {}
        self._confirmed: dict[str, dict] = {}

        if model_path and os.path.isfile(model_path):
            self._load_model(model_path)
        else:
            import logging
            logging.getLogger(__name__).warning(
                f"[ObstacleDetector] Modèle introuvable : {model_path} — détection désactivée")

    # ─────────────────────────────────────────────────────────────────────
    def _load_model(self, path: str):
        try:
            from ultralytics import YOLO
            self._model = YOLO(path)
            self._model.fuse()
        except ImportError:
            raise ImportError("ultralytics non installé.")

    # ─────────────────────────────────────────────────────────────────────
    def detect(self, frame: np.ndarray, annotated: np.ndarray | None = None,
               lane_left_cx: float | None = None, lane_right_cx: float | None = None):

        if self._model is None:
            return [], False, annotated

        h, w = frame.shape[:2]

        # Zone de voie
        if lane_left_cx is not None and lane_right_cx is not None:
            lane_x1 = int(lane_left_cx)
            lane_x2 = int(lane_right_cx)
        else:
            margin  = int(w * (1.0 - self.lane_width_ratio) / 2)
            lane_x1 = margin
            lane_x2 = w - margin

        results = self._model.predict(
            frame, conf=self.conf_threshold, verbose=False, imgsz=320)

        # Détections brutes cette frame (par label, meilleure conf)
        raw: dict[str, dict] = {}
        total_area = h * w

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                if cls_id in _IGNORE_CLASSES:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])

                if (x2 - x1) < w * _MIN_WIDTH_RATIO:
                    continue

                in_lane = not (x2 < lane_x1 or x1 > lane_x2)
                if not in_lane:
                    continue

                area  = (x2 - x1) * (y2 - y1)
                ratio = area / total_area
                if ratio > 0.25:
                    proximity, color = "TRES_PROCHE", COLOR_RED
                elif ratio > 0.08:
                    proximity, color = "PROCHE",      COLOR_ORANGE
                else:
                    proximity, color = "DETECTE",     COLOR_WHITE

                label = self._model.names.get(cls_id, f"cls{cls_id}") \
                    if hasattr(self._model, "names") else f"cls{cls_id}"

                det = {"label": label, "conf": round(conf, 2),
                       "bbox": (x1, y1, x2, y2), "in_lane": True,
                       "proximity": proximity, "color": color}

                if label not in raw or conf > raw[label]["conf"]:
                    raw[label] = det

        # ── Filtre temporel (hystérésis) ──────────────────────────────────
        all_labels = set(self._counters.keys()) | set(raw.keys())
        for label in all_labels:
            seen, lost, last = self._counters.get(label, (0, 0, {}))
            if label in raw:
                self._counters[label] = (min(seen + 1, _CONFIRM_FRAMES), 0, raw[label])
            else:
                self._counters[label] = (seen, lost + 1, last)

        # Mettre à jour confirmés
        new_confirmed = {}
        for lbl, (seen, lost, last) in self._counters.items():
            if seen >= _CONFIRM_FRAMES and lost < _REMOVE_FRAMES:
                new_confirmed[lbl] = last
        self._confirmed = new_confirmed

        # ── Annotation ────────────────────────────────────────────────────
        if annotated is not None:
            for det in self._confirmed.values():
                x1, y1, x2, y2 = det["bbox"]
                color = det["color"]
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                cv2.putText(annotated, f"{det['label']} {det['proximity']}",
                            (x1, max(y1 - 5, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, color, 1)

        obstacles = list(self._confirmed.values())
        obstacle_in_lane = len(obstacles) > 0
        return obstacles, obstacle_in_lane, annotated

    # ─────────────────────────────────────────────────────────────────────
    @property
    def available(self) -> bool:
        return self._model is not None
