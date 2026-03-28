"""
sign_detector.py — Détection de panneaux avec YOLOv8n (finetuné GTSRB)
=======================================================================
Classes détectées (label index → nom) :
  0: speed_30  1: speed_50  2: speed_70  3: speed_80
  4: stop      5: dir_straight  6: dir_right  7: dir_left

Filtre temporel : une détection n'est confirmée que si elle apparaît
dans au moins `confirm_frames` frames consécutives → élimine le bruit.
"""

import os
from collections import defaultdict

import cv2
import numpy as np

from esibot_vision.config import SIGN_CLASSES, SIGN_COLORS, COLOR_WHITE


class SignDetector:

    def __init__(self,
                 model_path:     str,
                 conf_threshold: float = 0.60,
                 confirm_frames: int   = 3):
        """
        Parameters
        ----------
        model_path     : chemin vers le fichier .pt YOLOv8 finetuné
        conf_threshold : seuil de confiance minimum (0–1)
                         0.70 recommandé pour réduire les faux positifs
        confirm_frames : nombre de frames consécutives avant de confirmer
                         une détection (filtre anti-bruit)
        """
        self.conf_threshold = conf_threshold
        self.confirm_frames = confirm_frames
        self._model         = None

        # Compteur par classe : (frames_vues, frames_perdues)
        self._counters: dict[str, tuple[int, int]] = {}
        # Classes actuellement confirmées
        self._confirmed: set[str] = set()

        if model_path and os.path.isfile(model_path):
            self._load_model(model_path)
        else:
            import logging
            logging.getLogger(__name__).warning(
                f"[SignDetector] Modèle introuvable : {model_path} — détection désactivée")

    # ─────────────────────────────────────────────────────────────────────
    def _load_model(self, path: str):
        try:
            from ultralytics import YOLO
            self._model = YOLO(path)
            self._model.fuse()
        except ImportError:
            raise ImportError(
                "ultralytics non installé. Faites : pip install ultralytics")

    # ─────────────────────────────────────────────────────────────────────
    def detect(self, frame: np.ndarray, annotated: np.ndarray | None = None):
        """
        Détecte les panneaux avec filtre temporel anti-bruit.

        Returns
        -------
        confirmed  : liste de dicts (seulement les détections stables)
            [{'label': 'stop', 'conf': 0.87, 'bbox': (x1,y1,x2,y2)}, ...]
        annotated  : image annotée
        """
        if self._model is None:
            return [], annotated

        results = self._model.predict(
            frame,
            conf=self.conf_threshold,
            verbose=False,
            imgsz=320,
        )

        # Classes détectées dans cette frame
        detected_this_frame: dict[str, dict] = {}

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf   = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label  = SIGN_CLASSES.get(cls_id, f"cls{cls_id}")

                # Seuil spécifique par panneau
                min_conf = 0.85 if label == "stop" else self.conf_threshold
                if conf < min_conf:
                    continue

                # Garder la détection avec la meilleure confiance par classe
                if label not in detected_this_frame or \
                   conf > detected_this_frame[label]["conf"]:
                    detected_this_frame[label] = {
                        "label": label,
                        "conf":  round(conf, 2),
                        "bbox":  (x1, y1, x2, y2),
                    }

        # ── Mise à jour compteurs temporels (hystérésis) ─────────────────
        # - Pour CONFIRMER : confirm_frames détections consécutives
        # - Pour RETIRER   : remove_frames absences consécutives
        seen_labels = set(detected_this_frame.keys())
        all_labels  = set(self._counters.keys()) | seen_labels

        for label in all_labels:
            if label in seen_labels:
                # Détecté → incrémenter compteur présence, reset compteur absence
                cnt, lost = self._counters.get(label, (0, 0))
                self._counters[label] = (min(cnt + 1, self.confirm_frames), 0)
            else:
                # Absent → incrémenter compteur absence
                cnt, lost = self._counters.get(label, (0, 0))
                self._counters[label] = (cnt, lost + 1)

        # Confirmer si assez de détections consécutives
        # Retirer si absent depuis remove_frames frames
        remove_frames = 8
        new_confirmed = set()
        for lbl, (cnt, lost) in self._counters.items():
            if cnt >= self.confirm_frames:
                if lost < remove_frames:
                    new_confirmed.add(lbl)
        self._confirmed = new_confirmed

        # ── Construire liste de détections confirmées ─────────────────────
        confirmed = [
            detected_this_frame[lbl]
            for lbl in self._confirmed
            if lbl in detected_this_frame
        ]

        # ── Annotation ────────────────────────────────────────────────────
        if annotated is not None:
            for det in confirmed:
                label = det["label"]
                conf  = det["conf"]
                x1, y1, x2, y2 = det["bbox"]
                color = SIGN_COLORS.get(label, COLOR_WHITE)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                cv2.putText(annotated, f"{label} {conf:.0%}",
                            (x1, max(y1 - 5, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        return confirmed, annotated

    # ─────────────────────────────────────────────────────────────────────
    @property
    def available(self) -> bool:
        return self._model is not None
