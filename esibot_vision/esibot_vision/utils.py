"""
utils.py — FPSCounter + draw_hud
"""
import time
import cv2
from esibot_vision.config import COLOR_WHITE, COLOR_GREEN, COLOR_RED, COLOR_CYAN, COLOR_ORANGE


class FPSCounter:
    """Compteur FPS glissant sur 1 seconde."""

    def __init__(self):
        self._count = 0
        self._t0    = time.time()
        self.fps    = 0.0

    def tick(self):
        self._count += 1
        now = time.time()
        elapsed = now - self._t0
        if elapsed >= 1.0:
            self.fps    = self._count / elapsed
            self._count = 0
            self._t0    = now

    def get(self) -> float:
        return self.fps


def draw_hud(img, fps: float, lane_error: float, lane_status: str,
             sign_labels: list, obstacle_in_lane: bool):
    """
    Dessine le HUD (FPS, état voie, panneaux, obstacles) sur l'image annotée.

    Parameters
    ----------
    img            : tableau numpy BGR (modifié en place)
    fps            : FPS courant
    lane_error     : erreur normalisée [-1 … +1]
    lane_status    : "IN_LANE" | "LANE_LEFT" | "LANE_RIGHT" | "NO_LANE"
    sign_labels    : liste de str (ex. ["stop", "speed_50"])
    obstacle_in_lane : True si un obstacle bloque la voie
    """
    h, w = img.shape[:2]

    # ── FPS ──────────────────────────────────────────────────────────────
    cv2.putText(img, f"FPS:{fps:.1f}",
                (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_WHITE, 1)

    # ── Lane ─────────────────────────────────────────────────────────────
    status_color = COLOR_GREEN if lane_status == "IN_LANE" else COLOR_ORANGE
    if lane_status == "NO_LANE":
        status_color = COLOR_RED
    lane_txt = f"Lane:{lane_status}  err:{lane_error:+.2f}"
    cv2.putText(img, lane_txt,
                (5, h - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.40, status_color, 1)

    # ── Panneaux détectés ────────────────────────────────────────────────
    signs_txt = "Signs:" + (",".join(sign_labels) if sign_labels else "none")
    cv2.putText(img, signs_txt,
                (5, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.40, COLOR_CYAN, 1)

    # ── Obstacle ─────────────────────────────────────────────────────────
    if obstacle_in_lane:
        cv2.putText(img, "!! OBSTACLE IN LANE !!",
                    (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.35, COLOR_RED, 1)
