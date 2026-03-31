"""
lane_detector.py - Dual-lane detection (black tape on white A4 paper)
=====================================================================
Filters:
  1. Minimum area
  2. Elongation  : minAreaRect ratio >= min_elongation
  3. BBox height : >= min_height_ratio * ROI height
  4. Selection   : lowest centroid (closest to robot)
  5. EMA smoothing: avoids abrupt jumps between frames (stable contour)
  6. Max jump    : if new centroid is too far from previous -> ignored
"""

import cv2
import numpy as np
from esibot_vision.config import COLOR_CYAN, COLOR_YELLOW, COLOR_GREEN, COLOR_GRAY

_MIN_ELONGATION = 2.5
_MIN_HEIGHT_RATIO = 0.10
_EMA_ALPHA = 0.35  # smoothing: 0=frozen, 1=no smoothing
_MAX_JUMP_PX = 60  # max allowed jump between 2 frames (px)


class LaneDetector:

    def __init__(
        self,
        roi_ratio: float = 1.0,
        threshold: int = 60,
        min_area: int = 200,
        min_elongation: float = _MIN_ELONGATION,
        min_height_ratio: float = _MIN_HEIGHT_RATIO,
    ):
        self.roi_ratio = roi_ratio
        self.threshold = threshold
        self.min_area = min_area
        self.min_elongation = min_elongation
        self.min_height_ratio = min_height_ratio

        # Smoothed centroids (EMA)
        self._left_smooth: float | None = None
        self._right_smooth: float | None = None
        self._left_inner_smooth: float | None = None
        self._right_inner_smooth: float | None = None

    # ─────────────────────────────────────────────────────────────────────
    def detect(self, frame: np.ndarray, annotated: np.ndarray | None = None):
        h, w = frame.shape[:2]
        roi_y = int(h * (1.0 - self.roi_ratio))
        roi = frame[roi_y:h, :]
        roi_h = roi.shape[0]

        # ── Binary mask ─────────────────────────────────────────────────
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY_INV)

        # OPEN (5,5): remove small noise
        # CLOSE (3,3): close small holes in the tape WITHOUT merging distant objects
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

        if annotated is not None and roi_y > 0:
            cv2.line(annotated, (w // 2, roi_y), (w // 2, h - 1), COLOR_GRAY, 1)

        mid_x = w // 2

        # ── Raw detection in each half ──────────────────────────────────
        raw_left, left_inner = self._find_raw_centroid(
            mask[:, :mid_x], roi_h, side="left"
        )
        raw_right, right_inner = self._find_raw_centroid(
            mask[:, mid_x:], roi_h, side="right"
        )

        # x_offset for full-image coordinates
        left_raw_cx = (raw_left + 0) if raw_left is not None else None
        right_raw_cx = (raw_right + mid_x) if raw_right is not None else None
        # Inner edges (for obstacle filter)
        left_inner_x = (left_inner + 0) if left_inner is not None else None
        right_inner_x = (right_inner + mid_x) if right_inner is not None else None

        # ── EMA smoothing + max-jump filter ─────────────────────────────
        left_cx = self._smooth(left_raw_cx, "_left_smooth", w)
        right_cx = self._smooth(right_raw_cx, "_right_smooth", w)
        # Smoothed inner edges (used by obstacle_detector)
        left_inner_cx = self._smooth(left_inner_x, "_left_inner_smooth", w)
        right_inner_cx = self._smooth(right_inner_x, "_right_inner_smooth", w)

        # ── Annotation ────────────────────────────────────────────────────
        if annotated is not None:
            self._draw_tape(mask[:, :mid_x], annotated, roi_y, 0, left_cx)
            self._draw_tape(mask[:, mid_x:], annotated, roi_y, mid_x, right_cx)

        # ── Compute error and status ─────────────────────────────────────
        if left_cx is not None and right_cx is not None:
            lane_mid = (left_cx + right_cx) / 2.0
            lane_error = (lane_mid - mid_x) / mid_x
            lane_status = "IN_LANE"
            if annotated is not None:
                line_len = (h - roi_y) // 4
                cv2.line(
                    annotated,
                    (int(lane_mid), h - 1),
                    (int(lane_mid), h - 1 - line_len),
                    COLOR_GREEN,
                    2,
                )
        elif left_cx is not None:
            lane_error = (left_cx - mid_x) / mid_x
            lane_status = "LANE_RIGHT"
        elif right_cx is not None:
            lane_error = (right_cx - mid_x) / mid_x
            lane_status = "LANE_LEFT"
        else:
            lane_error = 0.0
            lane_status = "NO_LANE"

        return lane_error, lane_status, annotated, left_inner_cx, right_inner_cx

    # ─────────────────────────────────────────────────────────────────────
    def _smooth(self, raw_cx: float | None, attr: str, w: int) -> float | None:
        """Apply EMA + max-jump filter on a centroid."""
        prev = getattr(self, attr)

        if raw_cx is None:
            # No detection this frame -> keep the previous value
            if prev is not None:
                # Keep the last known value (robot continues by inertia)
                pass
            return prev

        if prev is None:
            # First detection
            setattr(self, attr, raw_cx)
            return raw_cx

        # Max-jump filter
        if abs(raw_cx - prev) > _MAX_JUMP_PX:
            return prev  # ignore this jump -> keep previous value

        # EMA
        smoothed = _EMA_ALPHA * raw_cx + (1.0 - _EMA_ALPHA) * prev
        setattr(self, attr, smoothed)
        return smoothed

    # ─────────────────────────────────────────────────────────────────────
    def _find_raw_centroid(
        self, half_mask: np.ndarray, roi_h: int, side: str = "left"
    ) -> tuple[float | None, float | None]:
        """
        Return (centroid_x, inner_edge_x) of the best tape contour.
        inner_edge = right edge of left tape / left edge of right tape.
        """
        cnts, _ = cv2.findContours(
            half_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not cnts:
            return None, None

        candidates = [
            c
            for c in cnts
            if cv2.contourArea(c) >= self.min_area and self._is_tape(c, roi_h)
        ]
        if not candidates:
            return None, None

        def cy_of(c):
            M = cv2.moments(c)
            return M["m01"] / M["m00"] if M["m00"] != 0 else 0

        c = max(candidates, key=cy_of)
        M = cv2.moments(c)
        if M["m00"] == 0:
            return None, None

        cx = float(M["m10"] / M["m00"])
        x, y, bw, bh = cv2.boundingRect(c)

        # Inner edge: right side for left tape, left side for right tape
        if side == "left":
            inner = float(x + bw)  # right edge of left tape
        else:
            inner = float(x)  # left edge of right tape

        return cx, inner

    # ─────────────────────────────────────────────────────────────────────
    def _is_tape(self, contour: np.ndarray, roi_h: int) -> bool:
        rect = cv2.minAreaRect(contour)
        _, (rw, rh), _ = rect
        if rw == 0 or rh == 0:
            return False
        if max(rw, rh) / min(rw, rh) < self.min_elongation:
            return False
        _, _, _, bh = cv2.boundingRect(contour)
        if bh < roi_h * self.min_height_ratio:
            return False
        return True

    # ─────────────────────────────────────────────────────────────────────
    def _draw_tape(self, half_mask, annotated, roi_y, x_offset, smoothed_cx):
        """Draw the tape contour and the smoothed centroid."""
        cnts, _ = cv2.findContours(
            half_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not cnts:
            return
        roi_h_local = annotated.shape[0] - roi_y if annotated is not None else 100
        candidates = [
            c
            for c in cnts
            if cv2.contourArea(c) >= self.min_area and self._is_tape(c, roi_h_local)
        ]
        if not candidates:
            return

        def cy_of(c):
            M = cv2.moments(c)
            return M["m01"] / M["m00"] if M["m00"] != 0 else 0

        c = max(candidates, key=cy_of)
        c_shifted = c.copy()
        c_shifted[:, :, 0] += x_offset
        cv2.drawContours(annotated[roi_y:], [c_shifted], -1, COLOR_YELLOW, 2)

        # Smoothed centroid point
        if smoothed_cx is not None:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cy = int(M["m01"] / M["m00"])
                cv2.circle(
                    annotated, (int(smoothed_cx), roi_y + cy), 6, COLOR_YELLOW, -1
                )
