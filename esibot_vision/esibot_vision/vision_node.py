#!/usr/bin/env python3
"""
vision_node.py — EsiBot vision temps réel
==========================================
Caméra ESP32-CAM inclinée 30-45° vers le bas, hauteur ~15-20cm.
  - Partie HAUTE de l'image → devant le robot (obstacles, murs)
  - Partie BASSE de l'image → sol devant le robot (ligne)

Topics souscrits :
  /camera/image_raw           sensor_msgs/Image

Topics publiés :
  /vision/line_position       std_msgs/Float32   [-1.0 gauche … 0.0 centre … +1.0 droite]
  /vision/obstacle_detected   std_msgs/Bool
  /vision/detections          std_msgs/String    (JSON résumé)
  /camera/image_annotated     sensor_msgs/Image  (si publish_annotated=True)
"""

import json
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String


# ─── Constantes couleurs annotation ──────────────────────────────────────────
COLOR_CYAN   = (255, 255,   0)   # ROI ligne
COLOR_GRAY   = (160, 160, 160)   # ROI obstacle
COLOR_YELLOW = (  0, 255, 255)   # contour ligne + centroïde
COLOR_GREEN  = (  0, 255,   0)   # DETECTE
COLOR_ORANGE = (  0, 165, 255)   # PROCHE
COLOR_RED    = (  0,   0, 255)   # TRES_PROCHE
COLOR_WHITE  = (255, 255, 255)   # texte


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        # ── Paramètres image ──────────────────────────────────────────────
        self.declare_parameter('image_width',  320)
        self.declare_parameter('image_height', 240)

        # ── Paramètres ligne ──────────────────────────────────────────────
        self.declare_parameter('line_roi_ratio',  0.55)
        self.declare_parameter('line_threshold',  60)
        self.declare_parameter('line_color',      'dark')

        # ── Paramètres obstacle ───────────────────────────────────────────
        self.declare_parameter('obstacle_roi_ratio', 0.40)
        self.declare_parameter('obstacle_min_area',  1500)

        # ── Publication image annotée ─────────────────────────────────────
        self.declare_parameter('publish_annotated', True)

        self._load_params()

        # ── État interne thread-safe ───────────────────────────────────────
        self._frame           = None
        self._frame_lock      = threading.Lock()
        self._last_frame_time = time.time()

        # ── Subscriber ────────────────────────────────────────────────────
        self.create_subscription(Image, '/camera/image_raw', self._cb_image, 10)

        # ── Publishers ────────────────────────────────────────────────────
        self._pub_line       = self.create_publisher(Float32, '/vision/line_position',     10)
        self._pub_obstacle   = self.create_publisher(Bool,    '/vision/obstacle_detected', 10)
        self._pub_detections = self.create_publisher(String,  '/vision/detections',        10)
        self._pub_annotated  = self.create_publisher(Image,   '/camera/image_annotated',   10)

        # ── Timer traitement 10 Hz ────────────────────────────────────────
        self.create_timer(1.0 / 30.0, self._process)  # 30 Hz max

        self._fps_counter = 0
        self._fps_time    = time.time()
        self._fps         = 0.0

        self.get_logger().info('vision_node démarré — 10 Hz')

    # ─────────────────────────────────────────────────────────────────────
    def _load_params(self):
        self.img_w  = self.get_parameter('image_width').value
        self.img_h  = self.get_parameter('image_height').value

        self.line_roi_ratio  = float(self.get_parameter('line_roi_ratio').value)
        self.line_threshold  = int(self.get_parameter('line_threshold').value)
        self.line_color      = self.get_parameter('line_color').value

        self.obstacle_roi_ratio = float(self.get_parameter('obstacle_roi_ratio').value)
        self.obstacle_min_area  = int(self.get_parameter('obstacle_min_area').value)

        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)

    # ─────────────────────────────────────────────────────────────────────
    def _cb_image(self, msg: Image):
        """Réception image — conversion et stockage thread-safe."""
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3)).copy()
        with self._frame_lock:
            self._frame = frame
            self._last_frame_time = time.time()

    # ─────────────────────────────────────────────────────────────────────
    def _process(self):
        """Traitement principal appelé par le timer 10 Hz."""

        if time.time() - self._last_frame_time > 5.0 and self._frame is not None:
            self.get_logger().warn('Aucune frame depuis 5s — flux caméra interrompu ?')

        with self._frame_lock:
            if self._frame is None:
                return
            frame = self._frame.copy()

        h, w = frame.shape[:2]

        # ── Calcul FPS ────────────────────────────────────────────────────
        self._fps_counter += 1
        now = time.time()
        if now - self._fps_time >= 1.0:
            self._fps = self._fps_counter / (now - self._fps_time)
            self._fps_counter = 0
            self._fps_time = now

        annotated = frame.copy() if self.publish_annotated else None

        # ── ROI ligne : bas de l'image ────────────────────────────────────
        line_roi_y = int(h * (1.0 - self.line_roi_ratio))
        # ── ROI obstacle : haut de l'image ───────────────────────────────
        obs_roi_y  = int(h * self.obstacle_roi_ratio)

        if annotated is not None:
            cv2.rectangle(annotated, (0, line_roi_y), (w - 1, h - 1), COLOR_CYAN, 1)
            cv2.rectangle(annotated, (0, 0), (w - 1, obs_roi_y), COLOR_GRAY, 1)

        # ── A) Détection ligne ────────────────────────────────────────────
        line_position, line_detected = self._detect_line(
            frame, h, w, line_roi_y, annotated)

        # ── B) Détection obstacles ────────────────────────────────────────
        obstacle_detected, obstacles, annotated = self._detect_obstacle(
            frame, annotated)

        # ── Publication topics ────────────────────────────────────────────
        msg_line = Float32()
        msg_line.data = float(line_position)
        self._pub_line.publish(msg_line)

        msg_obs = Bool()
        msg_obs.data = obstacle_detected
        self._pub_obstacle.publish(msg_obs)

        proximity_rank  = {'TRES_PROCHE': 2, 'PROCHE': 1, 'DETECTE': 0}
        worst_proximity = max(obstacles, key=lambda o: proximity_rank[o['proximity']])['proximity'] \
            if obstacles else ''

        detections = {
            'line_position':      round(line_position, 3),
            'line_detected':      line_detected,
            'obstacle_detected':  obstacle_detected,
            'obstacle_count':     len(obstacles),
            'obstacle_proximity': worst_proximity,
            'obstacles':          [{'proximity': o['proximity'], 'area': o['area']} for o in obstacles],
        }
        msg_det = String()
        msg_det.data = json.dumps(detections)
        self._pub_detections.publish(msg_det)

        # ── Logs ──────────────────────────────────────────────────────────
        if line_detected:
            self.get_logger().info(
                f'Ligne : position={line_position:+.2f}', throttle_duration_sec=1.0)
        if obstacle_detected:
            self.get_logger().warn(
                f'{len(obstacles)} obstacle(s) — pire proximité : {worst_proximity}',
                throttle_duration_sec=0.5)

        # ── Image annotée ─────────────────────────────────────────────────
        if self.publish_annotated and annotated is not None:
            self._draw_overlay(annotated, h, w, line_position, line_detected,
                               obstacle_detected, worst_proximity)
            self._publish_image(annotated)

    # ─────────────────────────────────────────────────────────────────────
    def _detect_line(self, frame, h, w, roi_y, annotated):
        """
        Détection ligne noire sur fond clair.
        ROI = ligne_roi_y … bas de l'image.
        Retourne (position_normalisée, ligne_détectée).
        """
        roi  = frame[roi_y:h, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        if self.line_color == 'dark':
            _, mask = cv2.threshold(
                gray, self.line_threshold, 255, cv2.THRESH_BINARY_INV)
        else:
            _, mask = cv2.threshold(
                gray, self.line_threshold, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return 0.0, False

        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < 200:
            return 0.0, False

        M = cv2.moments(c)
        if M['m00'] == 0:
            return 0.0, False

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        position = (cx - w / 2.0) / (w / 2.0)

        if annotated is not None:
            cv2.drawContours(annotated[roi_y:], [c], -1, COLOR_YELLOW, 2)
            cv2.circle(annotated, (cx, roi_y + cy), 7, COLOR_YELLOW, -1)

        return position, True

    # ─────────────────────────────────────────────────────────────────────
    def _detect_obstacle(self, frame, annotated):
        """
        Détection de TOUS les obstacles dans les 40% haut de l'image.

        Stratégie : construire UN masque binaire combiné (Canny rempli + HSV),
        puis appeler findContours UNE seule fois → pas de doublons, pas de fusion
        parasite due à un grand kernel morphologique.

        Retourne (obs_ok, obstacles, annotated) où obstacles est une liste de dicts :
          [{'rect': (x,y,w,h), 'proximity': 'PROCHE', 'area': 1234}, ...]
        """
        h, w = frame.shape[:2]
        roi_y = int(h * self.obstacle_roi_ratio)
        roi   = frame[0:roi_y, :]
        total_area = roi_y * w

        # ── Méthode 1 : Canny → remplir les régions fermées ──────────────
        gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=2)
        mask_canny = np.zeros_like(gray)
        cnts_tmp, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(mask_canny, cnts_tmp, -1, 255, cv2.FILLED)

        # ── Méthode 2 : HSV couleurs vives (rouge + orange) ───────────────
        hsv     = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_r1 = cv2.inRange(hsv, np.array([0,   120, 70]), np.array([10,  255, 255]))
        mask_r2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        mask_o  = cv2.inRange(hsv, np.array([10,  120, 70]), np.array([25,  255, 255]))
        mask_hsv = cv2.bitwise_or(cv2.bitwise_or(mask_r1, mask_r2), mask_o)
        # Kernel (3,3) : referme les trous INTERNES à un objet sans ponter 2 objets voisins
        mask_hsv = cv2.morphologyEx(mask_hsv, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

        # ── Masque combiné ────────────────────────────────────────────────
        mask_combined = cv2.bitwise_or(mask_canny, mask_hsv)
        # MORPH_OPEN (pas CLOSE) : supprime le bruit isolé SANS combler les espaces
        # entre deux objets proches → ils restent deux contours séparés
        mask_combined = cv2.morphologyEx(
            mask_combined, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        cnts, _ = cv2.findContours(
            mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        obstacles = []

        for c in cnts:
            area = cv2.contourArea(c)
            if area < self.obstacle_min_area:
                continue

            # ── Filtre solidité : rejette les contours fragmentés (bruit Canny) ──
            hull      = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            if hull_area == 0:
                continue
            if (area / hull_area) < 0.35:
                continue

            ratio = area / total_area

            if ratio > 0.25:
                proximity = 'TRES_PROCHE'
                color = COLOR_RED
                self.get_logger().warn(
                    'Obstacle TRES_PROCHE !', throttle_duration_sec=0.5)
            elif ratio > 0.10:
                proximity = 'PROCHE'
                color = COLOR_ORANGE
                self.get_logger().warn(
                    'Obstacle PROCHE !', throttle_duration_sec=0.5)
            else:
                proximity = 'DETECTE'
                color = COLOR_GREEN

            rect = cv2.boundingRect(c)
            obstacles.append({'rect': rect, 'proximity': proximity, 'area': int(area)})

            if annotated is not None:
                # minAreaRect → rectangle orienté au plus près de l'objet réel
                min_rect = cv2.minAreaRect(c)
                box = np.intp(cv2.boxPoints(min_rect))
                cv2.drawContours(annotated, [box], -1, color, 2)
                x, y = box[:, 0].min(), box[:, 1].min()
                cv2.putText(annotated, proximity, (x, max(y - 5, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        obs_ok = len(obstacles) > 0
        return obs_ok, obstacles, annotated

    # ─────────────────────────────────────────────────────────────────────
    def _draw_overlay(self, img, h, w, line_pos, line_det, obs_det, proximity):
        """Dessine FPS + statuts sur l'image annotée."""
        cv2.putText(img, f'FPS:{self._fps:.1f}',
                    (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_WHITE, 1)

        line_txt = f'Ligne:{line_pos:+.2f}' if line_det else 'Ligne:--'
        cv2.putText(img, line_txt,
                    (5, h - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_CYAN, 1)

        obs_txt = f'Obs:{proximity}' if obs_det else 'Obs:aucun'
        cv2.putText(img, obs_txt,
                    (5, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    COLOR_RED if obs_det else COLOR_WHITE, 1)

    # ─────────────────────────────────────────────────────────────────────
    def _publish_image(self, img):
        """Publie un tableau numpy BGR comme sensor_msgs/Image."""
        h, w = img.shape[:2]
        msg = Image()
        msg.height       = h
        msg.width        = w
        msg.encoding     = 'bgr8'
        msg.is_bigendian = False
        msg.step         = w * 3
        msg.data         = img.tobytes()
        self._pub_annotated.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
