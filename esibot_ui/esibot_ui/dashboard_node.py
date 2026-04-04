"""
dashboard_node.py — EsiBot UI
==============================
Nœud ROS2 Python embarqué sur le Raspberry Pi.
Sert les fichiers statiques du dashboard web (React buildé) via HTTP.

L'utilisateur ouvre http://<robot-ip>:8080 depuis son navigateur.
Le dashboard se connecte ensuite à rosbridge sur ws://<robot-ip>:9090.

Usage :
  ros2 run esibot_ui dashboard_node
  ros2 launch esibot_ui dashboard.launch.py
"""

import os
import threading
import functools
import http.server
import socket

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class DashboardNode(Node):
    """Nœud ROS2 servant le dashboard web EsiBot via HTTP."""

    def __init__(self):
        super().__init__('dashboard_node')

        # ── Paramètres ──────────────────────────────────────────────────────
        self.declare_parameter('port',     8080)
        self.declare_parameter('host',     '0.0.0.0')
        self.declare_parameter('web_root', '')

        port     = self.get_parameter('port').value
        host     = self.get_parameter('host').value
        web_root = self.get_parameter('web_root').value

        # Dossier web/ dans share du package (fichiers React buildés)
        if not web_root:
            pkg_share = get_package_share_directory('esibot_ui')
            web_root  = os.path.join(pkg_share, 'web')

        if not os.path.isdir(web_root):
            self.get_logger().error(
                f"Dossier web introuvable : {web_root}\n"
                "Veuillez builder le dashboard : cd web_src && npm install && npm run build\n"
                "Puis copier dist/ → web/ et rebuilder le package."
            )
            return

        # ── Serveur HTTP dans un thread séparé ──────────────────────────────
        handler = functools.partial(
            http.server.SimpleHTTPRequestHandler,
            directory=web_root
        )
        # Désactiver les logs HTTP dans le terminal ROS
        handler.log_message = lambda *args: None

        self._server = http.server.HTTPServer((host, port), handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            daemon=True,
            name='dashboard_http'
        )
        self._thread.start()

        # ── Afficher l'IP locale ─────────────────────────────────────────────
        local_ip = self._get_local_ip()

        self.get_logger().info(
            f'\n'
            f'=======================================================\n'
            f'  EsiBot Dashboard — dashboard_node\n'
            f'=======================================================\n'
            f'  URL locale    : http://{local_ip}:{port}\n'
            f'  URL réseau    : http://0.0.0.0:{port}\n'
            f'  Dossier web   : {web_root}\n'
            f'  ROS Bridge    : ws://{local_ip}:9090  (web_bridge)\n'
            f'=======================================================\n'
        )

    def _get_local_ip(self) -> str:
        """Récupère l'IP locale de la machine (Raspberry Pi)."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return 'localhost'

    def destroy_node(self):
        if hasattr(self, '_server'):
            self._server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
