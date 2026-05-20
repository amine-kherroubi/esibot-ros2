#!/usr/bin/env python3
"""
MJPEG Proxy Node — EsiBot Camera
==================================
Re-serves the ESP32-CAM MJPEG stream locally on port 8888.
The dashboard Raw tab connects directly via <img src="http://<Pi>:8888/stream">,
bypassing rosbridge entirely for maximum fluidity.
"""

import threading
import urllib.request
import urllib.error
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node


class MjpegProxyNode(Node):

    def __init__(self):
        super().__init__('mjpeg_proxy_node')

        self.declare_parameter('esp32_ip',   '192.168.1.80')
        self.declare_parameter('esp32_port', 80)
        self.declare_parameter('proxy_port', 8888)

        esp32_ip   = self.get_parameter('esp32_ip').value
        esp32_port = self.get_parameter('esp32_port').value
        proxy_port = self.get_parameter('proxy_port').value

        self.upstream_url = f'http://{esp32_ip}:{esp32_port}/stream'

        server = HTTPServer(('0.0.0.0', proxy_port), self._make_handler())
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

        self.get_logger().info(
            f'MJPEG proxy listening on port {proxy_port} '
            f'→ upstream {self.upstream_url}'
        )

    def _make_handler(self):
        upstream_url = self.upstream_url
        logger = self.get_logger()

        class Handler(BaseHTTPRequestHandler):

            def do_GET(self):
                if self.path != '/stream':
                    self.send_error(404)
                    return
                try:
                    upstream = urllib.request.urlopen(upstream_url, timeout=10)
                except urllib.error.URLError as e:
                    logger.warning(f'Upstream unreachable: {e.reason}')
                    self.send_error(502, 'ESP32 unreachable')
                    return

                self.send_response(200)
                content_type = upstream.headers.get(
                    'Content-Type',
                    'multipart/x-mixed-replace; boundary=frame'
                )
                self.send_header('Content-Type', content_type)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()

                try:
                    while True:
                        chunk = upstream.read(4096)
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    upstream.close()

            def log_message(self, *args):
                pass  # suppress HTTP access logs

        return Handler


def main(args=None):
    rclpy.init(args=args)
    node = MjpegProxyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
