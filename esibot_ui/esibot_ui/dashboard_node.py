"""
dashboard_node.py — EsiBot UI
==============================
HTTP server for the EsiBot dashboard (React SPA).

Authentication flow (IoT-grade, no external deps):
  1. React app loads → GET /api/session
     - Valid cookie  → 200 {authenticated: true}  → show Dashboard
     - No/bad cookie → 401 {authenticated: false}  → show LoginPage
  2. User submits login form → POST /api/login {username, password}
     - Credentials verified server-side via scrypt hash (from .esibot_auth)
     - Success → Set-Cookie: esibot_session=<hmac-signed-token>; HttpOnly
     - Failure → 401 + per-IP rate limiting (5 attempts / 60 s)
  3. POST /api/logout → clears cookie, removes session

Session token: <random_hex>.<hmac_sha256_sig> — signed with a server-side
ephemeral key (regenerated on restart), stored in memory with TTL.

Credentials file: /home/esibot/.esibot_auth  (chmod 600)
  ESIBOT_USER=<username>
  ESIBOT_SCRYPT=<salt_hex>:<key_hex>
"""

import hashlib
import hmac
import http.server
import json
import os
import secrets
import socket
import threading
import time
from collections import defaultdict
from http.cookies import SimpleCookie

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


# ── Auth constants ────────────────────────────────────────────────────────────

_AUTH_FILE_DEFAULT = '/home/esibot/.esibot_auth'
_MAX_FAILS    = 5        # failed attempts before lockout
_WINDOW_SECS  = 60       # rolling window for rate limiting
_SESSION_TTL  = 8 * 3600 # session lifetime: 8 hours

# Ephemeral secret — regenerated on every restart; all sessions invalidated
_SESSION_SECRET: bytes = secrets.token_bytes(32)

_failed_attempts: dict = defaultdict(list)
_attempts_lock = threading.Lock()

_sessions: dict = {}     # token -> expiry_timestamp
_sessions_lock = threading.Lock()


# ── Credential helpers ────────────────────────────────────────────────────────

def _load_credentials() -> tuple[str, str]:
    path = os.environ.get('ESIBOT_AUTH_FILE', _AUTH_FILE_DEFAULT)
    if not os.path.isfile(path):
        raise RuntimeError(
            f"Auth file not found: {path}\n"
            "Generate it with: python3 ~/gen_auth.py"
        )
    user = stored = None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith('ESIBOT_USER='):
                user = line[len('ESIBOT_USER='):]
            elif line.startswith('ESIBOT_SCRYPT='):
                stored = line[len('ESIBOT_SCRYPT='):]
    if not user or not stored:
        raise RuntimeError(f"Auth file invalid (missing ESIBOT_USER or ESIBOT_SCRYPT): {path}")
    return user, stored


def _verify_password(password: str, stored: str) -> bool:
    try:
        salt_hex, key_hex = stored.split(':', 1)
        salt = bytes.fromhex(salt_hex)
        key = hashlib.scrypt(password.encode(), salt=salt, n=2**14, r=8, p=1)
        # constant-time comparison to prevent timing attacks
        return hmac.compare_digest(key.hex(), key_hex)
    except Exception:
        return False


# ── Rate limiting ─────────────────────────────────────────────────────────────

def _is_rate_limited(ip: str) -> bool:
    now = time.time()
    with _attempts_lock:
        _failed_attempts[ip] = [t for t in _failed_attempts[ip] if now - t < _WINDOW_SECS]
        return len(_failed_attempts[ip]) >= _MAX_FAILS


def _record_failure(ip: str) -> None:
    with _attempts_lock:
        _failed_attempts[ip].append(time.time())


# ── Session management ────────────────────────────────────────────────────────

def _create_session() -> str:
    raw = secrets.token_hex(32)
    sig = hmac.new(_SESSION_SECRET, raw.encode(), hashlib.sha256).hexdigest()
    token = f"{raw}.{sig}"
    with _sessions_lock:
        _sessions[token] = time.time() + _SESSION_TTL
    return token


def _validate_session(token: str | None) -> bool:
    if not token or '.' not in token:
        return False
    raw, sig = token.rsplit('.', 1)
    expected = hmac.new(_SESSION_SECRET, raw.encode(), hashlib.sha256).hexdigest()
    if not hmac.compare_digest(sig, expected):
        return False
    with _sessions_lock:
        expiry = _sessions.get(token, 0)
        if time.time() > expiry:
            _sessions.pop(token, None)
            return False
    return True


def _get_session_cookie(headers) -> str | None:
    raw = headers.get('Cookie', '')
    if not raw:
        return None
    c = SimpleCookie()
    c.load(raw)
    m = c.get('esibot_session')
    return m.value if m else None


# ── HTTP handler ──────────────────────────────────────────────────────────────

class _DashboardHandler(http.server.SimpleHTTPRequestHandler):

    _creds: tuple[str, str] | None = None  # loaded once at startup

    # ── Routing ───────────────────────────────────────────────────────────────

    def do_GET(self):
        if self.path == '/api/session':
            self._handle_session_check()
        else:
            super().do_GET()   # serve static files — no auth needed here
                               # (JS bundle no longer contains any credentials)

    def do_POST(self):
        if self.path == '/api/login':
            self._handle_login()
        elif self.path == '/api/logout':
            self._handle_logout()
        else:
            self._json(404, {'error': 'Not found'})

    # ── Auth endpoints ────────────────────────────────────────────────────────

    def _handle_session_check(self):
        token = _get_session_cookie(self.headers)
        if _validate_session(token):
            self._json(200, {'authenticated': True})
        else:
            self._json(401, {'authenticated': False})

    def _handle_login(self):
        ip = self.client_address[0]

        if _is_rate_limited(ip):
            self._json(429, {'error': 'Too many attempts — wait 60s'},
                       extra={'Retry-After': str(_WINDOW_SECS)})
            return

        length = int(self.headers.get('Content-Length', 0))
        try:
            body = json.loads(self.rfile.read(length))
            username = str(body.get('username', ''))
            password = str(body.get('password', ''))
        except Exception:
            self._json(400, {'error': 'Invalid JSON'})
            return

        expected_user, stored_hash = self._creds
        # Both checks run regardless — prevents username enumeration via timing
        user_ok = hmac.compare_digest(username, expected_user)
        pass_ok = _verify_password(password, stored_hash)

        if not (user_ok and pass_ok):
            _record_failure(ip)
            self._json(401, {'error': 'Invalid credentials'})
            return

        token = _create_session()
        self._json(
            200,
            {'authenticated': True},
            cookie=(
                f'esibot_session={token}; '
                f'HttpOnly; SameSite=Strict; Path=/; Max-Age={_SESSION_TTL}'
            ),
        )

    def _handle_logout(self):
        token = _get_session_cookie(self.headers)
        if token:
            with _sessions_lock:
                _sessions.pop(token, None)
        self._json(
            200, {},
            cookie='esibot_session=; HttpOnly; SameSite=Strict; Path=/; Max-Age=0'
        )

    # ── Response helpers ──────────────────────────────────────────────────────

    def _json(self, code: int, data: dict,
              cookie: str = None, extra: dict = None):
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        if cookie:
            self.send_header('Set-Cookie', cookie)
        if extra:
            for k, v in extra.items():
                self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass  # suppress access logs


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class DashboardNode(Node):

    def __init__(self):
        super().__init__('dashboard_node')

        self.declare_parameter('port',     8080)
        self.declare_parameter('host',     '0.0.0.0')
        self.declare_parameter('web_root', '')

        port     = self.get_parameter('port').value
        host     = self.get_parameter('host').value
        web_root = self.get_parameter('web_root').value

        if not web_root:
            pkg_share = get_package_share_directory('esibot_ui')
            web_root  = os.path.join(pkg_share, 'web')

        if not os.path.isdir(web_root):
            self.get_logger().error(
                f"Web root not found: {web_root}\n"
                "Build the dashboard: cd src/dashboard && npm run build\n"
                "Then copy dist/ → esibot_ui/web/ and rebuild the package."
            )
            return

        try:
            _DashboardHandler._creds = _load_credentials()
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return

        import functools
        handler = functools.partial(_DashboardHandler, directory=web_root)

        self._server = http.server.HTTPServer((host, port), handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            daemon=True,
            name='dashboard_http'
        )
        self._thread.start()

        local_ip = self._get_local_ip()
        self.get_logger().info(
            f'\n'
            f'=======================================================\n'
            f'  EsiBot Dashboard\n'
            f'=======================================================\n'
            f'  URL           : http://{local_ip}:{port}\n'
            f'  Auth          : session cookie (scrypt + HMAC-SHA256)\n'
            f'  Rate limit    : {_MAX_FAILS} attempts / {_WINDOW_SECS}s per IP\n'
            f'  Session TTL   : {_SESSION_TTL // 3600}h\n'
            f'=======================================================\n'
        )

    def _get_local_ip(self) -> str:
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
