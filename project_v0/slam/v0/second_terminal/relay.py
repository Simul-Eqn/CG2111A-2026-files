#!/usr/bin/env python3
"""Second terminal relay server with reconnect support."""

import os
import ssl
import sys
from pathlib import Path

from .net_utils import TCPServer, sendTPacketFrame, recvTPacketFrame

try:
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from settings import ARM_TERM_TIMEOUT_DEFAULT
except Exception:
    ARM_TERM_TIMEOUT_DEFAULT = 30

TLS_ENABLED = os.getenv('TLS_ENABLED', 'True').lower() in ('true', '1', 'yes')
SECOND_TERM_PORT = int(os.getenv('SECOND_TERM_PORT', '65432'))
SECOND_TERM_TIMEOUT = int(os.getenv('SECOND_TERM_TIMEOUT', str(ARM_TERM_TIMEOUT_DEFAULT)))

_script_dir = Path(__file__).parent.parent
TLS_CERT_PATH = str(_script_dir / 'certs' / 'server.crt')
TLS_KEY_PATH = str(_script_dir / 'certs' / 'server.key')

_st_server = None
_st_conn = None


def _make_server_ssl_context():
    if not TLS_ENABLED:
        return None
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.load_cert_chain(TLS_CERT_PATH, TLS_KEY_PATH)
    return ctx


def _try_accept(timeout: float = 0.05):
    global _st_conn
    if _st_server is None or _st_conn is not None:
        return
    conn = _st_server.accept(timeout=timeout)
    if conn is not None:
        _st_conn = conn
        print('[second_relay] second_terminal connected')


def onPacketReceived(raw_frame: bytes):
    global _st_conn
    if _st_conn is None:
        _try_accept(0.0)
        return
    ok = sendTPacketFrame(_st_conn, raw_frame)
    if not ok:
        print('[second_relay] second_terminal disconnected (send failed)')
        _st_conn = None


def recvFromSecondTerminal():
    global _st_conn

    if _st_server is None:
        return None

    if _st_conn is None:
        _try_accept(0.01)
        return None

    if not _st_server.hasData():
        return None

    frame = recvTPacketFrame(_st_conn)
    if frame is None:
        print('[second_relay] second_terminal disconnected')
        _st_conn = None
        return None
    return frame


def start():
    global _st_server, _st_conn

    if _st_server is not None:
        return

    ssl_context = _make_server_ssl_context() if TLS_ENABLED else None
    _st_server = TCPServer(port=SECOND_TERM_PORT, ssl_context=ssl_context)
    _st_conn = None
    if _st_server.start():
        print(
            '[second_relay] waiting for second_terminal.py '
            f'on port {SECOND_TERM_PORT} (reconnect enabled)'
        )
        _try_accept(timeout=SECOND_TERM_TIMEOUT)


def shutdown():
    global _st_server, _st_conn
    if _st_server is not None:
        _st_server.close()
        _st_server = None
    _st_conn = None
    print('[second_relay] shutdown complete')
