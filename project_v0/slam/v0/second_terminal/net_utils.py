#!/usr/bin/env python3
"""Network helpers for second terminal relay (length-prefixed TCP/TLS)."""

import select as _select
import socket
import struct

_LEN_FMT = '>I'
_LEN_SIZE = struct.calcsize(_LEN_FMT)


def _recv_exact(sock, n: int):
    buf = b''
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except (OSError, ConnectionResetError):
            return None
        if not chunk:
            return None
        buf += chunk
    return buf


def _send_framed(sock, payload: bytes) -> bool:
    try:
        header = struct.pack(_LEN_FMT, len(payload))
        sock.sendall(header + payload)
        return True
    except (OSError, BrokenPipeError):
        return False


def _recv_framed(sock):
    header = _recv_exact(sock, _LEN_SIZE)
    if header is None:
        return None
    size = struct.unpack(_LEN_FMT, header)[0]
    if size == 0:
        return b''
    return _recv_exact(sock, size)


def sendTPacketFrame(sock, frame: bytes) -> bool:
    return _send_framed(sock, frame)


def recvTPacketFrame(sock):
    return _recv_framed(sock)


class TCPServer:
    def __init__(self, host='0.0.0.0', port=65432, ssl_context=None):
        self.host = host
        self.port = port
        self.ssl_context = ssl_context
        self._server_sock = None
        self.conn = None

    def start(self) -> bool:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen(1)
            self._server_sock = s
            print(f"[TCPServer] Listening on {self.host}:{self.port}")
            return True
        except OSError as err:
            print(f"[TCPServer] Could not bind {self.host}:{self.port}: {err}")
            return False

    def accept(self, timeout: float = 0.1):
        if not self._server_sock:
            return None
        self._server_sock.settimeout(timeout)
        try:
            conn, addr = self._server_sock.accept()
            if self.ssl_context is not None:
                conn = self.ssl_context.wrap_socket(conn, server_side=True)
            conn.setblocking(True)
            self.conn = conn
            print(f"[TCPServer] Client connected from {addr}")
            return conn
        except socket.timeout:
            return None
        except OSError as err:
            print(f"[TCPServer] Accept error: {err}")
            return None

    def hasData(self) -> bool:
        if not self.conn:
            return False
        r, _, _ = _select.select([self.conn], [], [], 0)
        return bool(r)

    def close(self):
        if self.conn is not None:
            try:
                self.conn.close()
            except OSError:
                pass
            self.conn = None
        if self._server_sock is not None:
            try:
                self._server_sock.close()
            except OSError:
                pass
            self._server_sock = None


class TCPClient:
    def __init__(self, host='localhost', port=65432, ssl_context=None, server_hostname=None):
        self.host = host
        self.port = port
        self.ssl_context = ssl_context
        self.server_hostname = server_hostname or host
        self.sock = None

    def connect(self, timeout: float = 5.0) -> bool:
        try:
            s = socket.create_connection((self.host, self.port), timeout=timeout)
            if self.ssl_context is not None:
                s = self.ssl_context.wrap_socket(s, server_hostname=self.server_hostname)
            s.setblocking(True)
            self.sock = s
            return True
        except OSError as err:
            print(f"[TCPClient] Connect failed {self.host}:{self.port}: {err}")
            return False

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
