#!/usr/bin/env python3
"""
net_utils.py  -  Network utilities for the arm terminal relay.

Provides a minimal, SSL-upgradeable TCP layer for:
  - slam.py relay server that accepts the arm terminal
  - arm_terminal.py  client that connects to that relay

Public API
----------
TCPServer         - bind on a port, accept one client connection
TCPClient         - connect to a TCPServer
sendTPacketFrame  - forward a raw TPacket frame (MAGIC + payload + checksum)
recvTPacketFrame  - receive a raw TPacket frame
"""

import select as _select
import socket
import struct
import ssl
from typing import Optional


# ---------------------------------------------------------------------------
# Length-prefix framing
# ---------------------------------------------------------------------------
# Every message is preceded by a 4-byte big-endian unsigned integer that
# gives the payload length.

_LEN_FMT  = '>I'
_LEN_SIZE = struct.calcsize(_LEN_FMT)


def _sendFramed(sock, data: bytes) -> bool:
    """Send *data* over *sock* with a 4-byte length header.

    Returns True on success; False if a network error occurred.
    """
    try:
        header = struct.pack(_LEN_FMT, len(data))
        sock.sendall(header + data)
        return True
    except (OSError, BrokenPipeError) as err:
        print(f"[net_utils] send error: {err}")
        return False


def _recvFramed(sock):
    """Receive one length-prefixed message from *sock*.

    Returns the payload bytes, or None if the connection was closed or an error occurred.
    """
    header = _recvExact(sock, _LEN_SIZE)
    if header is None:
        return None
    length = struct.unpack(_LEN_FMT, header)[0]
    if length == 0:
        return b''
    return _recvExact(sock, length)


def _recvExact(sock, n: int):
    """Read exactly *n* bytes from *sock*, blocking until all have arrived.

    Returns the bytes read, or None on error or connection close.
    """
    buf = b''
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except (OSError, ConnectionResetError) as err:
            print(f"[net_utils] recv error: {err}")
            return None
        if not chunk:
            return None
        buf += chunk
    return buf


# ---------------------------------------------------------------------------
# TPacket frame forwarding
# ---------------------------------------------------------------------------

def sendTPacketFrame(sock, raw_frame: bytes) -> bool:
    """Send a complete framed TPacket (MAGIC + TPacket + checksum).

    Args:
        sock: connected socket
        raw_frame: complete 103-byte frame

    Returns:
        True on success, False on error
    """
    return _sendFramed(sock, raw_frame)


def recvTPacketFrame(sock):
    """Receive a complete framed TPacket frame.

    Returns the 103-byte frame, or None on error.
    """
    return _recvFramed(sock)


# ---------------------------------------------------------------------------
# TCP Server / Client
# ---------------------------------------------------------------------------

class TCPServer:
    def __init__(self, port: int, ssl_context: Optional[ssl.SSLContext] = None):
        self.port = port
        self.ssl_context = ssl_context
        self.sock = None
        self.accepted_sock = None

    def start(self) -> bool:
        """Start listening on the configured port."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(('0.0.0.0', self.port))
            self.sock.listen(1)
            self.sock.settimeout(0.1)
            return True
        except OSError as err:
            print(f"[net_utils] Failed to start server on port {self.port}: {err}")
            self.sock = None
            return False

    def accept(self, timeout: int = 30) -> Optional[socket.socket]:
        """Wait for one incoming client connection.

        Args:
            timeout: seconds to wait for a connection

        Returns:
            connected socket (plain TCP or TLS-wrapped), or None on timeout
        """
        if not self.sock:
            return None

        import time
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                client_sock, addr = self.sock.accept()
                if self.ssl_context:
                    try:
                        client_sock = self.ssl_context.wrap_socket(
                            client_sock, server_side=True
                        )
                    except ssl.SSLError as err:
                        print(f"[net_utils] TLS handshake failed with {addr}: {err}")
                        client_sock.close()
                        continue
                return client_sock
            except socket.timeout:
                time.sleep(0.05)
        return None

    def hasData(self) -> bool:
        """Check if the socket has any data ready."""
        if not self.sock:
            return False
        try:
            rlist, _, _ = _select.select([self.sock], [], [], 0)
            return len(rlist) > 0
        except:
            return False

    def close(self):
        """Close the server socket."""
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None


class TCPClient:
    def __init__(self, host: str, port: int, ssl_context: Optional[ssl.SSLContext] = None):
        self.host = host
        self.port = port
        self.ssl_context = ssl_context
        self.sock = None

    def connect(self) -> bool:
        """Connect to the remote server.

        Returns True on success, False on failure.
        """
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            if self.ssl_context:
                try:
                    self.sock = self.ssl_context.wrap_socket(
                        self.sock, server_hostname=self.host
                    )
                except ssl.SSLError as err:
                    print(f"[net_utils] TLS handshake failed: {err}")
                    self.sock.close()
                    self.sock = None
                    return False
            return True
        except OSError as err:
            print(f"[net_utils] Connection to {self.host}:{self.port} failed: {err}")
            self.sock = None
            return False

    def close(self):
        """Close the client connection."""
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
