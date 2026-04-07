#!/usr/bin/env python3
"""
relay.py  -  Arm terminal relay for slam.py.

This module runs a TCP/TLS server that listens for connections from arm_terminal.py.
Commands received from the arm terminal are relayed to the API server.

The relay is designed to be integrated into slam.py or run as a standalone service.

Architecture:
    [arm_terminal.py] <--TCP:65433--> [slam.py relay] <--API--> [rp_lidar_api.py]

Environment variables:
    ARM_RELAY_PORT      - Port to listen on (default: 65433)
    ARM_TERM_TIMEOUT    - Seconds to wait for connection (default: 30)
    TLS_ENABLED         - Enable TLS (default: True)
    TLS_CERT_PATH       - Path to server.crt (default: certs/server.crt)
    TLS_KEY_PATH        - Path to server.key (default: certs/server.key)
"""

import os
import ssl
import socket
import struct
import select
import threading
import time
import sys
from pathlib import Path
from datetime import datetime

from .net_utils import TCPServer, sendTPacketFrame, recvTPacketFrame

# Try to import default configuration from settings
try:
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from settings import ARM_RELAY_PORT_DEFAULT, ARM_TERM_TIMEOUT_DEFAULT
except ImportError:
    # Fallback defaults if settings.py is not available
    ARM_RELAY_PORT_DEFAULT = 65433
    ARM_TERM_TIMEOUT_DEFAULT = 30


# ============================================================
# Configuration (from environment or defaults)
# ============================================================

ARM_RELAY_PORT = int(os.getenv('ARM_RELAY_PORT', str(ARM_RELAY_PORT_DEFAULT)))
ARM_TERM_TIMEOUT = int(os.getenv('ARM_TERM_TIMEOUT', str(ARM_TERM_TIMEOUT_DEFAULT)))
TLS_ENABLED = os.getenv('TLS_ENABLED', 'True').lower() in ('true', '1', 'yes')

# Paths are relative to certs/ in the slam directory
_script_dir = Path(__file__).parent.parent
TLS_CERT_PATH = str(_script_dir / 'certs' / 'server.crt')
TLS_KEY_PATH = str(_script_dir / 'certs' / 'server.key')


# ============================================================
# Module state
# ============================================================

_relay_server = None
_relay_thread = None
_relay_client = None
_relay_lock = threading.Lock()
_shutdown_event = threading.Event()

# Import function reference for relaying to API server
_api_relay_callback = None


# ============================================================
# API Relay Callback Setup
# ============================================================

def set_api_relay_callback(callback):
    """Set the callback function that relays frames to the API server.

    The callback should accept a raw TPacket frame and forward it to the API.
    This is called from slam.py to connect the relay to the actual API server.

    Args:
        callback: async function(raw_frame: bytes) -> bool
                  Returns True if frame was sent successfully.
    """
    global _api_relay_callback
    _api_relay_callback = callback


# ============================================================
# TLS Context
# ============================================================

def _make_server_ssl_context():
    """Create a TLS server context using the configured certificate and key."""
    if not TLS_ENABLED:
        return None
    try:
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.minimum_version = ssl.TLSVersion.TLSv1_2
        ctx.load_cert_chain(TLS_CERT_PATH, TLS_KEY_PATH)
        return ctx
    except (FileNotFoundError, ssl.SSLError) as err:
        print(f"[arm_relay] Warning: TLS setup failed: {err}")
        print(f"[arm_relay]   Cert: {TLS_CERT_PATH}")
        print(f"[arm_relay]   Key:  {TLS_KEY_PATH}")
        return None


# ============================================================
# Frame relaying
# ============================================================

def onPacketReceived(raw_frame: bytes):
    """Forward a raw TPacket frame from the API server to the arm terminal.

    Call this when the API server sends a response/message that should go
    to the arm terminal.

    Args:
        raw_frame: the complete framed packet (MAGIC + TPacket + checksum)
    """
    global _relay_client

    with _relay_lock:
        if _relay_client is not None:
            try:
                ok = sendTPacketFrame(_relay_client, raw_frame)
                if not ok:
                    print("[arm_relay] Forward to arm terminal failed.")
                    _relay_client = None
            except Exception as err:
                print(f"[arm_relay] Error forwarding frame: {err}")
                _relay_client = None


def _relay_from_client_to_api():
    """Read frames from the arm terminal and relay them to the API server.

    Runs in the relay thread.
    """
    global _relay_client

    if _relay_client is None or _api_relay_callback is None:
        return

    try:
        frame = recvTPacketFrame(_relay_client)
        if frame is None:
            print("[arm_relay] Arm terminal disconnected.")
            with _relay_lock:
                _relay_client = None
            return
        
        # Relay to API server from this worker thread.
        import asyncio
        try:
            asyncio.run(_api_relay_callback(frame))
        except Exception as err:
            print(f"[arm_relay] API relay failed: {err}")
    except Exception as err:
        print(f"[arm_relay] Client read error: {err}")


# ============================================================
# Relay Server Thread
# ============================================================

def _relay_server_thread():
    """Main loop for the relay server thread.

    Waits for an incoming connection from arm_terminal.py and reads frames
    from it, relaying them to the API server.
    """
    global _relay_server, _relay_client

    print(f"[arm_relay] Starting relay server on port {ARM_RELAY_PORT}...")
    
    ssl_context = _make_server_ssl_context() if TLS_ENABLED else None
    _relay_server = TCPServer(port=ARM_RELAY_PORT, ssl_context=ssl_context)
    
    if not _relay_server.start():
        print(f"[arm_relay] Failed to start server.")
        return

    print(f"[arm_relay] Relay server listening on port {ARM_RELAY_PORT}.")
    print(f"[arm_relay] Waiting for arm_terminal.py to connect "
          f"(timeout: {ARM_TERM_TIMEOUT}s)...")

    _relay_client = _relay_server.accept(timeout=ARM_TERM_TIMEOUT)
    if _relay_client is None:
        print(f"[arm_relay] No arm terminal connected within {ARM_TERM_TIMEOUT}s.")
        return
    
    print(f"[arm_relay] Arm terminal connected!")

    # Main relay loop
    while not _shutdown_event.is_set():
        # Check for incoming data from the arm terminal
        try:
            if _relay_client:
                # Only read a frame when data is ready; otherwise keep connection alive.
                readable, _, errored = select.select([_relay_client], [], [_relay_client], 0.1)
                if errored:
                    with _relay_lock:
                        _relay_client = None
                    print("[arm_relay] Arm terminal disconnected (socket error).")
                    break
                if readable:
                    _relay_from_client_to_api()
        except Exception as err:
            print(f"[arm_relay] Thread error: {err}")
            time.sleep(0.1)

    print("[arm_relay] Relay server thread exiting.")


# ============================================================
# Lifecycle
# ============================================================

def start():
    """Start the arm relay server in a background thread.

    Call this from slam.py after setting up the API relay callback.
    """
    global _relay_thread, _shutdown_event

    if _relay_thread is not None and _relay_thread.is_alive():
        print("[arm_relay] Relay already running.")
        return

    _shutdown_event.clear()
    _relay_thread = threading.Thread(target=_relay_server_thread, daemon=True)
    _relay_thread.start()
    print("[arm_relay] Relay started.")


def shutdown():
    """Stop the relay server and close all connections.

    Call this from slam.py's cleanup code.
    """
    global _relay_thread, _relay_server, _relay_client, _shutdown_event

    print("[arm_relay] Shutting down...")
    _shutdown_event.set()

    with _relay_lock:
        if _relay_client is not None:
            try:
                _relay_client.close()
            except:
                pass
            _relay_client = None

    if _relay_server is not None:
        try:
            _relay_server.close()
        except:
            pass
        _relay_server = None

    if _relay_thread is not None and _relay_thread.is_alive():
        _relay_thread.join(timeout=2.0)
    _relay_thread = None

    print("[arm_relay] Relay shut down.")


def is_client_connected() -> bool:
    """Check if an arm terminal is currently connected."""
    global _relay_client
    with _relay_lock:
        return _relay_client is not None
