#!/usr/bin/env python3
"""
arm_relay_manager.py  -  Manages ARM relay and API server communication for slam.py.

This module starts the arm relay server and manages communication with the robot API server.
It should be imported and started by slam.py during initialization.

Usage in slam.py:
    from arm_relay_manager import start_relay, stop_relay
    
    # In the app's on_mount():
    start_relay()
    
    # In the app's on_unmount():
    stop_relay()
"""

import os
import asyncio
import struct
import threading
from pathlib import Path

# Try to import the relay module from the arm_terminal package
try:
    from arm_terminal import relay as arm_relay
    _relay_available = True
except ImportError as err:
    print(f"[relay_manager] Warning: arm_terminal.relay not available: {err}")
    _relay_available = False

# Import default configuration from settings
try:
    from settings import (
        ARM_SERVER_IP_DEFAULT,
        ARM_SERVER_PORT_DEFAULT,
        ARM_RELAY_PORT_DEFAULT,
        ARM_TERM_TIMEOUT_DEFAULT,
    )
except ImportError:
    # Fallback defaults if settings.py is not available
    ARM_SERVER_IP_DEFAULT = '100.71.68.106'
    ARM_SERVER_PORT_DEFAULT = 9999
    ARM_RELAY_PORT_DEFAULT = 65433
    ARM_TERM_TIMEOUT_DEFAULT = 30


# ============================================================
# Configuration
# ============================================================

SLAM_SERVER_IP = os.getenv('SLAM_SERVER_IP', ARM_SERVER_IP_DEFAULT)
SLAM_API_PORT = int(os.getenv('SLAM_LIDAR_PORT', str(ARM_SERVER_PORT_DEFAULT)))


# ============================================================
# Module State
# ============================================================

_relay_thread = None
_api_connection = None
_api_lock = threading.Lock()


# ============================================================
# API Server Connection
# ============================================================

class APIServerClient:
    """Simple async client for the robot API server."""
    
    def __init__(self, host: str = SLAM_SERVER_IP, port: int = SLAM_API_PORT):
        self.host = host
        self.port = port
        self.reader = None
        self.writer = None
    
    async def connect(self) -> bool:
        """Connect to the API server."""
        try:
            self.reader, self.writer = await asyncio.open_connection(
                self.host, self.port
            )
            print(f"[relay_manager] Connected to API server at {self.host}:{self.port}")
            return True
        except Exception as err:
            print(f"[relay_manager] Failed to connect to API server: {err}")
            return False
    
    async def send_frame(self, raw_frame: bytes) -> bool:
        """Send a raw TPacket frame to the API server.
        
        Args:
            raw_frame: 103-byte framed TPacket
        
        Returns:
            True if sent successfully, False otherwise
        """
        if not self.writer:
            return False
        
        try:
            # Send with length prefix (4-byte big-endian)
            length_hdr = struct.pack('>I', len(raw_frame))
            self.writer.write(length_hdr + raw_frame)
            await self.writer.drain()
            return True
        except Exception as err:
            print(f"[relay_manager] Failed to send to API: {err}")
            self._close()
            return False
    
    def _close(self):
        """Close the connection."""
        if self.writer:
            try:
                self.writer.close()
            except:
                pass
            self.writer = None
        self.reader = None
    
    async def close(self):
        """Async close."""
        self._close()


# ============================================================
# Relay Management
# ============================================================

async def _api_relay_callback(raw_frame: bytes) -> bool:
    """Callback that relays frames from arm_terminal to the API server.
    
    This is called by the arm relay when it receives a command from the arm terminal.
    
    Args:
        raw_frame: 103-byte framed TPacket command
    
    Returns:
        True if relayed successfully, False otherwise
    """
    global _api_connection
    
    with _api_lock:
        if _api_connection is None:
            return False
        return await _api_connection.send_frame(raw_frame)


def _setup_relay():
    """Set up the arm relay and API server connection."""
    global _api_connection
    
    if not _relay_available:
        print("[relay_manager] ARM relay module not available.")
        return False
    
    # Create API connection
    _api_connection = APIServerClient(SLAM_SERVER_IP, SLAM_API_PORT)
    
    # Connect to API server
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(_api_connection.connect())
    except Exception as err:
        print(f"[relay_manager] Failed to set up API connection: {err}")
        return False
    
    # Set the relay callback
    arm_relay.set_api_relay_callback(_api_relay_callback)
    
    # Start the relay server
    arm_relay.start()
    print("[relay_manager] ARM relay started.")
    return True


def start_relay(host: str = None, port: int = None):
    """Start the ARM relay server.
    
    This should be called from slam.py during initialization.
    
    Args:
        host: API server host (uses SLAM_SERVER_IP env var if not provided)
        port: API server port (uses SLAM_LIDAR_PORT env var if not provided)
    """
    global SLAM_SERVER_IP, SLAM_API_PORT
    
    if host:
        SLAM_SERVER_IP = host
    if port:
        SLAM_API_PORT = port
    
    if not _relay_available:
        print("[relay_manager] ARM relay not available; skipping startup.")
        return
    
    print("[relay_manager] Starting ARM relay...")
    _setup_relay()


def stop_relay():
    """Stop the ARM relay server.
    
    This should be called from slam.py's cleanup code.
    """
    global _api_connection
    
    if not _relay_available:
        return
    
    print("[relay_manager] Shutting down ARM relay...")
    
    arm_relay.shutdown()
    
    with _api_lock:
        if _api_connection:
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(_api_connection.close())
            except:
                pass
            _api_connection = None
    
    print("[relay_manager] ARM relay shut down.")


def is_relay_available() -> bool:
    """Check if the ARM relay is available."""
    return _relay_available


def is_relay_connected() -> bool:
    """Check if the arm terminal is currently connected via the relay."""
    if not _relay_available:
        return False
    return arm_relay.is_client_connected()
