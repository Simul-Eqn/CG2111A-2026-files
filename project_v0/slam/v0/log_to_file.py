import sys
from datetime import datetime
from pathlib import Path


_LOG_FILE = Path(__file__).with_name("lidar_client.log")


def _client_log(message: str) -> None:
    """Write client logs to stderr and a file so Textual UI does not hide them."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{timestamp}] {message}"
    print(line, file=sys.stderr, flush=True)
    try:
        with _LOG_FILE.open("a", encoding="utf-8") as f:
            f.write(line + "\n")
    except Exception:
        # Logging failures should not interrupt data acquisition.
        pass
