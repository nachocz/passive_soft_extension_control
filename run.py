"""Open the interactive pages locally (Python standard library only).

    python run.py                 serve docs/ at http://127.0.0.1:8765 and open the browser
    python run.py --port 9000     another port (a free port is used if it is taken)
    python run.py --no-browser
    python run.py --sandbox       serve the pages as anonymous.4open.science does: every document
                                  in a sandboxed origin, without CORS headers, not shown in frames

The pages are static files; any web server for the docs/ folder works as well,
e.g. GitHub Pages, and docs/index.html also opens directly from the disk.
"""
from __future__ import annotations

import argparse
import functools
import http.server
import socket
import sys
import threading
import webbrowser
from pathlib import Path

DOCS = Path(__file__).resolve().parent / "docs"


class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map,
                      ".js": "text/javascript", ".mjs": "text/javascript", ".json": "application/json",
                      ".woff2": "font/woff2", ".mp3": "audio/mpeg", ".mp4": "video/mp4", ".md": "text/plain; charset=utf-8"}

    sandbox = False

    def end_headers(self) -> None:
        if self.sandbox:
            self.send_header("Content-Security-Policy", "sandbox allow-scripts allow-popups allow-forms allow-modals")
            self.send_header("X-Frame-Options", "SAMEORIGIN")
        super().end_headers()

    def log_message(self, *args) -> None:      # quiet console
        pass


def free_port(preferred: int) -> int:
    with socket.socket() as s:
        try:
            s.bind(("127.0.0.1", preferred))
            return preferred
        except OSError:
            s.bind(("127.0.0.1", 0))
            return s.getsockname()[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--sandbox", action="store_true", help="serve the pages as anonymous.4open.science does")
    args = parser.parse_args()
    port = free_port(args.port)
    handler = type("SandboxHandler", (Handler,), {"sandbox": True}) if args.sandbox else Handler
    server = http.server.ThreadingHTTPServer(("127.0.0.1", port), functools.partial(handler, directory=str(DOCS)))
    url = f"http://127.0.0.1:{port}/"
    print(f"Serving {DOCS} at {url}  (Ctrl+C to stop)")
    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
