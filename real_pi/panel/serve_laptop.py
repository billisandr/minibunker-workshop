#!/usr/bin/env python3
# ============================================================================
#  serve_laptop.py — serve THIS panel page from a laptop (the "laptop server"
#  option). The page then talks to the Pi's API over the network.
#
#  The control loop + API always run ON THE PI (that's where the CAN + camera
#  are): `python run.py --web` there. This script only hosts the static page so
#  you can open it on the laptop and point its "API base" field at the Pi, e.g.
#      http://raspberrypi2.local:8080
#  (CORS on the Pi's API is open, so cross-origin calls work.)
#
#  Usage (on the LAPTOP, from real_pi/panel/):
#      python serve_laptop.py                 # http://localhost:8090
#      python serve_laptop.py 9000            # custom port
#  Then open the URL and set API base to http://<pi-host>:8080.
#
#  (Equivalent one-liner: `python -m http.server 8090` from this directory.)
# ============================================================================
import os
import sys
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8090
HERE = os.path.dirname(os.path.abspath(__file__))

handler = partial(SimpleHTTPRequestHandler, directory=HERE)
print(f"[serve_laptop] panel at  http://localhost:{PORT}")
print("[serve_laptop] set the page's API base to  http://<pi-host>:8080  "
      "(e.g. http://raspberrypi2.local:8080)")
print("[serve_laptop] Ctrl-C to stop.")
try:
    ThreadingHTTPServer(("0.0.0.0", PORT), handler).serve_forever()
except KeyboardInterrupt:
    print("\n[serve_laptop] stopped.")
