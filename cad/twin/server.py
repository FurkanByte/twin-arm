"""TwinArm-5 digital-twin web app.

    python twin/server.py                  # sim mode, http://localhost:8080
    python twin/server.py --port COM5      # live mode: connects to the ESP32

From a phone on the same network: open the printed http://<computer's IP>:8080 address.
Physics (MuJoCo) and the ESP32 link run in this process; browsers receive the state over a WebSocket
and send commands over the same channel. Several browsers can connect at the same time.
"""
import argparse
import asyncio
import json
import os
import socket
import sys
import webbrowser

from aiohttp import WSMsgType, web

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core import Controller, list_ports  # noqa: E402

WEB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
RATE_HZ = 30


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


async def index(request):
    return web.FileResponse(os.path.join(WEB, "index.html"), headers={"Cache-Control": "no-cache"})


async def model(request):
    return web.json_response(request.app["ctl"].model_info())


async def mesh(request):
    i = int(request.match_info["i"])
    ctl = request.app["ctl"]
    if not 0 <= i < ctl.twin.m.nmesh:
        raise web.HTTPNotFound()
    return web.Response(body=ctl.mesh_bytes(i), content_type="application/octet-stream",
                        headers={"Cache-Control": "max-age=3600"})


async def ports(request):
    return web.json_response(list_ports())


async def ws_handler(request):
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    request.app["clients"].add(ws)
    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                try:
                    cmd = json.loads(msg.data)
                except ValueError:
                    continue
                await asyncio.get_running_loop().run_in_executor(None, request.app["ctl"].command, cmd)
    finally:
        request.app["clients"].discard(ws)
    return ws


async def broadcaster(app):
    ctl = app["ctl"]
    while True:
        await asyncio.sleep(1 / RATE_HZ)
        if not app["clients"]:
            continue
        data = json.dumps(ctl.snapshot(), separators=(",", ":"))
        for ws in list(app["clients"]):
            try:
                await ws.send_str(data)
            except Exception:
                app["clients"].discard(ws)


async def on_startup(app):
    app["task"] = asyncio.create_task(broadcaster(app))


async def on_cleanup(app):
    app["task"].cancel()


def main():
    ap = argparse.ArgumentParser(description="TwinArm-5 digital twin (web)")
    ap.add_argument("--port", help="ESP32 serial port (e.g. COM5); sim mode if omitted")
    ap.add_argument("--http", type=int, default=8080, help="web port")
    ap.add_argument("--host", default="0.0.0.0", help="0.0.0.0: phones on the same network can connect too")
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()

    app = web.Application()
    app["ctl"] = Controller(a.port)
    app["clients"] = set()
    app.router.add_get("/", index)
    app.router.add_get("/api/model", model)
    app.router.add_get("/api/mesh/{i}", mesh)
    app.router.add_get("/api/ports", ports)
    app.router.add_get("/ws", ws_handler)
    app.router.add_static("/static", WEB)
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)

    url = f"http://localhost:{a.http}"
    print(f"TwinArm-5 digital twin running:\n  this computer: {url}\n  phone (same Wi-Fi): http://{lan_ip()}:{a.http}",
          flush=True)
    if not a.no_browser:
        webbrowser.open(url)
    web.run_app(app, host=a.host, port=a.http, print=None)


if __name__ == "__main__":
    main()
