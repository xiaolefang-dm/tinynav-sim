"""Live manual round-trip through the HTTP elevator panel and Pilot teleop path."""

import asyncio, json, time, urllib.request, urllib.error
import websockets

BASE = "http://127.0.0.1:8092"


def status():
    return json.load(urllib.request.urlopen(BASE + "/status", timeout=2))


def command(action, floor=None, expect=True):
    data = {"action": action}
    if floor is not None:
        data["floor"] = floor
    req = urllib.request.Request(
        BASE + "/command",
        json.dumps(data).encode(),
        headers={"Content-Type": "application/json"},
    )
    try:
        result = json.load(urllib.request.urlopen(req, timeout=4))
    except urllib.error.HTTPError as e:
        result = json.load(e)
    print("COMMAND", data, result, flush=True)
    assert result["ok"] == expect, result


async def until(predicate, label, timeout=35):
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        s = status()
        if predicate(s):
            print(label, s, flush=True)
            return s
        await asyncio.sleep(0.1)
    raise RuntimeError(label + " timeout: " + str(status()))


async def drive(ws, target):
    start = time.monotonic()
    while time.monotonic() - start < 30:
        s = status()
        x = s["robot"][0]
        if abs(target - x) < 0.045:
            break
        await ws.send(
            json.dumps(
                {
                    "linear_x": max(-0.5, min(0.5, (target - x) * 1.8)),
                    "linear_y": 0.0,
                    "angular_z": 0.0,
                }
            )
        )
        await asyncio.sleep(0.06)
    else:
        raise RuntimeError("Drive timeout " + str(s))
    await ws.send(json.dumps({"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}))
    print("DRIVE", target, status(), flush=True)


async def run():
    async with websockets.connect("ws://127.0.0.1:8000/ws/teleop") as ws:
        try:
            await until(
                lambda s: s.get("confirmed") and s.get("robot") is not None, "READY"
            )
            await drive(ws, 2.62)
            for _ in range(30):
                await ws.send(
                    json.dumps({"linear_x": 0.5, "linear_y": 0.0, "angular_z": 0.0})
                )
                await asyncio.sleep(0.06)
            await ws.send(json.dumps({"linear_x": 0.0}))
            assert status()["robot"][0] < 2.69, status()
            print("CLOSED_DOOR_BLOCKED", status(), flush=True)
            command("open")
            await until(lambda s: s["door_open"], "DOOR_OPEN")
            command("floor", 1, expect=False)
            await drive(ws, 3.0)
            command("close", expect=False)
            await drive(ws, 4.5)
            command("close")
            await until(
                lambda s: s["door_fraction"] == 0 and s["motion"] == "idle", "CLOSED"
            )
            command("floor", 1)
            await until(
                lambda s: s["floor"] == 1 and abs(s["robot"][2] - 3) < 0.03,
                "CARRIED_TO_FLOOR_1",
            )
            command("open")
            await until(lambda s: s["door_open"], "UPPER_DOOR_OPEN")
            await drive(ws, 1.5)
            assert abs(status()["robot"][2] - 3) < 0.03
            print("REVERSE_EXIT_FLOOR_1", status(), flush=True)
            command("close")
            await until(
                lambda s: s["door_fraction"] == 0 and s["motion"] == "idle",
                "UPPER_CLOSED",
            )
            command("floor", 0)
            await until(lambda s: s["floor"] == 0, "EMPTY_CABIN_DOWN")
            assert abs(status()["robot"][2] - 3) < 0.03
            command("floor", 1)
            await until(lambda s: s["floor"] == 1, "EMPTY_CABIN_RETURN")
            command("open")
            await until(lambda s: s["door_open"], "RETURN_OPEN")
            await drive(ws, 4.5)
            command("close")
            await until(
                lambda s: s["door_fraction"] == 0 and s["motion"] == "idle",
                "RETURN_CLOSED",
            )
            command("floor", 0)
            await until(
                lambda s: s["floor"] == 0 and abs(s["robot"][2]) < 0.03,
                "CARRIED_TO_FLOOR_0",
            )
            command("open")
            await until(lambda s: s["door_open"], "GROUND_OPEN")
            await drive(ws, 0.8)
            print("ROUND_TRIP_PASSED", status(), flush=True)
        finally:
            await ws.send(
                json.dumps({"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0})
            )


asyncio.run(run())
