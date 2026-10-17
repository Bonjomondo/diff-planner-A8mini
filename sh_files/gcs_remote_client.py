#!/usr/bin/env python3
"""Minimal GCS-side client for the protocol-2 remote mission link.

The onboard gateway listens on 127.0.0.1:8765. When the GCS runs on the same
machine as the gateway (this onboard computer), no SSH tunnel is needed.

Usage:
  python3 gcs_remote_client.py status
  python3 gcs_remote_client.py upload --points "-0.5,0,1.0; 0.5,0,1.0"
  python3 gcs_remote_client.py start

Points are "x,y,z" or "x,y,z,hover_sec", separated by ';'.
Coordinates must satisfy remote_max_abs_xy and remote_min_z/max_z from the launch.
"""
import argparse
import json
import socket
import sys
import uuid

HOST = "127.0.0.1"
PORT = 8765


def read_frame(stream):
    line = stream.readline()
    if not line:
        raise ConnectionError("gateway link closed")
    return json.loads(line)


def latest_telemetry(sock, stream):
    while True:
        frame = read_frame(stream)
        if frame.get("type") == "telemetry":
            return frame


def command(sock, stream, operation, **fields):
    request_id = str(uuid.uuid4())
    payload = dict(type="command", protocol=2, request_id=request_id,
                   operation=operation, map_session_id="", mission_id="", revision=0)
    payload.update(fields)
    sock.sendall(json.dumps(payload, allow_nan=False).encode("utf-8") + b"\n")
    while True:
        frame = read_frame(stream)  # skip async telemetry pushed meanwhile
        if frame.get("type") == "ack" and frame.get("request_id") == request_id:
            return frame


def parse_points(text):
    points = []
    for index, chunk in enumerate(text.split(";")):
        chunk = chunk.strip()
        if not chunk:
            continue
        values = [float(v) for v in chunk.split(",")]
        if len(values) not in (3, 4):
            raise ValueError("each point must be x,y,z or x,y,z,hover_sec: " + chunk)
        x, y, z = values[0], values[1], values[2]
        hover = values[3] if len(values) == 4 else 0.0
        points.append(dict(id=index + 1, x=x, y=y, z=z, hover_sec=hover))
    if not points:
        raise ValueError("no points given")
    return points


def print_mission(mission, prefix=""):
    print(prefix + json.dumps(mission, ensure_ascii=False, sort_keys=True))


def main():
    parser = argparse.ArgumentParser(description="Protocol-2 remote mission client")
    parser.add_argument("action", choices=["status", "upload", "start"])
    parser.add_argument("--points", help="'x,y,z[,hover_sec]; ...' for upload")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    with socket.create_connection((args.host, args.port), timeout=5) as sock:
        stream = sock.makefile("rb")
        telemetry = latest_telemetry(sock, stream)
        mission = telemetry["mission"]

        if args.action == "status":
            print(json.dumps(dict(state=mission["state"], map_session_id=mission["map_session_id"],
                                  mission_id=mission["mission_id"], revision=mission["revision"],
                                  completed=mission["completed"], total=mission["total"],
                                  odom_fresh=mission["odom_fresh"], flight_ready=mission["flight_ready"],
                                  armed=telemetry["armed"], mode=telemetry["mode"],
                                  world=(telemetry["world_x"], telemetry["world_y"], telemetry["world_z"])),
                             ensure_ascii=False, indent=2))
            return

        if args.action == "upload":
            if not args.points:
                parser.error("upload requires --points")
            ack = command(sock, stream, "upload",
                          map_session_id=mission["map_session_id"],
                          mission_id="m" + uuid.uuid4().hex[:8], revision=1,
                          frame_id="world", waypoints=parse_points(args.points))
            print_mission(ack["mission"], "upload accepted=%s reason=%s\n" % (ack["accepted"], ack["reason"]))
            if not ack["accepted"]:
                sys.exit(1)
            print("Now take off and confirm hover, then run: start")
            return

        if args.action == "start":
            if mission["state"] != "READY":
                print("Mission is not READY (state=%s); upload first" % mission["state"], file=sys.stderr)
                sys.exit(1)
            ack = command(sock, stream, "start",
                          map_session_id=mission["map_session_id"],
                          mission_id=mission["mission_id"], revision=mission["revision"])
            print_mission(ack["mission"], "start accepted=%s reason=%s\n" % (ack["accepted"], ack["reason"]))
            if not ack["accepted"]:
                sys.exit(1)


if __name__ == "__main__":
    main()