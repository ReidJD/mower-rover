#!/usr/bin/env python3
"""
test_ntrip.py — EarthScope NTRIP connection tester.

Tests your EarthScope credentials against ntrip.earthscope.org and
verifies that the P779 station is streaming live RTCM corrections.

Usage:
    python test_ntrip.py                        # prompts for credentials
    python test_ntrip.py --user U --password P  # non-interactive
    python test_ntrip.py --user U --password P --station P779
    python test_ntrip.py --sourcetable-only     # just list available streams

Requirements: Python 3.6+ standard library only (no pip installs needed).
"""

import argparse
import base64
import getpass
import socket
import sys
import time

# ── EarthScope NTRIP caster ───────────────────────────────────────────────────
HOST = "ntrip.earthscope.org"
PORT = 2101

# RTCM byte-stream sanity check: valid RTCM3 frames start with 0xD3
RTCM3_PREAMBLE = 0xD3


def make_auth_header(user: str, password: str) -> str:
    token = base64.b64encode(f"{user}:{password}".encode()).decode()
    return f"Authorization: Basic {token}"


def fetch_sourcetable(user: str, password: str) -> list[str]:
    """
    Fetch the caster sourcetable. Returns the raw lines.
    Raises ConnectionError on auth failure or network error.
    """
    request = (
        f"GET / HTTP/1.0\r\n"
        f"Host: {HOST}\r\n"
        f"Ntrip-Version: Ntrip/1.0\r\n"
        f"User-Agent: NTRIP TestClient/1.0\r\n"
        f"{make_auth_header(user, password)}\r\n"
        f"\r\n"
    )

    with socket.create_connection((HOST, PORT), timeout=10) as s:
        s.sendall(request.encode())
        data = b""
        while True:
            chunk = s.recv(4096)
            if not chunk:
                break
            data += chunk
            # Sourcetable ends with ENDSOURCETABLE
            if b"ENDSOURCETABLE" in data:
                break

    text = data.decode(errors="replace")
    lines = text.splitlines()

    if not lines:
        raise ConnectionError("No response from caster.")

    status = lines[0]
    if "200 OK" not in status and "ICY 200" not in status:
        raise ConnectionError(f"Auth failed or bad response: {status}")

    return lines


def find_mountpoints(sourcetable_lines: list[str], station: str) -> list[str]:
    """Return all STR lines matching the station name."""
    station_upper = station.upper()
    return [
        line for line in sourcetable_lines
        if line.startswith("STR;") and station_upper in line.upper()
    ]


def stream_rtcm(user: str, password: str, mountpoint: str, duration: int = 5) -> dict:
    """
    Connect to a mountpoint and read RTCM data for `duration` seconds.
    Tries Ntrip/2.0 (HTTP/1.1) first, falls back to Ntrip/1.0 (HTTP/1.0).
    Returns a dict with bytes_received, rtcm_frames, elapsed.
    """
    # EarthScope requires Ntrip/2.0 for data streams
    request = (
        f"GET /{mountpoint} HTTP/1.1\r\n"
        f"Host: {HOST}\r\n"
        f"Ntrip-Version: Ntrip/2.0\r\n"
        f"User-Agent: NTRIP TestClient/1.0\r\n"
        f"{make_auth_header(user, password)}\r\n"
        f"Connection: close\r\n"
        f"\r\n"
    )

    result = {"bytes_received": 0, "rtcm_frames": 0, "elapsed": 0.0, "error": None}

    try:
        with socket.create_connection((HOST, PORT), timeout=15) as s:
            s.sendall(request.encode())

            # Read and check HTTP response header
            header = b""
            while b"\r\n\r\n" not in header:
                chunk = s.recv(256)
                if not chunk:
                    break
                header += chunk

            header_text = header.decode(errors="replace")
            first_line = header_text.splitlines()[0] if header_text else ""
            # Accept: HTTP 200, ICY 200 (Ntrip/1.0), or SOURCETABLE 200
            if not any(ok in first_line for ok in ("200 OK", "ICY 200", "200")):
                result["error"] = f"Bad mountpoint response: {first_line}"
                return result

            # Strip any data already in the header buffer past \r\n\r\n
            sep = header.find(b"\r\n\r\n")
            leftover = header[sep + 4:] if sep != -1 else b""

            # Stream for duration seconds
            s.settimeout(2.0)
            start = time.monotonic()
            buf = leftover

            while time.monotonic() - start < duration:
                try:
                    chunk = s.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
                    result["bytes_received"] += len(chunk)

                    # Count RTCM3 frames (preamble 0xD3)
                    result["rtcm_frames"] += buf.count(bytes([RTCM3_PREAMBLE]))
                    buf = b""  # consumed

                    # Print a live dot every ~0.5 s
                    sys.stdout.write(".")
                    sys.stdout.flush()
                except socket.timeout:
                    continue

            result["elapsed"] = time.monotonic() - start

    except Exception as exc:
        result["error"] = str(exc)

    return result


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="EarthScope NTRIP connection tester")
    p.add_argument("--user",            help="EarthScope username")
    p.add_argument("--password",        help="EarthScope password")
    p.add_argument("--station",         default="P779",
                                        help="Station ID to look for (default: P779)")
    p.add_argument("--mountpoint",      help="Override mountpoint name directly")
    p.add_argument("--duration",        type=int, default=5,
                                        help="Seconds to stream RTCM data (default: 5)")
    p.add_argument("--sourcetable-only", action="store_true",
                                        help="Only fetch + display the sourcetable")
    args = p.parse_args()

    # Credentials
    user     = args.user     or input("EarthScope username: ").strip()
    password = args.password or getpass.getpass("EarthScope password: ")

    print(f"\n{'='*60}")
    print(f"  NTRIP Test  →  {HOST}:{PORT}")
    print(f"{'='*60}")

    # ── Step 1: Fetch sourcetable ─────────────────────────────────────────────
    print("\n[1/3] Fetching sourcetable...", end=" ", flush=True)
    try:
        lines = fetch_sourcetable(user, password)
        print("✓  Credentials accepted.")
    except ConnectionError as e:
        print(f"\n✗  FAILED: {e}")
        print("\nCheck your username/password and try again.")
        print("To request new credentials: rtgps@earthscope.org")
        sys.exit(1)

    if args.sourcetable_only:
        print(f"\nFull sourcetable ({len(lines)} lines):\n")
        for line in lines:
            print(line)
        return

    # ── Step 2: Find the station ──────────────────────────────────────────────
    station = args.station.upper()
    print(f"\n[2/3] Searching sourcetable for '{station}'...", end=" ", flush=True)
    matches = find_mountpoints(lines, station)

    if not matches:
        print(f"\n✗  Station '{station}' not found in sourcetable.")
        print("\nAll available mountpoints (STR lines):")
        for line in lines:
            if line.startswith("STR;"):
                print("  ", line[:80])
        sys.exit(1)

    print(f"✓  Found {len(matches)} stream(s):")
    for m in matches:
        fields = m.split(";")
        mp = fields[1] if len(fields) > 1 else "?"
        fmt = fields[3] if len(fields) > 3 else "?"
        print(f"    Mountpoint: {mp:30s}  Format: {fmt}")

    # Pick the mountpoint to test
    if args.mountpoint:
        mountpoint = args.mountpoint
    else:
        # Prefer RTCM3 streams
        rtcm_matches = [m for m in matches if "RTCM" in m.upper()]
        chosen = rtcm_matches[0] if rtcm_matches else matches[0]
        mountpoint = chosen.split(";")[1]

    # ── Step 3: Stream RTCM data ──────────────────────────────────────────────
    print(f"\n[3/3] Connecting to mountpoint '{mountpoint}' "
          f"for {args.duration} s...", end=" ", flush=True)
    result = stream_rtcm(user, password, mountpoint, args.duration)
    print()  # newline after dots

    if result["error"]:
        print(f"\n✗  Stream error: {result['error']}")
        if "401" in str(result["error"]):
            print()
            print("  Your credentials are valid (sourcetable access works) but")
            print("  streaming access to data mountpoints is not enabled.")
            print()
            print("  Email rtgps@earthscope.org and ask them to enable real-time")
            print("  data streaming for your account. Mention you want access to")
            print(f"  station {station} via NTRIP for a rover positioning project.")
        sys.exit(1)

    kb = result["bytes_received"] / 1024
    rate = kb / result["elapsed"] if result["elapsed"] > 0 else 0

    print(f"\n{'='*60}")
    print(f"  Result")
    print(f"{'='*60}")
    print(f"  Bytes received : {result['bytes_received']:,} ({kb:.1f} KB)")
    print(f"  RTCM3 frames   : {result['rtcm_frames']}")
    print(f"  Duration       : {result['elapsed']:.1f} s")
    print(f"  Data rate      : {rate:.2f} KB/s")

    if result["bytes_received"] > 0 and result["rtcm_frames"] > 0:
        print(f"\n  ✓  SUCCESS — {station} is live and streaming RTCM corrections.")
        print(f"\n  Use these settings in your rover config:")
        print(f"    Host      : {HOST}")
        print(f"    Port      : {PORT}")
        print(f"    Mountpoint: {mountpoint}")
        print(f"    User      : {user}")
    elif result["bytes_received"] > 0:
        print(f"\n  ⚠  Data received but no RTCM3 frames detected.")
        print(f"     The stream may be in a different format — check mountpoint type.")
    else:
        print(f"\n  ✗  No data received. Station may be offline.")
        print(f"     Check status: https://www.unavco.org/instrumentation/networks/status/nota/overview/{station}")

    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
