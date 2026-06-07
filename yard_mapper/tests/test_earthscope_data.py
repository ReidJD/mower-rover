#!/usr/bin/env python3
"""
test_earthscope_data.py — Test EarthScope RINEX archive access.

Uses the earthscope-sdk for authentication (OAuth2 device code flow).
You must run `es login` once before using this script.

Install:
    pip install earthscope-sdk

Login (do this once):
    es login
    # Follow the URL it prints, approve in browser

Usage:
    python test_earthscope_data.py
    python test_earthscope_data.py --station P779 --days-back 3
    python test_earthscope_data.py --download   # actually save the file

PPK workflow summary:
    1. During mowing, ZED-F9P logs raw observations (UBX format)
    2. After session, run this script to download P779 RINEX for that day
    3. Post-process: rtkpost rover.ubx + P779.crx.gz → positions.pos
    4. Import positions.pos into yard mapper pipeline
"""

import argparse
import datetime
import sys

ARCHIVE_BASE = "https://gage-data.earthscope.org/archive/gnss"

# RINEX 3 daily: SSSS00USA_R_YYYYDDD0000_01D_30S_MO.crx.gz
# RINEX 2 daily: ssssddds.yyo.gz
VARIANTS = [
    ("RINEX3", "rinex3/obs", lambda sta, yr, doy:
        f"{sta.upper()}00USA_R_{yr}{doy:03d}0000_01D_30S_MO.crx.gz"),
    ("RINEX2", "rinex/obs",  lambda sta, yr, doy:
        f"{sta.lower()}{doy:03d}0.{str(yr)[-2:]}o.gz"),
]


def doy(d: datetime.date) -> int:
    return d.timetuple().tm_yday


def find_file(client, station: str, dates: list[datetime.date]) -> tuple[str, str] | None:
    """Return (url, filename) for the first accessible file, or None."""
    for label, subpath, fname_fn in VARIANTS:
        print(f"\nTrying {label}...")
        for d in dates:
            yr  = d.year
            day = doy(d)
            fname = fname_fn(station, yr, day)
            url   = f"{ARCHIVE_BASE}/{subpath}/{yr}/{day:03d}/{fname}"
            print(f"  {d} (DOY {day:03d})  {fname}  ", end="", flush=True)
            try:
                resp = client.get(url)
                if resp.status_code == 200:
                    print(f"HTTP 200 ✓")
                    return url, fname
                else:
                    print(f"HTTP {resp.status_code}")
            except Exception as e:
                print(f"Error: {e}")
    return None


def main():
    p = argparse.ArgumentParser(description="EarthScope RINEX archive tester")
    p.add_argument("--station",   default="P779")
    p.add_argument("--days-back", type=int, default=2,
                   help="How many days back to check (default: 2)")
    p.add_argument("--download",  action="store_true",
                   help="Actually download the file to the current directory")
    args = p.parse_args()

    # ── Import SDK ────────────────────────────────────────────────────────────
    try:
        from earthscope_sdk import EarthScopeClient
    except ImportError:
        print("earthscope-sdk not installed. Run:")
        print("  pip install earthscope-sdk")
        sys.exit(1)

    # ── Build date list ───────────────────────────────────────────────────────
    today  = datetime.date.today()
    # Today's file usually isn't posted yet; start from yesterday
    dates  = [today - datetime.timedelta(days=i) for i in range(1, args.days_back + 3)]
    station = args.station.upper()

    print(f"\n{'='*60}")
    print(f"  EarthScope RINEX Archive Test  —  station {station}")
    print(f"{'='*60}")

    # ── Connect (uses stored token from `es login`) ───────────────────────────
    try:
        client = EarthScopeClient()
    except Exception as e:
        print(f"\n✗  Could not create EarthScope client: {e}")
        print("\nMake sure you've run:  es login")
        sys.exit(1)

    with client:
        result = find_file(client, station, dates)

        print(f"\n{'='*60}")
        if result is None:
            print(f"""
  ✗  No accessible RINEX files found for {station}.

  Possible causes:
  • File exists but naming convention differs — browse manually:
      https://gage-data.earthscope.org/archive/gnss/rinex3/obs/
  • Archive access not yet enabled on your account
  • Station ID wrong (check: https://www.unavco.org/instrumentation/
      networks/status/nota/overview/{station})

  Email data-help@earthscope.org if you think access should be enabled.
""")
            sys.exit(1)

        url, fname = result
        print(f"""
  ✓  Archive access confirmed!

  File : {fname}
  URL  : {url}
""")

        if args.download:
            print(f"  Downloading {fname} ...", end=" ", flush=True)
            try:
                resp = client.get(url)
                resp.raise_for_status()
                with open(fname, "wb") as f:
                    f.write(resp.content)
                size_kb = len(resp.content) / 1024
                print(f"done ({size_kb:.0f} KB)")
                print(f"  Saved to: ./{fname}")
            except Exception as e:
                print(f"failed: {e}")
        else:
            print(f"  To download this file, re-run with --download")
            print(f"  Or on the Pi:")
            print(f"    python test_earthscope_data.py --download")

        print(f"""
  PPK WORKFLOW:
    1. ZED-F9P rover logs raw observations during mowing (UBX format)
    2. Download matching RINEX:
         python test_earthscope_data.py --download
    3. Post-process with RTKLIB:
         rtkpost -rover rover.ubx -base {fname} -o positions.pos
    4. Import positions.pos into yard mapper pipeline
""")
        print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
