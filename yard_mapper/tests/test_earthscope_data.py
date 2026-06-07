#!/usr/bin/env python3
"""
test_earthscope_data.py — Test EarthScope RINEX archive access.

Uses the earthscope-sdk for authentication (OAuth2 device code flow).
Set ES_OAUTH2__REFRESH_TOKEN env var before running.

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
import asyncio
import datetime
import sys

ARCHIVE_BASE = "https://gage-data.earthscope.org/archive/gnss"

VARIANTS = [
    ("RINEX3", "rinex3/obs", lambda sta, yr, doy:
        f"{sta.upper()}00USA_R_{yr}{doy:03d}0000_01D_30S_MO.crx.gz"),
    ("RINEX2", "rinex/obs",  lambda sta, yr, doy:
        f"{sta.lower()}{doy:03d}0.{str(yr)[-2:]}o.gz"),
]


def doy(d: datetime.date) -> int:
    return d.timetuple().tm_yday


async def find_file(client, station: str, dates: list) -> tuple | None:
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
                resp = await client.ctx.httpx_client.get(url)
                if resp.status_code == 200:
                    print("HTTP 200 ✓")
                    return url, fname
                else:
                    print(f"HTTP {resp.status_code}")
            except Exception as e:
                print(f"Error: {e}")
    return None


async def run(args):
    try:
        from earthscope_sdk import AsyncEarthScopeClient
    except ImportError:
        print("earthscope-sdk not installed. Run:  pip install earthscope-sdk")
        sys.exit(1)

    today   = datetime.date.today()
    dates   = [today - datetime.timedelta(days=i) for i in range(1, args.days_back + 3)]
    station = args.station.upper()

    print(f"\n{'='*60}")
    print(f"  EarthScope RINEX Archive Test  —  station {station}")
    print(f"{'='*60}")

    async with AsyncEarthScopeClient() as client:
        result = await find_file(client, station, dates)

        print(f"\n{'='*60}")
        if result is None:
            print(f"""
  ✗  No accessible RINEX files found for {station}.

  Possible causes:
  • File naming differs — browse manually:
      https://gage-data.earthscope.org/archive/gnss/rinex3/obs/
  • Archive access not yet enabled on your account

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
                resp = await client.ctx.httpx_client.get(url)
                resp.raise_for_status()
                with open(fname, "wb") as f:
                    f.write(resp.content)
                size_kb = len(resp.content) / 1024
                print(f"done ({size_kb:.0f} KB)")
                print(f"  Saved to: ./{fname}")
            except Exception as e:
                print(f"failed: {e}")
        else:
            print(f"  Re-run with --download to save the file.")

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


def main():
    p = argparse.ArgumentParser(description="EarthScope RINEX archive tester")
    p.add_argument("--station",   default="P779")
    p.add_argument("--days-back", type=int, default=2)
    p.add_argument("--download",  action="store_true",
                   help="Actually download the file")
    args = p.parse_args()
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
