"""
storage/database.py — SQLite data logger for yard_mapper.

Opens a WAL-mode SQLite database and provides a single append_row() method.
WAL mode allows the database to be read from another process (e.g., a
monitoring script) without blocking writes.

Schema (one row = one lidar measurement):
    timestamp_us    INTEGER   microseconds, time.monotonic_ns() // 1000
    servo_deg       REAL      servo angle at time of measurement
    lidar_mm        INTEGER   distance in mm; -1 = invalid
    strength        INTEGER   lidar signal strength
    imu_roll_deg    REAL      mower roll (right-side-down positive)
    imu_pitch_deg   REAL      mower pitch (nose-down positive)
    imu_heading_deg REAL      magnetic heading 0–360
    gps_lat         REAL      WGS-84 latitude; NULL until GPS fitted
    gps_lon         REAL      WGS-84 longitude; NULL until GPS fitted
    gps_alt_m       REAL      ellipsoidal altitude; NULL until GPS fitted
    fix_type        INTEGER   0=none 4=RTK-fixed 5=RTK-float; NULL until GPS fitted
    hdop            REAL      horizontal dilution of precision; NULL until GPS fitted
"""

import logging
import os
import sqlite3
import time
from dataclasses import dataclass
from typing import Optional

import config

logger = logging.getLogger(__name__)

CREATE_TABLE_SQL = """
CREATE TABLE IF NOT EXISTS scans (
    id              INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp_us    INTEGER NOT NULL,
    servo_deg       REAL    NOT NULL,
    lidar_mm        INTEGER NOT NULL,
    strength        INTEGER NOT NULL,
    imu_roll_deg    REAL    NOT NULL,
    imu_pitch_deg   REAL    NOT NULL,
    imu_heading_deg REAL    NOT NULL,
    gps_lat         REAL,
    gps_lon         REAL,
    gps_alt_m       REAL,
    fix_type        INTEGER,
    hdop            REAL
);
"""

CREATE_INDEX_SQL = """
CREATE INDEX IF NOT EXISTS idx_timestamp ON scans (timestamp_us);
"""


@dataclass
class ScanRow:
    """One measurement record. GPS fields are None until Phase 3."""
    servo_deg: float
    lidar_mm: int
    strength: int
    imu_roll_deg: float
    imu_pitch_deg: float
    imu_heading_deg: float
    gps_lat: Optional[float] = None
    gps_lon: Optional[float] = None
    gps_alt_m: Optional[float] = None
    fix_type: Optional[int] = None
    hdop: Optional[float] = None


class ScanDatabase:
    """
    Thin wrapper around SQLite for logging scan data.

    Usage:
        db = ScanDatabase()
        db.open()
        db.append_row(ScanRow(...))
        db.close()

    Or as a context manager:
        with ScanDatabase() as db:
            db.append_row(...)
    """

    def __init__(self, path: str = config.DB_PATH):
        self._path = path
        self._conn: Optional[sqlite3.Connection] = None
        self._row_count = 0

    def open(self) -> None:
        """Create the database directory and open the connection."""
        os.makedirs(os.path.dirname(self._path), exist_ok=True)
        self._conn = sqlite3.connect(self._path, check_same_thread=False)
        self._conn.execute("PRAGMA journal_mode=WAL;")
        self._conn.execute("PRAGMA synchronous=NORMAL;")  # safe with WAL
        self._conn.executescript(CREATE_TABLE_SQL + CREATE_INDEX_SQL)
        self._conn.commit()
        logger.info("Database opened at %s", self._path)

    def close(self) -> None:
        if self._conn:
            self._conn.commit()
            self._conn.close()
            self._conn = None
        logger.info("Database closed. Rows written: %d", self._row_count)

    def append_row(self, row: ScanRow) -> None:
        """Append one measurement row. Thread-safe (SQLite WAL handles it)."""
        if self._conn is None:
            raise RuntimeError("Database is not open. Call open() first.")
        ts_us = time.monotonic_ns() // 1_000
        self._conn.execute(
            """
            INSERT INTO scans (
                timestamp_us, servo_deg, lidar_mm, strength,
                imu_roll_deg, imu_pitch_deg, imu_heading_deg,
                gps_lat, gps_lon, gps_alt_m, fix_type, hdop
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                ts_us,
                row.servo_deg,
                row.lidar_mm,
                row.strength,
                row.imu_roll_deg,
                row.imu_pitch_deg,
                row.imu_heading_deg,
                row.gps_lat,
                row.gps_lon,
                row.gps_alt_m,
                row.fix_type,
                row.hdop,
            ),
        )
        self._row_count += 1
        # Commit every 100 rows to balance durability vs. write speed
        if self._row_count % 100 == 0:
            self._conn.commit()

    def flush(self) -> None:
        """Force commit any pending rows."""
        if self._conn:
            self._conn.commit()

    @property
    def row_count(self) -> int:
        return self._row_count

    # ── Context manager support ───────────────────────────────────────────────

    def __enter__(self) -> "ScanDatabase":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.close()
