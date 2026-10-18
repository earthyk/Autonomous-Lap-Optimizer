#!/usr/bin/env python3
"""
create_results_csvs.py
----------------------
Creates the results/ folder and the two logging CSV files beside this script.
- If a file does not exist, it is created with the correct header row only.
- If a file already exists and has data rows, the data is preserved and the
  header is checked for compatibility.  A mismatch aborts with a clear error
  rather than silently replacing the file.
- Paths are always relative to THIS file's directory, regardless of where the
  script is invoked from.
"""

import csv
import os
import sys

# ── Resolve paths relative to this script, not the CWD ──────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(SCRIPT_DIR, "results")

LAP_DATA_PATH   = os.path.join(RESULTS_DIR, "lap_data.csv")
TRACK_DATA_PATH = os.path.join(RESULTS_DIR, "track_data.csv")

# ── Canonical headers ────────────────────────────────────────────────────────
LAP_DATA_HEADER = [
    "run_id", "timestamp_utc", "version", "track", "car",
    "lap_number", "lap_time_seconds", "damage_total", "damage_delta",
    "status", "change_description",
]

TRACK_DATA_HEADER = [
    "run_id", "timestamp_utc", "version", "track", "car",
    "lap_number", "curLapTime", "lastLapTime", "distFromStart", "distRaced",
    "speedX", "speedY", "angle", "trackPos", "damage",
    "track_0",  "track_1",  "track_2",  "track_3",  "track_4",
    "track_5",  "track_6",  "track_7",  "track_8",  "track_9",
    "track_10", "track_11", "track_12", "track_13", "track_14",
    "track_15", "track_16", "track_17", "track_18",
    "steer", "accel", "brake", "gear",
]

# ── Helpers ──────────────────────────────────────────────────────────────────
def ensure_csv(path, expected_header):
    """
    Create path with expected_header if it does not exist.
    If it already exists:
      - empty file  → write header
      - header only → verify and report
      - has data    → verify header; abort on mismatch, preserve data on match
    Returns True on success, False on incompatible-header error.
    """
    name = os.path.basename(path)
    exists = os.path.isfile(path)

    if not exists:
        with open(path, "w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(expected_header)
        print(f"  CREATED  : {path}")
        print(f"  HEADER   : {expected_header}\n")
        return True

    # File exists — read what is there
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        rows = list(reader)

    if not rows:
        # Completely empty — write header
        with open(path, "w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(expected_header)
        print(f"  REPAIRED : {path} (was empty, wrote header)")
        print(f"  HEADER   : {expected_header}\n")
        return True

    existing_header = rows[0]
    data_rows = rows[1:]

    if existing_header == expected_header:
        if data_rows:
            print(f"  EXISTS   : {path}")
            print(f"  STATUS   : Header matches. {len(data_rows)} data row(s) preserved.")
        else:
            print(f"  EXISTS   : {path}")
            print(f"  STATUS   : Header-only file, already correct.")
        print(f"  HEADER   : {existing_header}\n")
        return True
    else:
        # Incompatible — do NOT overwrite
        print(f"  ERROR    : {path}")
        print(f"  STATUS   : Header MISMATCH — file NOT modified.")
        print(f"  EXPECTED : {expected_header}")
        print(f"  FOUND    : {existing_header}")
        missing = [c for c in expected_header if c not in existing_header]
        extra   = [c for c in existing_header if c not in expected_header]
        if missing: print(f"  MISSING cols : {missing}")
        if extra:   print(f"  EXTRA cols   : {extra}")
        print()
        return False

# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    print(f"\nScript dir  : {SCRIPT_DIR}")
    print(f"Results dir : {RESULTS_DIR}\n")

    # 1. Create results/ if needed
    if not os.path.isdir(RESULTS_DIR):
        os.makedirs(RESULTS_DIR)
        print(f"  CREATED  : {RESULTS_DIR}\n")
    else:
        print(f"  EXISTS   : {RESULTS_DIR}\n")

    # 2. Process each CSV
    ok1 = ensure_csv(LAP_DATA_PATH,   LAP_DATA_HEADER)
    ok2 = ensure_csv(TRACK_DATA_PATH, TRACK_DATA_HEADER)

    # 3. Summary
    if ok1 and ok2:
        print("All done. Both CSV files are ready for logging.\n")
        print("Next step: connect automatic telemetry logging to these files.")
        print("  Creating empty CSVs alone does not capture any telemetry.\n")
    else:
        print("One or more files have incompatible headers.")
        print("Inspect the files listed above before proceeding.\n")
        sys.exit(1)

if __name__ == "__main__":
    main()
