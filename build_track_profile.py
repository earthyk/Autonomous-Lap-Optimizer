#!/usr/bin/env python
# build_track_profile.py
#
# PURPOSE
#   Reads results/track_data.csv, selects the one verified complete zero-damage
#   lap, bins the telemetry by distFromStart, and writes results/track_profile.csv.
#
# ACCEPTANCE CRITERIA (all must pass or the script aborts with no output file):
#   1. Exactly one run in lap_data.csv has status='completed' AND damage_delta=0.0
#      AND a confirmed lap-boundary crossing (curLapTime reset) in track_data.csv.
#   2. track_label and car_label are noted as metadata (cannot be validated vs TORCS).
#   3. No fabrication: bins with fewer than MIN_SAMPLES samples get coverage_ok=0.
#      Bins with coverage_ok=0 are written but the driver treats them as gaps.
#
# OUTPUT FORMAT
#   results/track_profile.csv
#   Preamble lines start with '#' and carry metadata key=value pairs.
#   Data lines are standard CSV with a header row.
#
# CONSERVATIVE SPEED FORMULA
#   speed_conservative_kmh = min(speed_p10_kmh, BASE_TARGET) * CONSERVATISM
#   where BASE_TARGET = 90 km/h (the V2 straight-line target),
#   CONSERVATISM = 0.90 (10% safety margin below observed p10).
#   This means the profile never raises speed above V2's own reactive limit;
#   it can only lower it.
#
# BIN WIDTH
#   BIN_M = 50 metres.  At 5 Hz sampling and ~90 km/h that is ~2-3 samples/bin
#   on average; sections with more overlap (start/finish area) get more.
#   MIN_SAMPLES = 2 is therefore the minimum accepted coverage threshold.
#
# LIMITATIONS (do not remove this block)
#   - Source lap uses track='unknown', car='unknown'; cannot confirm TORCS match.
#   - track_9 never dropped below 99 m in the source lap: the profile has no
#     speed data for tight corners.  The driver falls back to live sensors there.
#   - Profile is a static snapshot; it does not update during a run.
#   - Conservative targets here are what the reactive car happened to drive,
#     not optimal racing lines.

import csv
import os
import sys
from collections import defaultdict

# ---- Configuration -----------------------------------------------------------
TRACK_DATA_CSV  = os.path.join('results', 'track_data.csv')
LAP_DATA_CSV    = os.path.join('results', 'lap_data.csv')
PROFILE_CSV     = os.path.join('results', 'track_profile.csv')

BIN_M           = 50        # distance bin width (metres)
MIN_SAMPLES     = 2         # minimum samples for coverage_ok=1
BASE_TARGET     = 90.0      # km/h — V2 reactive straight-line target
CONSERVATISM    = 0.90      # fraction applied to observed p10

# ---- Step 1: find the accepted source run ------------------------------------

def load_lap_data(path):
    rows = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows

def find_source_run(lap_rows):
    """Return (run_id, lap_time, damage_delta) or raise ValueError with reason."""
    completed = []
    for r in lap_rows:
        if r['status'] == 'completed':
            try:
                dmg = float(r['damage_delta']) if r['damage_delta'] else None
            except ValueError:
                dmg = None
            try:
                lt = float(r['lap_time_seconds']) if r['lap_time_seconds'] else None
            except ValueError:
                lt = None
            completed.append((r['run_id'], lt, dmg, r))
    if not completed:
        raise ValueError('No run with status=completed in %s' % path)
    # Filter: damage_delta must be exactly 0.0
    zero_dmg = [(rid, lt, dmg, row) for rid, lt, dmg, row in completed
                if dmg is not None and dmg == 0.0]
    if not zero_dmg:
        raise ValueError(
            'No completed run has damage_delta=0.0. '
            'Completed runs: %s' % [x[0][:8] for x in completed])
    if len(zero_dmg) > 1:
        # Prefer the one with the best (lowest) lap time; if equal, take earliest
        zero_dmg.sort(key=lambda x: (x[1] if x[1] is not None else 9999))
        print('INFO: %d completed zero-damage runs; selecting fastest: %s (%.3f s)'
              % (len(zero_dmg), zero_dmg[0][0][:8], zero_dmg[0][1] or 0))
    rid, lt, dmg, row = zero_dmg[0]
    return rid, lt, dmg, row

# ---- Step 2: verify lap-boundary crossing in track_data ----------------------

def verify_lap_boundary(track_rows, run_id):
    """
    Return track_length_m (float) if a curLapTime reset is detected.
    The lap length is the distRaced value at the reset row minus the distRaced
    at the first real data row of the run.
    """
    run_rows = [r for r in track_rows if r['run_id'] == run_id]
    if not run_rows:
        raise ValueError('run_id %s not found in track_data.csv' % run_id[:8])

    # Find first row with distRaced > 0 (skip pre-race static rows)
    start_dr = None
    for r in run_rows:
        dr = float(r['distRaced'])
        if dr > 0:
            start_dr = dr
            break
    if start_dr is None:
        raise ValueError('No distRaced > 0 found for run %s' % run_id[:8])

    prev_lt = None
    for i, r in enumerate(run_rows):
        lt = float(r['curLapTime'])
        dr = float(r['distRaced'])
        if prev_lt is not None and lt < prev_lt - 5.0:
            # curLapTime reset: this is the lap boundary
            track_length_m = dr - start_dr
            if track_length_m <= 100:
                raise ValueError(
                    'Lap boundary found but implied track_length=%.1f m is implausible'
                    % track_length_m)
            print('INFO: Lap boundary confirmed at row %d, distRaced=%.1f m, '
                  'track_length=%.1f m' % (i, dr, track_length_m))
            return track_length_m
        prev_lt = lt

    raise ValueError(
        'No curLapTime reset (lap boundary) found for run %s. '
        'This run did not complete a lap.' % run_id[:8])

# ---- Step 3: extract the single lap's telemetry rows -------------------------

def extract_lap_rows(track_rows, run_id):
    """
    Return only the rows from the completed lap (lap_number='1' or before
    the reset, where distFromStart is increasing from 0).
    We use lap_number='0' rows up to and including the last row before the
    curLapTime reset, since that is the lap we timed.
    """
    run_rows = [r for r in track_rows if r['run_id'] == run_id]

    # Find the index of the lap boundary
    prev_lt = None
    boundary_idx = None
    for i, r in enumerate(run_rows):
        lt = float(r['curLapTime'])
        if prev_lt is not None and lt < prev_lt - 5.0:
            boundary_idx = i
            break
        prev_lt = lt

    if boundary_idx is None:
        raise ValueError('Could not re-locate lap boundary in extract step.')

    # Determine a distRaced threshold to exclude the opening-straight
    # acceleration rows.  The car spawns at the start/finish line; the
    # pre-race + first-few-seconds rows have the SAME distFromStart as the
    # late-lap rows approaching the finish.  Mixing them corrupts those bins.
    # We skip any row with distRaced < BIN_M * 2 (100 m past first motion).
    first_real_dr = None
    for r in run_rows:
        dr = float(r['distRaced'])
        if dr > 0:
            first_real_dr = dr
            break
    dr_threshold = (first_real_dr or 0.0) + BIN_M * 2

    # Include rows up to (not including) the reset row.
    # Exclude: pre-race countdown (curLapTime < 0), negative distFromStart,
    #          and opening-straight rows (distRaced < dr_threshold).
    lap_rows = []
    for r in run_rows[:boundary_idx]:
        lt = float(r['curLapTime'])
        if lt < 0:
            continue  # pre-race countdown
        ds = float(r['distFromStart'])
        if ds < 0:
            continue
        dr = float(r['distRaced'])
        if dr < dr_threshold:
            continue  # opening-straight acceleration; skip to avoid bin contamination
        lap_rows.append(r)

    print('INFO: %d telemetry rows extracted for the completed lap.' % len(lap_rows))
    return lap_rows

# ---- Step 4: build bins ------------------------------------------------------

def percentile(data, pct):
    """Simple percentile without numpy."""
    if not data:
        return None
    s = sorted(data)
    k = (len(s) - 1) * pct / 100.0
    lo = int(k)
    hi = lo + 1
    if hi >= len(s):
        return s[lo]
    frac = k - lo
    return s[lo] + frac * (s[hi] - s[lo])

def build_bins(lap_rows, track_length_m, bin_m):
    n_bins = int(track_length_m / bin_m) + 1
    bins = defaultdict(list)
    for r in lap_rows:
        ds = float(r['distFromStart'])
        b  = int(ds / bin_m)
        bins[b].append(float(r['speedX']))

    result = []
    for b in range(n_bins):
        bin_start = b * bin_m
        bin_end   = bin_start + bin_m
        speeds    = bins[b]
        n         = len(speeds)
        if n >= MIN_SAMPLES:
            p10  = percentile(speeds, 10)
            p50  = percentile(speeds, 50)
            cons = min(p10, BASE_TARGET) * CONSERVATISM
            coverage_ok = 1
        else:
            p10  = None
            p50  = None
            cons = None
            coverage_ok = 0
        result.append({
            'bin_start_m':            bin_start,
            'bin_end_m':              bin_end,
            'sample_count':           n,
            'speed_p50_kmh':          ('%.2f' % p50)  if p50  is not None else '',
            'speed_p10_kmh':          ('%.2f' % p10)  if p10  is not None else '',
            'speed_conservative_kmh': ('%.2f' % cons) if cons is not None else '',
            'coverage_ok':            coverage_ok,
        })
    return result

# ---- Step 5: write profile CSV -----------------------------------------------

def write_profile(path, bins, meta):
    with open(path, 'w', newline='') as f:
        # Metadata preamble
        for k, v in meta.items():
            f.write('# %s=%s\n' % (k, v))
        f.write('#\n')
        f.write('# speed_conservative_kmh = min(speed_p10_kmh, %.1f) * %.2f\n'
                % (BASE_TARGET, CONSERVATISM))
        f.write('# Bins with coverage_ok=0 are gaps; driver must fall back to reactive.\n')
        f.write('# Do not edit this file manually; regenerate with build_track_profile.py.\n')
        f.write('#\n')
        # Data
        fieldnames = [
            'bin_start_m', 'bin_end_m', 'sample_count',
            'speed_p50_kmh', 'speed_p10_kmh', 'speed_conservative_kmh',
            'coverage_ok',
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(bins)
    print('INFO: Wrote %d bins to %s' % (len(bins), path))

# ---- Main --------------------------------------------------------------------

def main():
    # Validate input files exist
    for p in (TRACK_DATA_CSV, LAP_DATA_CSV):
        if not os.path.exists(p):
            print('ERROR: Required file not found: %s' % p)
            sys.exit(1)

    # Load data
    lap_rows   = load_lap_data(LAP_DATA_CSV)
    track_rows = []
    with open(TRACK_DATA_CSV, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            track_rows.append(row)
    print('INFO: Loaded %d lap rows, %d track rows.' % (len(lap_rows), len(track_rows)))

    # Find source run
    try:
        run_id, lap_time, dmg_delta, lap_meta = find_source_run(lap_rows)
    except ValueError as e:
        print('REJECTED: %s' % e)
        sys.exit(1)
    print('INFO: Accepted source run: %s  lap_time=%.3f s  damage_delta=%.1f'
          % (run_id[:8], lap_time, dmg_delta))

    # Verify lap boundary in track data
    try:
        track_length_m = verify_lap_boundary(track_rows, run_id)
    except ValueError as e:
        print('REJECTED: %s' % e)
        sys.exit(1)

    # Extract completed-lap rows
    try:
        lap_telemetry = extract_lap_rows(track_rows, run_id)
    except ValueError as e:
        print('REJECTED: %s' % e)
        sys.exit(1)

    # Check for minimum coverage
    if len(lap_telemetry) < 50:
        print('REJECTED: Only %d telemetry rows — insufficient coverage.'
              % len(lap_telemetry))
        sys.exit(1)

    # Check damage in every row (belt-and-suspenders)
    max_dmg = max(float(r['damage']) for r in lap_telemetry)
    if max_dmg > 0.0:
        print('REJECTED: damage > 0 detected in telemetry rows (max=%.1f).' % max_dmg)
        sys.exit(1)

    # Build bins
    bins = build_bins(lap_telemetry, track_length_m, BIN_M)
    n_gap  = sum(1 for b in bins if b['coverage_ok'] == 0)
    n_ok   = sum(1 for b in bins if b['coverage_ok'] == 1)
    print('INFO: %d bins total — %d covered, %d gaps (coverage_ok=0).'
          % (len(bins), n_ok, n_gap))

    # Collect metadata
    meta = {
        'source_run_id':       run_id,
        'source_version':      lap_meta.get('version', 'unknown'),
        'source_track_label':  lap_meta.get('track', 'unknown'),
        'source_car_label':    lap_meta.get('car', 'unknown'),
        'source_lap_time_s':   '%.3f' % lap_time,
        'source_damage_delta': '%.1f' % dmg_delta,
        'track_length_m':      '%.1f' % track_length_m,
        'bin_width_m':         str(BIN_M),
        'min_samples':         str(MIN_SAMPLES),
        'base_target_kmh':     '%.1f' % BASE_TARGET,
        'conservatism':        '%.2f' % CONSERVATISM,
        'n_bins_covered':      str(n_ok),
        'n_bins_gap':          str(n_gap),
        'generated_by':        'build_track_profile.py',
        'warning_track_label': (
            'unknown — cannot confirm this profile matches current TORCS config'),
        'warning_corner_data': (
            'track_9 never below 99m in source lap — no tight-corner speed data exists'),
    }

    # Write
    os.makedirs(os.path.dirname(PROFILE_CSV), exist_ok=True)
    write_profile(PROFILE_CSV, bins, meta)
    print('DONE.')

if __name__ == '__main__':
    main()
