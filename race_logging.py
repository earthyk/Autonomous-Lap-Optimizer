#!/usr/bin/env python3
"""
race_logging.py  –  Reusable logging helpers for the TORCS snakeoil client.

Responsibilities
----------------
* Ensure results/ directory and the two CSV files exist with correct headers.
* Validate existing headers rather than silently overwriting them.
* Log per-step track_data rows at ~5 Hz (based on advancing curLapTime,
  not assumed packet rate).
* Detect completed laps from curLapTime reset + forward progress guard;
  record lastLapTime (from server), damage delta, status.
* Record an incomplete attempt row on any exit path.
* Flush and close files on every exit path (Ctrl+C, server shutdown, error).

Lap-detection algorithm
-----------------------
A lap crossing is triggered when curLapTime resets to a value smaller than
the previous sample's curLapTime by more than LAP_RESET_THRESHOLD seconds.
Additional guards:
  - distRaced must have advanced by at least MIN_LAP_DIST metres since the
    last crossing (rules out spawn-near-start, restarts and reconnects).
  - The car must have been moving forward (at least one positive speedX sample
    since the last crossing).
  - A reverse crossing (car going backwards through start) is logged as
    status='reverse_crossing' and does NOT increment the lap counter.
  - lastLapTime <= 0 after a reset is logged as status='ambiguous'.
  - Equal consecutive lastLapTime values are accepted as valid (spec allows).
  - Restarts / reconnects reset all lap state so they are not counted.

Track-data sampling
-------------------
Sampled when curLapTime has advanced by at least SAMPLE_INTERVAL_S since the
last sample for the current lap.  If curLapTime resets (new lap), the sample
clock resets too.

Standard-library only; no third-party packages required.
"""

import csv
import os
import sys
import uuid
import datetime

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR  = os.path.join(SCRIPT_DIR, 'results')
LAP_CSV      = os.path.join(RESULTS_DIR, 'lap_data.csv')
TRACK_CSV    = os.path.join(RESULTS_DIR, 'track_data.csv')

# Sensor angles in the order they are sent by setup_connection; stored as
# metadata in comments; also listed here so callers can document them.
TRACK_SENSOR_ANGLES = [-45, -19, -12, -7, -4, -2.5, -1.7, -1, -0.5,
                        0, 0.5, 1, 1.7, 2.5, 4, 7, 12, 19, 45]

LAP_HEADER = [
    'run_id', 'timestamp_utc', 'version', 'track', 'car',
    'lap_number', 'lap_time_seconds', 'damage_total', 'damage_delta',
    'status', 'change_description',
    'max_speed_kmh', 'min_corner_speed_kmh',
]

TRACK_HEADER = [
    'run_id', 'timestamp_utc', 'version', 'track', 'car',
    'lap_number', 'curLapTime', 'lastLapTime', 'distFromStart', 'distRaced',
    'speedX', 'speedY', 'angle', 'trackPos', 'damage',
    'track_0',  'track_1',  'track_2',  'track_3',  'track_4',
    'track_5',  'track_6',  'track_7',  'track_8',  'track_9',
    'track_10', 'track_11', 'track_12', 'track_13', 'track_14',
    'track_15', 'track_16', 'track_17', 'track_18',
    'steer', 'accel', 'brake', 'gear',
    'rpm', 'corner_target', 'overspeed_kmh', 'brake_active',
]

# Lap detection / sampling parameters
LAP_RESET_THRESHOLD  = 5.0    # seconds; curLapTime drop larger than this = new lap
MIN_LAP_DIST         = 100.0  # metres; minimum distRaced advance to count a crossing
SAMPLE_INTERVAL_S    = 0.20   # seconds of sim lap time between track_data rows (~5 Hz)

# ---------------------------------------------------------------------------
# CSV file setup
# ---------------------------------------------------------------------------

def _ensure_results_dir():
    """Create results/ directory beside this script if absent."""
    if not os.path.isdir(RESULTS_DIR):
        try:
            os.makedirs(RESULTS_DIR)
        except OSError as exc:
            sys.exit('FATAL: cannot create results directory %s: %s' % (RESULTS_DIR, exc))


def _open_csv(path, expected_header):
    """
    Open *path* for appending.  If the file is new or empty, write the header.
    If it already exists with data, verify the header and abort on mismatch.

    Returns an open file handle (append mode) and a csv.writer bound to it.
    The caller is responsible for closing.
    """
    is_new = not os.path.isfile(path) or os.path.getsize(path) == 0

    if not is_new:
        # Read the existing first line to check the header.
        try:
            with open(path, 'r', newline='', encoding='utf-8') as fh:
                existing = next(csv.reader(fh), [])
        except OSError as exc:
            sys.exit('FATAL: cannot read %s: %s' % (path, exc))

        if existing != expected_header:
            missing = [c for c in expected_header if c not in existing]
            extra   = [c for c in existing if c not in expected_header]
            msg = ('FATAL: Header mismatch in %s.\n'
                   '  Expected : %s\n'
                   '  Found    : %s\n'
                   '  Missing  : %s\n'
                   '  Extra    : %s\n'
                   'File NOT modified. Resolve manually before running.'
                   % (path, expected_header, existing, missing, extra))
            sys.exit(msg)

    try:
        fh = open(path, 'a', newline='', encoding='utf-8')
    except OSError as exc:
        sys.exit('FATAL: cannot open %s for appending: %s' % (path, exc))

    writer = csv.writer(fh)
    if is_new:
        try:
            writer.writerow(expected_header)
            fh.flush()
        except OSError as exc:
            sys.exit('FATAL: cannot write header to %s: %s' % (path, exc))

    return fh, writer


def _safe_writerow(writer, fh, row):
    """Write a row and flush; abort with actionable message on failure."""
    try:
        writer.writerow(row)
        fh.flush()
    except OSError as exc:
        sys.exit('FATAL: write failed to %s: %s\nNo silent data loss.' % (fh.name, exc))


# ---------------------------------------------------------------------------
# Public helper: RaceLogger
# ---------------------------------------------------------------------------

class RaceLogger:
    """
    Attach one instance of this to a snakeoil Client run.

    Usage (inside the main loop, after drive_example and before
    respond_to_server so the clipped actions are already set):

        logger = RaceLogger(
            version='v1',
            track_label=C.trackname,
            car_label='unknown',
            change_description='baseline – drive_example unchanged',
        )
        # ... main loop ...
            logger.on_step(C)
        # ... on exit ...
        logger.on_finish(status='step_limit')
    """

    def __init__(self, version, track_label, car_label, change_description):
        _ensure_results_dir()

        self.run_id      = str(uuid.uuid4())
        self.ts_utc      = datetime.datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ')
        self.version     = version
        self.track_label = track_label if track_label else 'unknown'
        self.car_label   = car_label   if car_label   else 'unknown'
        self.change_desc = change_description if change_description else 'unknown'

        self._lap_fh,   self._lap_w   = _open_csv(LAP_CSV,   LAP_HEADER)
        self._track_fh, self._track_w = _open_csv(TRACK_CSV, TRACK_HEADER)

        # Lap-detection state
        self._lap_number         = 0      # laps completed so far
        self._prev_cur_lap_time  = None   # curLapTime at previous step
        self._last_crossing_dist = 0.0    # distRaced at last valid crossing
        self._seen_forward       = False  # any positive speedX since last crossing?
        self._prev_damage        = 0.0    # damage at start of current lap
        self._closed             = False

        # Per-lap speed trackers (reset each lap boundary)
        self._lap_max_speed = 0.0    # highest speedX seen this lap
        self._lap_min_speed = 999.0  # lowest speedX seen while moving (>10 km/h)

        # Track-data sampling state
        self._last_sample_lap_time = None  # curLapTime of last sample

        print('[race_logging] run_id=%s  version=%s  track=%s  car=%s'
              % (self.run_id, self.version, self.track_label, self.car_label))
        print('[race_logging] Sensor angles: %s' % TRACK_SENSOR_ANGLES)
        print('[race_logging] lap_data  -> %s' % LAP_CSV)
        print('[race_logging] track_data-> %s' % TRACK_CSV)

    # ------------------------------------------------------------------
    # Per-step call (called once per simulation packet)
    # ------------------------------------------------------------------

    def on_step(self, client, rpm=None, corner_target=None,
                overspeed_kmh=None, brake_active=None):
        """
        Call after drive_example(client) so that steer/accel/brake/gear
        reflect the clipped values that will actually be sent.

        Optional kwargs (pass from drive_example for richer diagnostics):
          rpm           – engine RPM from S['rpm']
          corner_target – the speed limit (km/h) computed by the throttle logic
          overspeed_kmh – speedX minus corner_target (negative = under limit)
          brake_active  – 1 if the prospective-braking block fired, else 0
        """
        if self._closed:
            return

        S = client.S.d
        R = client.R.d

        cur  = float(S.get('curLapTime',  0.0) or 0.0)
        last = float(S.get('lastLapTime', 0.0) or 0.0)
        dist = float(S.get('distRaced',   0.0) or 0.0)
        spx  = float(S.get('speedX',      0.0) or 0.0)
        dmg  = float(S.get('damage',      0.0) or 0.0)

        # Resolve optional diagnostics — fall back to sensor dict if not passed
        _rpm           = rpm           if rpm           is not None else float(S.get('rpm', 0.0) or 0.0)
        _corner_target = corner_target if corner_target is not None else 0.0
        _overspeed     = overspeed_kmh if overspeed_kmh is not None else 0.0
        _brake_active  = brake_active  if brake_active  is not None else 0

        # Track forward movement
        if spx > 0.5:
            self._seen_forward = True

        # Update per-lap speed trackers
        if spx > self._lap_max_speed:
            self._lap_max_speed = spx
        # min_corner_speed: only track when the car is actually moving (>10 km/h)
        # so the stationary pre-race countdown doesn't set it to ~0.
        if spx > 10.0 and spx < self._lap_min_speed:
            self._lap_min_speed = spx

        # ---- Lap detection -----------------------------------------------
        if self._prev_cur_lap_time is not None:
            drop = self._prev_cur_lap_time - cur
            if drop > LAP_RESET_THRESHOLD:
                self._handle_crossing(cur, last, dist, spx, dmg)

        self._prev_cur_lap_time = cur

        # ---- Track-data sampling -----------------------------------------
        self._maybe_sample(client, S, R, cur, last, dist, dmg, spx,
                           _rpm, _corner_target, _overspeed, _brake_active)

    # ------------------------------------------------------------------
    # Lap-crossing handler
    # ------------------------------------------------------------------

    def _handle_crossing(self, cur, last, dist, spx, dmg):
        dist_since_last = dist - self._last_crossing_dist

        # Guard: not enough distance driven → spawn/restart/reconnect
        if dist_since_last < MIN_LAP_DIST:
            print('[race_logging] Crossing ignored: only %.1f m since last '
                  'crossing (min=%.0f) – likely spawn/restart.'
                  % (dist_since_last, MIN_LAP_DIST))
            # Reset sampling clock for new lap without counting it
            self._last_sample_lap_time = None
            return

        # Guard: reverse crossing
        if not self._seen_forward:
            status = 'reverse_crossing'
            print('[race_logging] Reverse crossing detected at dist=%.1f – not counted.' % dist)
            self._write_lap_row(status, last, dmg)
            self._reset_lap_state(dist, dmg)
            return

        # Guard: ambiguous (server returned lastLapTime <= 0 after reset)
        if last <= 0.0:
            status = 'ambiguous'
            print('[race_logging] Ambiguous crossing (lastLapTime=%.3f) at dist=%.1f'
                  % (last, dist))
            self._write_lap_row(status, last, dmg)
            self._reset_lap_state(dist, dmg)
            return

        # Valid completed lap
        self._lap_number += 1
        status = 'completed'
        print('[race_logging] Lap %d completed. lastLapTime=%.3f s  damage_delta=%.0f'
              % (self._lap_number, last, dmg - self._prev_damage))
        self._write_lap_row(status, last, dmg)
        self._reset_lap_state(dist, dmg)

    def _write_lap_row(self, status, last_lap_time, dmg_now):
        dmg_delta  = dmg_now - self._prev_damage
        lap_time   = last_lap_time if last_lap_time > 0.0 else ''
        max_spd    = round(self._lap_max_speed, 2) if self._lap_max_speed > 0 else ''
        min_spd    = round(self._lap_min_speed, 2) if self._lap_min_speed < 999.0 else ''
        row = [
            self.run_id, self.ts_utc, self.version,
            self.track_label, self.car_label,
            self._lap_number,   # 0 for anomalous crossings that were not counted
            lap_time,
            dmg_now,
            dmg_delta,
            status,
            self.change_desc,
            max_spd,
            min_spd,
        ]
        _safe_writerow(self._lap_w, self._lap_fh, row)

    def _reset_lap_state(self, dist, dmg):
        self._last_crossing_dist   = dist
        self._seen_forward         = False
        self._prev_damage          = dmg
        self._last_sample_lap_time = None  # reset sample clock for new lap
        self._lap_max_speed        = 0.0
        self._lap_min_speed        = 999.0

    # ------------------------------------------------------------------
    # Track-data sampling
    # ------------------------------------------------------------------

    def _maybe_sample(self, client, S, R, cur, last, dist, dmg, spx,
                      rpm=0.0, corner_target=0.0, overspeed_kmh=0.0,
                      brake_active=0):
        # Reset sample clock when lap resets (handled by _reset_lap_state
        # which sets _last_sample_lap_time = None).
        if self._last_sample_lap_time is None:
            self._last_sample_lap_time = cur
            # Sample immediately on the first packet of a lap
            self._write_track_row(S, R, cur, last, dist, dmg, spx,
                                  rpm, corner_target, overspeed_kmh,
                                  brake_active)
            return

        if cur - self._last_sample_lap_time >= SAMPLE_INTERVAL_S:
            self._last_sample_lap_time = cur
            self._write_track_row(S, R, cur, last, dist, dmg, spx,
                                  rpm, corner_target, overspeed_kmh,
                                  brake_active)

    def _write_track_row(self, S, R, cur, last, dist, dmg, spx,
                         rpm=0.0, corner_target=0.0, overspeed_kmh=0.0,
                         brake_active=0):
        track_sensors = S.get('track', [0.0] * 19)
        if not isinstance(track_sensors, list):
            track_sensors = [track_sensors]
        # Pad or trim to exactly 19 values
        track_sensors = list(track_sensors) + [0.0] * 19
        track_sensors = track_sensors[:19]

        row = [
            self.run_id, self.ts_utc, self.version,
            self.track_label, self.car_label,
            self._lap_number,
            cur,
            last,
            float(S.get('distFromStart', 0.0) or 0.0),
            dist,
            spx,
            float(S.get('speedY',   0.0) or 0.0),
            float(S.get('angle',    0.0) or 0.0),
            float(S.get('trackPos', 0.0) or 0.0),
            dmg,
        ] + track_sensors + [
            R.get('steer',  0.0),
            R.get('accel',  0.0),
            R.get('brake',  0.0),
            R.get('gear',   1),
            rpm,
            corner_target,
            overspeed_kmh,
            brake_active,
        ]
        _safe_writerow(self._track_w, self._track_fh, row)

    # ------------------------------------------------------------------
    # Finish / cleanup
    # ------------------------------------------------------------------

    def on_finish(self, status='unknown'):
        """
        Call on any exit path (step limit, Ctrl+C, server shutdown, error).
        If no laps were completed in this run, writes one incomplete row.
        Already-completed laps are never modified.
        Flushes and closes both files.
        """
        if self._closed:
            return
        self._closed = True

        if self._lap_number == 0 or status != 'completed':
            # Record the incomplete attempt (blank lap_time_seconds per spec)
            max_spd = round(self._lap_max_speed, 2) if self._lap_max_speed > 0 else ''
            min_spd = round(self._lap_min_speed, 2) if self._lap_min_speed < 999.0 else ''
            row = [
                self.run_id, self.ts_utc, self.version,
                self.track_label, self.car_label,
                self._lap_number,  # laps completed before exit
                '',                # blank lap_time_seconds
                '',                # damage_total unknown at abort
                '',                # damage_delta unknown at abort
                status,
                self.change_desc,
                max_spd,
                min_spd,
            ]
            _safe_writerow(self._lap_w, self._lap_fh, row)
            print('[race_logging] Incomplete attempt recorded. status=%s' % status)

        for fh in (self._lap_fh, self._track_fh):
            try:
                fh.flush()
                fh.close()
            except OSError:
                pass  # best-effort on close

        print('[race_logging] Files closed. run_id=%s  laps_completed=%d'
              % (self.run_id, self._lap_number))

    def on_restart(self):
        """
        Call when the server signals ***restart***.
        Resets all lap-detection state so the next crossing is not counted
        as continuing from before the restart.
        """
        print('[race_logging] Server restart signal – lap state reset.')
        self._prev_cur_lap_time  = None
        self._last_crossing_dist = 0.0
        self._seen_forward       = False
        self._last_sample_lap_time = None
        # Do not reset _prev_damage or _lap_number so the CSV record is
        # contiguous; the first crossing after restart will be guarded by
        # the MIN_LAP_DIST check.
