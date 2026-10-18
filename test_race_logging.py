#!/usr/bin/env python3
"""
test_race_logging.py
--------------------
Synthetic unit tests for race_logging.RaceLogger.

These tests use fabricated sensor dictionaries — no TORCS connection is needed.
Results are written to a temporary directory; the real results/ CSV files are
NOT touched.  All output is clearly labelled SYNTHETIC.

Test cases:
  1. Normal lap completion (two consecutive laps)
  2. Equal consecutive lastLapTime values (both must be accepted)
  3. Reverse crossing (speedX never positive after a reset)
  4. Spawn / restart guard (distRaced below MIN_LAP_DIST)
  5. Ambiguous crossing (lastLapTime <= 0 after reset)
  6. Ctrl+C interruption mid-lap (incomplete attempt recorded)
  7. Header-mismatch detection on an existing CSV
  8. ~5 Hz sampling rate check (correct row count for elapsed sim time)

Run:
    python test_race_logging.py
Expected output: all tests PASS.  No TORCS required.
"""

import csv
import os
import sys
import tempfile
import shutil
import traceback

# ---------------------------------------------------------------------------
# Patch race_logging to write to a temp directory for tests
# ---------------------------------------------------------------------------
import race_logging as rl_module

_orig_results_dir = rl_module.RESULTS_DIR
_orig_lap_csv     = rl_module.LAP_CSV
_orig_track_csv   = rl_module.TRACK_CSV


def _patch_paths(tmp_dir):
    rl_module.RESULTS_DIR = tmp_dir
    rl_module.LAP_CSV     = os.path.join(tmp_dir, 'lap_data.csv')
    rl_module.TRACK_CSV   = os.path.join(tmp_dir, 'track_data.csv')


def _restore_paths():
    rl_module.RESULTS_DIR = _orig_results_dir
    rl_module.LAP_CSV     = _orig_lap_csv
    rl_module.TRACK_CSV   = _orig_track_csv


# ---------------------------------------------------------------------------
# Fake client / sensor helpers
# ---------------------------------------------------------------------------

class FakeDriverAction:
    def __init__(self):
        self.d = {'steer': 0.0, 'accel': 0.5, 'brake': 0.0, 'gear': 1}

    def clip_to_limits(self):
        pass   # no-op for tests


class FakeServerState:
    def __init__(self):
        self.d = {}


class FakeClient:
    def __init__(self):
        self.S = FakeServerState()
        self.R = FakeDriverAction()
        self.so = object()   # truthy sentinel


def _make_state(curLapTime=1.0, lastLapTime=0.0,
                distFromStart=100.0, distRaced=500.0,
                speedX=80.0, speedY=0.0,
                angle=0.0, trackPos=0.0, damage=0.0,
                track=None):
    if track is None:
        track = [100.0] * 19
    return {
        'curLapTime':    curLapTime,
        'lastLapTime':   lastLapTime,
        'distFromStart': distFromStart,
        'distRaced':     distRaced,
        'speedX':        speedX,
        'speedY':        speedY,
        'angle':         angle,
        'trackPos':      trackPos,
        'damage':        damage,
        'track':         track,
        'wheelSpinVel':  [10.0, 10.0, 10.0, 10.0],
    }


def _read_lap_rows(tmp_dir):
    path = os.path.join(tmp_dir, 'lap_data.csv')
    with open(path, 'r', newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def _read_track_rows(tmp_dir):
    path = os.path.join(tmp_dir, 'track_data.csv')
    with open(path, 'r', newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


# ---------------------------------------------------------------------------
# Test runner
# ---------------------------------------------------------------------------

PASS = 0
FAIL = 0


def check(condition, name, detail=''):
    global PASS, FAIL
    if condition:
        print('  PASS  %s' % name)
        PASS += 1
    else:
        print('  FAIL  %s%s' % (name, ('  [%s]' % detail) if detail else ''))
        FAIL += 1


# ---------------------------------------------------------------------------
# Test 1 — Normal lap completion (two laps)
# ---------------------------------------------------------------------------

def test_normal_two_laps():
    print('\nTest 1: Normal two-lap completion')
    tmp = tempfile.mkdtemp()
    _patch_paths(tmp)
    try:
        c = FakeClient()
        logger = rl_module.RaceLogger('v1', 'test_track', 'test_car', 'unit_test')

        dist_offset = 200.0   # simulate car already 200 m from start

        # Lap 1 in progress
        for t in range(1, 61):
            c.S.d = _make_state(curLapTime=float(t), lastLapTime=0.0,
                                speedX=80.0, distRaced=dist_offset + t * 20.0)
            logger.on_step(c)

        # curLapTime resets -> lap 1 complete (lastLapTime=60.0)
        c.S.d = _make_state(curLapTime=0.5, lastLapTime=60.0,
                            speedX=80.0, distRaced=dist_offset + 61 * 20.0)
        logger.on_step(c)

        # Lap 2 in progress
        for t in range(1, 56):
            c.S.d = _make_state(curLapTime=float(t), lastLapTime=60.0,
                                speedX=80.0, distRaced=dist_offset + (61 + t) * 20.0)
            logger.on_step(c)

        # curLapTime resets -> lap 2 complete (lastLapTime=55.0)
        c.S.d = _make_state(curLapTime=0.3, lastLapTime=55.0,
                            speedX=80.0, distRaced=dist_offset + (61 + 56) * 20.0)
        logger.on_step(c)

        logger.on_finish(status='step_limit')

        rows = _read_lap_rows(tmp)
        completed = [r for r in rows if r['status'] == 'completed']
        check(len(completed) == 2,
              'Two completed laps recorded', 'count=%d' % len(completed))
        check(completed[0]['lap_time_seconds'] == '60.0',
              'Lap 1 time = 60.0', completed[0]['lap_time_seconds'])
        check(completed[1]['lap_time_seconds'] == '55.0',
              'Lap 2 time = 55.0', completed[1]['lap_time_seconds'])
        check(completed[0]['lap_number'] == '1',
              'Lap 1 number = 1', completed[0]['lap_number'])
        check(completed[1]['lap_number'] == '2',
              'Lap 2 number = 2', completed[1]['lap_number'])
    finally:
        shutil.rmtree(tmp)
        _restore_paths()


# ---------------------------------------------------------------------------
# Test 2 — Equal consecutive lastLapTime values accepted
# ---------------------------------------------------------------------------

def test_equal_lap_times():
    print('\nTest 2: Equal consecutive lastLapTime values must both be accepted')
    tmp = tempfile.mkdtemp()
    _patch_paths(tmp)
    try:
        c = FakeClient()
        logger = rl_module.RaceLogger('v1', 'test_track', 'test_car', 'unit_test')

        dist = 300.0
        for t in range(1, 61):
            c.S.d = _make_state(curLapTime=float(t), lastLapTime=0.0,
                                speedX=80.0, distRaced=dist + t * 20.0)
            logger.on_step(c)

        # Lap 1 completes: lastLapTime = 60.0
        c.S.d = _make_state(curLapTime=0.5, lastLapTime=60.0,
                            speedX=80.0, distRaced=dist + 61 * 20.0)
        logger.on_step(c)

        for t in range(1, 61):
            c.S.d = _make_state(curLapTime=float(t), lastLapTime=60.0,
                                speedX=80.0, distRaced=dist + (61 + t) * 20.0)
            logger.on_step(c)

        # Lap 2 completes: same lastLapTime = 60.0 (equal consecutive — must be valid)
        c.S.d = _make_state(curLapTime=0.5, lastLapTime=60.0,
                            speedX=80.0, distRaced=dist + (61 + 61) * 20.0)
        logger.on_step(c)

        logger.on_finish(status='step_limit')

        rows = _read_lap_rows(tmp)
        completed = [r for r in rows if r['status'] == 'completed']
        check(len(completed) == 2,
              'Equal lap times: both laps recorded as completed',
              'count=%d' % len(completed))
        check(completed[0]['lap_time_seconds'] == completed[1]['lap_time_seconds'],
              'Both lap times are equal (60.0)',
              '%s vs %s' % (completed[0]['lap_time_seconds'],
                            completed[1]['lap_time_seconds']))
    finally:
        shutil.rmtree(tmp)
        _restore_paths()


# ---------------------------------------------------------------------------
# Test 3 — Reverse crossing ignored
# Design: complete one valid forward lap (to advance _last_crossing_dist and
# reset _seen_forward), then drive backward only, then trigger a crossing.
# _seen_forward will be False at the crossing -> reverse_crossing.
# ---------------------------------------------------------------------------

def test_reverse_crossing():
    print('\nTest 3: Reverse crossing (negative speedX) must not count as a lap')
    tmp = tempfile.mkdtemp()
    _patch_paths(tmp)
    try:
        c = FakeClient()
        logger = rl_module.RaceLogger('v1', 'test_track', 'test_car', 'unit_test')

        # Phase 1: drive forward and complete a valid lap.
        # _reset_lap_state is called at the crossing, setting _seen_forward = False.
        dist = 500.0
        for t in range(1, 61):
            c.S.d = _make_state(curLapTime=float(t), lastLapTime=0.0,
                                speedX=80.0, distRaced=dist + t * 20.0)
            logger.on_step(c)

        # Valid forward crossing -> _seen_forward resets to False
        c.S.d = _make_state(curLapTime=0.5, lastLapTime=60.0,
                            speedX=80.0, distRaced=dist + 61 * 20.0)
        logger.on_step(c)

        # Phase 2: only backward motion — _seen_forward stays False throughout
        base_dist = dist + 61 * 20.0
        for t in range(1, 61):
            c.S.d = _make_state(curLapTime=float(t), lastLapTime=60.0,
                                speedX=-20.0, distRaced=base_dist + t * 5.0)
            logger.on_step(c)

        # curLapTime resets; distRaced advanced >MIN_LAP_DIST but _seen_forward is False
        # -> must be classified as reverse_crossing, not completed
        c.S.d = _make_state(curLapTime=0.3, lastLapTime=60.0,
                            speedX=-20.0, distRaced=base_dist + 61 * 5.0)
        logger.on_step(c)

        logger.on_finish(status='step_limit')

        rows = _read_lap_rows(tmp)
        rev_rows  = [r for r in rows if r['status'] == 'reverse_crossing']
        completed = [r for r in rows if r['status'] == 'completed']
        check(len(rev_rows) >= 1,
              'Reverse crossing recorded as reverse_crossing',
              'found=%d' % len(rev_rows))
        # The initial forward lap is completed; the reverse crossing must not add
        # another completed row.
        check(len(completed) == 1,
              'Only the initial forward lap is completed (not the reverse)',
              'completed=%d' % len(completed))
    finally:
        shutil.rmtree(tmp)
        _restore_paths()


# ---------------------------------------------------------------------------
# Test 4 — Spawn / restart guard (distRaced too small)
# ---------------------------------------------------------------------------

def test_spawn_guard():
    print('\nTest 4: Spawn guard - distRaced below MIN_LAP_DIST not counted')
    tmp = tempfile.mkdtemp()
    _patch_paths(tmp)
    try:
        c = FakeClient()
        logger = rl_module.RaceLogger('v1', 'test_track', 'test_car', 'unit_test')

        # Spawn scenario: distRaced barely moves (< 100 m)
        for t in range(1, 11):
            c.S.d = _make_state(curLapTime=float(t), lastLapTime=0.0,
                                speedX=10.0, distRaced=float(t) * 5.0)
            logger.on_step(c)

        # curLapTime resets but distRaced only = 55 m  (< MIN_LAP_DIST=100)
        c.S.d = _make_state(curLapTime=0.1, lastLapTime=10.0,
                            speedX=10.0, distRaced=55.0)
        logger.on_step(c)

        logger.on_finish(status='step_limit')

        rows = _read_lap_rows(tmp)
        completed = [r for r in rows if r['status'] == 'completed']
        check(len(completed) == 0,
              'Spawn crossing ignored (distRaced < MIN_LAP_DIST)',
              'completed=%d' % len(completed))
    finally:
        shutil.rmtree(tmp)
        _restore_paths()


# ---------------------------------------------------------------------------
# Test 5 — Ambiguous crossing (lastLapTime <= 0)
# ---------------------------------------------------------------------------

def test_ambiguous_crossing():
    print('\nTest 5: Ambiguous crossing (lastLapTime <= 0)')
    tmp = tempfile.mkdtemp()
    _patch_paths(tmp)
    try:
        c = FakeClient()
        logger = rl_module.RaceLogger('v1', 'test_track', 'test_car', 'unit_test')

        dist = 400.0
        for t in range(1, 61):
            c.S.d = _make_state(curLapTime=float(t), lastLapTime=0.0,
                                speedX=80.0, distRaced=dist + t * 20.0)
            logger.on_step(c)

        # curLapTime resets but lastLapTime is still 0 (server did not update it)
        c.S.d = _make_state(curLapTime=0.2, lastLapTime=0.0,
                            speedX=80.0, distRaced=dist + 61 * 20.0)
        logger.on_step(c)

        logger.on_finish(status='step_limit')

        rows = _read_lap_rows(tmp)
        ambiguous = [r for r in rows if r['status'] == 'ambiguous']
        completed  = [r for r in rows if r['status'] == 'completed']
        check(len(ambiguous) == 1,
              'Ambiguous crossing recorded as ambiguous',
              'found=%d' % len(ambiguous))
        check(len(completed) == 0,
              'No completed lap from ambiguous crossing',
              'completed=%d' % len(completed))
    finally:
        shutil.rmtree(tmp)
        _restore_paths()


# ---------------------------------------------------------------------------
# Test 6 — Ctrl+C interruption mid-lap (incomplete row, completed preserved)
# ---------------------------------------------------------------------------

def test_interruption():
    print('\nTest 6: Ctrl+C interruption mid-lap - incomplete row recorded')
    tmp = tempfile.mkdtemp()
    _patch_paths(tmp)
    try:
        c = FakeClient()
        logger = rl_module.RaceLogger('v1', 'test_track', 'test_car', 'unit_test')

        dist = 300.0
        # Lap 1 completes
        for t in range(1, 61):
            c.S.d = _make_state(curLapTime=float(t), lastLapTime=0.0,
                                speedX=80.0, distRaced=dist + t * 20.0)
            logger.on_step(c)
        c.S.d = _make_state(curLapTime=0.5, lastLapTime=60.0,
                            speedX=80.0, distRaced=dist + 61 * 20.0)
        logger.on_step(c)

        # Lap 2 partially driven, then interrupted
        for t in range(1, 31):
            c.S.d = _make_state(curLapTime=float(t), lastLapTime=60.0,
                                speedX=80.0, distRaced=dist + (61 + t) * 20.0)
            logger.on_step(c)

        logger.on_finish(status='interrupted')

        rows = _read_lap_rows(tmp)
        completed   = [r for r in rows if r['status'] == 'completed']
        interrupted = [r for r in rows if r['status'] == 'interrupted']
        check(len(completed) == 1,
              'Completed lap preserved after interruption',
              'completed=%d' % len(completed))
        check(len(interrupted) == 1,
              'Interrupted attempt recorded',
              'interrupted=%d' % len(interrupted))
        check(interrupted[0]['lap_time_seconds'] == '',
              'Interrupted row has blank lap_time_seconds',
              repr(interrupted[0]['lap_time_seconds']))
        check(completed[0]['lap_time_seconds'] == '60.0',
              'Completed lap time intact',
              completed[0]['lap_time_seconds'])
    finally:
        shutil.rmtree(tmp)
        _restore_paths()


# ---------------------------------------------------------------------------
# Test 7 — Header mismatch on existing CSV raises SystemExit
# ---------------------------------------------------------------------------

def test_header_mismatch():
    print('\nTest 7: Header mismatch on existing CSV must raise SystemExit')
    tmp = tempfile.mkdtemp()
    _patch_paths(tmp)
    try:
        # Write a lap_data.csv with a wrong header and a data row
        bad_path = os.path.join(tmp, 'lap_data.csv')
        with open(bad_path, 'w', newline='', encoding='utf-8') as f:
            csv.writer(f).writerow(['wrong_col1', 'wrong_col2'])
            csv.writer(f).writerow(['data', 'data'])

        raised = False
        try:
            rl_module.RaceLogger('v1', 'test_track', 'test_car', 'unit_test')
        except SystemExit:
            raised = True

        check(raised, 'SystemExit raised on header mismatch')
    finally:
        shutil.rmtree(tmp)
        _restore_paths()


# ---------------------------------------------------------------------------
# Test 8 — ~5 Hz sampling rate (SAMPLE_INTERVAL_S = 0.20)
# Packets arrive at 50 Hz (every 0.02 s of sim time).
# Over 10 s: first sample at t=0.0, then one per 0.20 s.
# floor(10.0 / 0.20) = 50 additional samples -> total = 51.
# Allow ±2 for floating-point boundary effects -> range [49, 53].
# ---------------------------------------------------------------------------

def test_sampling_rate():
    print('\nTest 8: ~5 Hz sampling (1 row per 0.2 s of sim time)')
    tmp = tempfile.mkdtemp()
    _patch_paths(tmp)
    try:
        c = FakeClient()
        logger = rl_module.RaceLogger('v1', 'test_track', 'test_car', 'unit_test')

        dist = 200.0
        t = 0.0
        n_steps = 500   # 500 * 0.02 = 10.0 s of sim time
        for _ in range(n_steps):
            c.S.d = _make_state(curLapTime=t, lastLapTime=0.0,
                                speedX=80.0, distRaced=dist + t * 20.0)
            logger.on_step(c)
            t = round(t + 0.02, 6)   # round to avoid float drift

        logger.on_finish(status='step_limit')

        rows = _read_track_rows(tmp)
        # t goes 0.00, 0.02, 0.04 ... 9.98; last value = 9.98
        # Ideal: 1 + floor(9.98 / 0.20) = 1 + 49 = 50 samples.
        # Allow ±3 for floating-point boundary variation: window [47, 53].
        expected_min = 47
        expected_max = 53
        check(expected_min <= len(rows) <= expected_max,
              '~5 Hz sampling: row count in [%d, %d]' % (expected_min, expected_max),
              'actual=%d' % len(rows))
    finally:
        shutil.rmtree(tmp)
        _restore_paths()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    print('=' * 60)
    print('SYNTHETIC TESTS for race_logging.py')
    print('NOTE: All data below is fabricated.  Not race results.')
    print('=' * 60)

    tests = [
        test_normal_two_laps,
        test_equal_lap_times,
        test_reverse_crossing,
        test_spawn_guard,
        test_ambiguous_crossing,
        test_interruption,
        test_header_mismatch,
        test_sampling_rate,
    ]

    for fn in tests:
        try:
            fn()
        except Exception:
            print('  EXCEPTION in %s:' % fn.__name__)
            traceback.print_exc()
            FAIL += 1
        finally:
            _restore_paths()

    print('\n' + '=' * 60)
    print('Results: %d passed, %d failed' % (PASS, FAIL))
    print('=' * 60)
    sys.exit(0 if FAIL == 0 else 1)
