#!/usr/bin/python
# snakeoil3_v2_track_memory.py
#
# OPTIONAL ENHANCEMENT — separate from the v1/v2 baseline chain.
# Do NOT use this file as the new baseline.  The baseline remains snakeoil3_v1.py.
# snakeoil3_v2.py is the active candidate; this file adds a track-memory layer
# on top of v2 and is tested independently.
#
# PURPOSE
#   At startup, load results/track_profile.csv (built by build_track_profile.py
#   from the one verified complete zero-damage lap: ada5927d, 162.504 s, v1).
#   Use the stored conservative speed targets as a feed-forward hint: look ahead
#   by a speed-dependent distance along the track and lower corner_target if the
#   profile indicates a slower section is approaching.
#
# FALLBACK POLICY
#   The driver falls back silently to the unchanged V2 reactive policy whenever:
#     - results/track_profile.csv is missing, unreadable, or has invalid metadata
#     - track_length_m is zero, negative, or absent in the profile header
#     - Any required metadata key is absent from the preamble
#     - The current distFromStart is outside [0, track_length_m * 1.05]
#         (position lost or sensor anomaly)
#     - Any bin in the look-ahead window has coverage_ok=0
#         (the driver never tightens speed on unmeasured terrain)
#   When in fallback, every step is handled identically to snakeoil3_v2.py.
#   A one-time warning is printed at startup if the profile fails to load.
#
# LOOK-AHEAD DISTANCE
#   lookahead_m = max(PROFILE_LOOKAHEAD_MIN_M, speedX_kmh * PROFILE_LOOKAHEAD_S)
#   At 90 km/h (25 m/s) this is max(30, 25) = 30 m (one 50-m bin).
#   At 120 km/h this is max(30, 33) = 33 m (still one bin).
#   The look-ahead is deliberately short and conservative; we do not speculate
#   about sections many bins ahead.
#
# SPEED LIMIT COMBINATION
#   profile_limit = minimum speed_conservative_kmh in look-ahead window
#   corner_target  = min(live_corner_target, profile_limit)
#   Live sensors always take the minimum — memory can only reduce speed, never
#   override a live sensor that says the car is already in danger.
#
# KNOWN LIMITATIONS (verified against data, not claims of success):
#   1. Source lap (ada5927d) uses track='unknown', car='unknown'.
#      The profile CANNOT confirm it matches the active TORCS configuration.
#   2. track_9 never dropped below 99 m in the source lap.
#      The profile has ZERO speed data for tight corners.  If the race config
#      includes a corner sequence not present in ada5927d, the profile is silent
#      there and live sensors take over.
#   3. The profile is a static snapshot from a single lap; it does not update.
#   4. Conservative targets here record what a reactive car happened to drive
#      (~79-81 km/h in most bins), not an optimised racing line.
#   5. This is NOT learning.  The profile cannot improve without a new source lap.
#
# VERSION: v2tm  (track-memory variant; separate from v2 baseline)
#
# Run command:
#   python snakeoil3_v2_track_memory.py --port 3001 --track unknown --car unknown
#
# Sensor angle order for track_0..track_18:
#   -45,-19,-12,-7,-4,-2.5,-1.7,-1,-0.5,0,0.5,1,1.7,2.5,4,7,12,19,45

import csv
import os
import socket
import sys
import getopt
import time

from race_logging import RaceLogger

PI = 3.14159265359

data_size = 2**17

# Version label written into every CSV row.
V1_VERSION = 'v2tm'

MAX_CONNECT_ATTEMPTS = 30

# Profile file path (relative to script location)
PROFILE_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           'results', 'track_profile.csv')

# Initialize help messages
ophelp  = 'Options:\n'
ophelp += ' --host, -H <host>    TORCS server host. [localhost]\n'
ophelp += ' --port, -p <port>    TORCS port. [3001]\n'
ophelp += ' --id,   -i <id>      ID for server. [SCR]\n'
ophelp += ' --steps,-m <#>       Maximum simulation steps. 1 sec ~ 50 steps. [100000]\n'
ophelp += ' --episodes,-e <#>    Maximum learning episodes. [1]\n'
ophelp += ' --track,-t <track>   Label for this track (for logging only). [unknown]\n'
ophelp += ' --car,  -C <car>     Label for car used (for logging only). [unknown]\n'
ophelp += ' --stage,-s <#>       0=warm up, 1=qualifying, 2=race, 3=unknown. [3]\n'
ophelp += ' --change,-c <desc>   Description of change vs previous run. [baseline]\n'
ophelp += ' --debug,-d           Output full telemetry.\n'
ophelp += ' --help, -h           Show this help.\n'
ophelp += ' --version,-v         Show current version.'
usage   = 'Usage: %s [options]\n' % sys.argv[0]
usage   = usage + ophelp
version = V1_VERSION


def clip(v, lo, hi):
    if v < lo:   return lo
    elif v > hi: return hi
    else:        return v


def bargraph(x, mn, mx, w, c='X'):
    if not w: return ''
    if x < mn: x = mn
    if x > mx: x = mx
    tx = mx - mn
    if tx <= 0: return 'backwards'
    upw = tx / float(w)
    if upw <= 0: return 'what?'
    negpu, pospu, negnonpu, posnonpu = 0, 0, 0, 0
    if mn < 0:
        if x < 0:
            negpu    = -x + min(0, mx)
            negnonpu = -mn + x
        else:
            negnonpu = -mn + min(0, mx)
    if mx > 0:
        if x > 0:
            pospu    = x - max(0, mn)
            posnonpu = mx - x
        else:
            posnonpu = mx - max(0, mn)
    nnc = int(negnonpu / upw) * '-'
    npc = int(negpu    / upw) * c
    ppc = int(pospu    / upw) * c
    pnc = int(posnonpu / upw) * '_'
    return '[%s]' % (nnc + npc + ppc + pnc)


# =============================================================================
# TrackProfile — loaded once at startup, used read-only during the race
# =============================================================================

class TrackProfile(object):
    """
    Loads results/track_profile.csv and provides speed-limit lookups.

    Attributes:
        valid         (bool)  — False if loading or validation failed; driver
                                must fall back to reactive-only in that case.
        reason        (str)   — human-readable explanation if valid=False.
        track_length_m (float) — lap length in metres from the profile metadata.
        bin_width_m   (float) — bin size in metres (typically 50).
        bins          (list)  — list of dicts, one per distance bin, sorted by
                                bin_start_m.  Each dict has:
                                  bin_start_m, bin_end_m (float)
                                  speed_conservative_kmh (float or None)
                                  coverage_ok (bool)
    """

    # Required metadata keys that must be present in the preamble
    REQUIRED_META = ('source_run_id', 'track_length_m', 'bin_width_m')

    def __init__(self, path):
        self.valid          = False
        self.reason         = 'not loaded'
        self.track_length_m = 0.0
        self.bin_width_m    = 50.0
        self.bins           = []
        self._load(path)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _load(self, path):
        if not os.path.exists(path):
            self.reason = 'Profile file not found: %s' % path
            return

        meta  = {}
        data_lines = []
        try:
            with open(path, newline='') as f:
                for line in f:
                    line = line.rstrip('\n').rstrip('\r')
                    if line.startswith('#'):
                        # Parse "# key=value" preamble lines
                        content = line[1:].strip()
                        if '=' in content:
                            k, _, v = content.partition('=')
                            meta[k.strip()] = v.strip()
                    else:
                        data_lines.append(line)
        except (IOError, OSError) as e:
            self.reason = 'Could not read profile: %s' % e
            return

        # Validate required metadata
        for key in self.REQUIRED_META:
            if key not in meta:
                self.reason = 'Missing required metadata key: %s' % key
                return

        # Parse track length
        try:
            tl = float(meta['track_length_m'])
        except ValueError:
            self.reason = 'track_length_m is not numeric: %s' % meta['track_length_m']
            return
        if tl <= 0:
            self.reason = 'track_length_m <= 0: %.1f' % tl
            return
        self.track_length_m = tl

        # Parse bin width
        try:
            self.bin_width_m = float(meta.get('bin_width_m', '50'))
        except ValueError:
            self.bin_width_m = 50.0

        # Parse CSV data rows (skip blank lines)
        if not data_lines:
            self.reason = 'Profile CSV has no data rows.'
            return
        reader = csv.DictReader(data_lines)
        bins = []
        required_cols = ('bin_start_m', 'coverage_ok', 'speed_conservative_kmh')
        header_checked = False
        for row in reader:
            if not header_checked:
                for col in required_cols:
                    if col not in row:
                        self.reason = 'Missing column in profile data: %s' % col
                        return
                header_checked = True
            try:
                b_start = float(row['bin_start_m'])
                b_end   = float(row['bin_end_m'])
                cov_ok  = (row['coverage_ok'].strip() == '1')
                if row['speed_conservative_kmh'].strip():
                    spd = float(row['speed_conservative_kmh'])
                else:
                    spd = None
            except (ValueError, KeyError) as e:
                self.reason = 'Malformed data row in profile: %s' % e
                return
            bins.append({
                'bin_start_m':            b_start,
                'bin_end_m':              b_end,
                'speed_conservative_kmh': spd,
                'coverage_ok':            cov_ok,
            })

        if not bins:
            self.reason = 'Profile contains zero data bins.'
            return

        # Sort by bin start for reliable lookup
        bins.sort(key=lambda b: b['bin_start_m'])
        self.bins   = bins
        self.valid  = True
        self.reason = 'OK'

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def lookup(self, dist_m, lookahead_m):
        """
        Return the minimum speed_conservative_kmh in the window
        [dist_m, dist_m + lookahead_m], wrapping around track_length_m.

        Returns None (fall back to reactive) if:
          - self.valid is False
          - dist_m is outside plausible track bounds
          - Any bin in the window has coverage_ok=False
          - No covered bins exist in the window
        """
        if not self.valid:
            return None

        tl = self.track_length_m

        # Sanity-check position
        if dist_m < 0 or dist_m > tl * 1.05:
            return None  # position lost

        # Normalise start of window to [0, tl)
        d_start = dist_m % tl
        d_end   = d_start + lookahead_m

        # Collect bins that overlap the window (handle wrap-around)
        window_bins = []
        for b in self.bins:
            bs = b['bin_start_m']
            be = b['bin_end_m']
            if d_end <= tl:
                # No wrap
                if be > d_start and bs < d_end:
                    window_bins.append(b)
            else:
                # Window wraps past the finish line
                wrap_end = d_end - tl
                if be > d_start or bs < wrap_end:
                    window_bins.append(b)

        if not window_bins:
            return None  # no bins in window

        # Any gap bin → fall back (do not guess on unmeasured territory)
        for b in window_bins:
            if not b['coverage_ok']:
                return None

        # Return the most conservative (lowest) speed in the window
        speeds = [b['speed_conservative_kmh'] for b in window_bins
                  if b['speed_conservative_kmh'] is not None]
        if not speeds:
            return None

        return min(speeds)


# =============================================================================
# Load profile once at module import time
# =============================================================================

PROFILE = TrackProfile(PROFILE_CSV)
if PROFILE.valid:
    print('TrackProfile loaded: %.0f m track, %d bins, reason=%s'
          % (PROFILE.track_length_m, len(PROFILE.bins), PROFILE.reason))
else:
    print('TrackProfile WARNING: falling back to reactive-only. Reason: %s'
          % PROFILE.reason)

# Look-ahead tuning constants
PROFILE_LOOKAHEAD_S     = 1.0   # seconds of forward look at current speed
PROFILE_LOOKAHEAD_MIN_M = 30.0  # minimum look-ahead regardless of speed (metres)


# =============================================================================
# V2 driving constants (unchanged from snakeoil3_v2.py)
# =============================================================================

CORNER_FACTOR   = 50    # km/h speed reduction per unit of |steer|
V_MIN           = 40    # minimum corner_target (km/h)

BRAKE_LOOKAHEAD = 60.0  # metres — gate distance for prospective braking
BRAKE_MAX       = 0.3   # maximum brake fraction (0–1)
BRAKE_RAMP      = 0.05  # brake increment/decrement per simulation step


# =============================================================================
# Client / ServerState / DriverAction — identical to snakeoil3_v2.py
# =============================================================================

class Client():
    def __init__(self, H=None, p=None, i=None, e=None, t=None, s=None,
                 d=None, vision=False, car=None, change=None):
        self.vision = vision

        self.host           = 'localhost'
        self.port           = 3001
        self.sid            = 'SCR'
        self.maxEpisodes    = 1
        self.trackname      = 'unknown'
        self.carname        = 'unknown'
        self.change_desc    = 'v2tm: track_memory layer on top of v2'
        self.stage          = 3
        self.debug          = False
        self.maxSteps       = 100000

        self.parse_the_command_line()

        if H:      self.host        = H
        if p:      self.port        = p
        if i:      self.sid         = i
        if e:      self.maxEpisodes = e
        if t:      self.trackname   = t
        if s:      self.stage       = s
        if d:      self.debug       = d
        if car:    self.carname     = car
        if change: self.change_desc = change

        self.S = ServerState()
        self.R = DriverAction()
        self.setup_connection()

    def setup_connection(self):
        try:
            self.so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error as emsg:
            print('Error: Could not create socket: %s' % emsg)
            sys.exit(-1)

        self.so.settimeout(1)

        a = "-45 -19 -12 -7 -4 -2.5 -1.7 -1 -.5 0 .5 1 1.7 2.5 4 7 12 19 45"
        initmsg = '%s(init %s)' % (self.sid, a)

        attempts = 0
        while True:
            attempts += 1
            if attempts > MAX_CONNECT_ATTEMPTS:
                print('\nCould not connect to TORCS server at %s:%d after %d attempts.'
                      % (self.host, self.port, MAX_CONNECT_ATTEMPTS))
                print('Manual steps:')
                print('  1. Start TORCS.')
                print('  2. In TORCS: Race > Quick Race (or your race config) > New Race.')
                print('  3. Re-run: python snakeoil3_v2_track_memory.py --port %d' % self.port)
                sys.exit(-1)

            try:
                self.so.sendto(initmsg.encode(), (self.host, self.port))
            except socket.error as emsg:
                print('Socket send error: %s' % emsg)
                sys.exit(-1)

            sockdata = ''
            try:
                sockdata, addr = self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error:
                remaining = MAX_CONNECT_ATTEMPTS - attempts
                print('Waiting for TORCS on %s:%d  (attempt %d/%d, %d remaining) ...'
                      % (self.host, self.port, attempts, MAX_CONNECT_ATTEMPTS, remaining))
                continue

            if '***identified***' in sockdata:
                print('Client connected on %d.' % self.port)
                break

    def parse_the_command_line(self):
        try:
            (opts, args) = getopt.getopt(
                sys.argv[1:], 'H:p:i:m:e:t:C:s:c:dhv',
                ['host=', 'port=', 'id=', 'steps=', 'episodes=',
                 'track=', 'car=', 'stage=', 'change=',
                 'debug', 'help', 'version'])
        except getopt.error as why:
            print('getopt error: %s\n%s' % (why, usage))
            sys.exit(-1)
        try:
            for opt in opts:
                if opt[0] in ('-h', '--help'):
                    print(usage)
                    sys.exit(0)
                if opt[0] in ('-d', '--debug'):
                    self.debug = True
                if opt[0] in ('-H', '--host'):
                    self.host = opt[1]
                if opt[0] in ('-i', '--id'):
                    self.sid = opt[1]
                if opt[0] in ('-t', '--track'):
                    self.trackname = opt[1]
                if opt[0] in ('-C', '--car'):
                    self.carname = opt[1]
                if opt[0] in ('-c', '--change'):
                    self.change_desc = opt[1]
                if opt[0] in ('-s', '--stage'):
                    self.stage = int(opt[1])
                ######### REQUIRED FOR RACE MODE - DO NOT CHANGE OR REMOVE #########
                if opt[0] in ('-p', '--port'):
                    self.port = int(opt[1])
                ######## END OF REQUIRED #########
                if opt[0] in ('-e', '--episodes'):
                    self.maxEpisodes = int(opt[1])
                if opt[0] in ('-m', '--steps'):
                    self.maxSteps = int(opt[1])
                if opt[0] in ('-v', '--version'):
                    print('%s %s' % (sys.argv[0], version))
                    sys.exit(0)
        except ValueError as why:
            print("Bad parameter '%s' for option %s: %s\n%s"
                  % (opt[1], opt[0], why, usage))
            sys.exit(-1)
        if args:
            print('Superfluous input? %s\n%s' % (', '.join(args), usage))
            sys.exit(-1)

    ######### REQUIRED FOR RACE MODE - DO NOT CHANGE OR REMOVE #########
    def get_servers_input(self):
        '''Server's input is stored in a ServerState object'''
        if not self.so: return
        sockdata = ''

        while True:
            try:
                sockdata, addr = self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error as emsg:
                print('.', end=' ')
            if '***identified***' in sockdata:
                print('Client connected on %d.' % self.port)
                continue
            elif '***shutdown***' in sockdata:
                print(('Server has stopped the race on %d. '
                        'You were in %d place.')
                       % (self.port, self.S.d.get('racePos', 0)))
                self.shutdown()
                return
            elif '***restart***' in sockdata:
                print('Server has restarted the race on %d.' % self.port)
                self.shutdown()
                return
            elif not sockdata:
                continue
            else:
                self.S.parse_server_str(sockdata)
                if self.debug:
                    sys.stderr.write("\x1b[2J\x1b[H")
                    print(self.S)
                break
    ########## END OF REQUIRED #########

    def respond_to_server(self):
        if not self.so: return
        try:
            message = repr(self.R)
            self.so.sendto(message.encode(), (self.host, self.port))
        except socket.error as emsg:
            print('Error sending to server: %s' % str(emsg))
            sys.exit(-1)
        if self.debug: print(self.R.fancyout())

    def shutdown(self):
        if not self.so: return
        print(('Race terminated or %d steps elapsed. Shutting down %d.'
               % (self.maxSteps, self.port)))
        self.so.close()
        self.so = None


class ServerState():
    '''What the server is reporting right now.'''
    def __init__(self):
        self.servstr = ''
        self.d = {}

    def parse_server_str(self, server_string):
        self.servstr = server_string.strip()[:-1]
        sslisted = self.servstr.strip().lstrip('(').rstrip(')').split(')(')
        for i in sslisted:
            w = i.split(' ')
            self.d[w[0]] = destringify(w[1:])

    def __repr__(self):
        return self.fancyout()

    def fancyout(self):
        out = ''
        sensors = [
            'stucktimer', 'fuel', 'distRaced', 'distFromStart',
            'opponents', 'wheelSpinVel', 'z', 'speedZ', 'speedY', 'speedX',
            'targetSpeed', 'rpm', 'skid', 'slip', 'track', 'trackPos', 'angle',
        ]
        for k in sensors:
            if type(self.d.get(k)) is list:
                if k == 'track':
                    strout = ''
                    raw_tsens = ['%.1f' % x for x in self.d['track']]
                    strout += (' '.join(raw_tsens[:9]) + '_' + raw_tsens[9]
                               + '_' + ' '.join(raw_tsens[10:]))
                elif k == 'opponents':
                    strout = ''
                    for osensor in self.d['opponents']:
                        if   osensor > 190: oc = '_'
                        elif osensor >  90: oc = '.'
                        elif osensor >  39: oc = chr(int(osensor / 2) + 97 - 19)
                        elif osensor >  13: oc = chr(int(osensor) + 65 - 13)
                        elif osensor >   3: oc = chr(int(osensor) + 48 - 3)
                        else:               oc = '?'
                        strout += oc
                    strout = ' -> ' + strout[:18] + ' ' + strout[18:] + ' <-'
                else:
                    strout = ', '.join([str(i) for i in self.d[k]])
            else:
                if k == 'gear':
                    gs = '_._._._._._._._._'
                    p  = int(self.d['gear']) * 2 + 2
                    l  = '%d' % self.d['gear']
                    if l == '-1': l = 'R'
                    if l == '0':  l = 'N'
                    strout = gs[:p] + '(%s)' % l + gs[p+3:]
                elif k == 'damage':
                    strout = '%6.0f %s' % (self.d[k], bargraph(self.d[k], 0, 10000, 50, '~'))
                elif k == 'fuel':
                    strout = '%6.0f %s' % (self.d[k], bargraph(self.d[k], 0, 100, 50, 'f'))
                elif k == 'speedX':
                    cx = 'R' if self.d[k] < 0 else 'X'
                    strout = '%6.1f %s' % (self.d[k], bargraph(self.d[k], -30, 300, 50, cx))
                elif k == 'speedY':
                    strout = '%6.1f %s' % (self.d[k], bargraph(self.d[k] * -1, -25, 25, 50, 'Y'))
                elif k == 'speedZ':
                    strout = '%6.1f %s' % (self.d[k], bargraph(self.d[k], -13, 13, 50, 'Z'))
                elif k == 'z':
                    strout = '%6.3f %s' % (self.d[k], bargraph(self.d[k], .3, .5, 50, 'z'))
                elif k == 'trackPos':
                    cx = '>' if self.d[k] < 0 else '<'
                    strout = '%6.3f %s' % (self.d[k], bargraph(self.d[k] * -1, -1, 1, 50, cx))
                elif k == 'stucktimer':
                    if self.d[k]:
                        strout = '%3d %s' % (self.d[k], bargraph(self.d[k], 0, 300, 50, "'"))
                    else:
                        strout = 'Not stuck!'
                elif k == 'rpm':
                    g = self.d['gear']
                    g = 'R' if g < 0 else '%1d' % g
                    strout = bargraph(self.d[k], 0, 10000, 50, g)
                elif k == 'angle':
                    asyms = [
                        "  !  ", ".|'  ", "./'  ", "_.-  ", ".--  ", "..-  ",
                        "---  ", ".__  ", "-._  ", "'-.  ", "'\.  ", "'|.  ",
                        "  |  ", "  .|'", "  ./'", "  .-'", "  _.-", "  __.",
                        "  ---", "  --.", "  -._", "  -..", "  '\.", "  '|.",
                    ]
                    rad    = self.d[k]
                    deg    = int(rad * 180 / PI)
                    symno  = int(.5 + (rad + PI) / (PI / 12))
                    symno  = symno % (len(asyms) - 1)
                    strout = '%5.2f %3d (%s)' % (rad, deg, asyms[symno])
                elif k == 'skid':
                    fwrps = self.d['wheelSpinVel'][0]
                    skid  = 0
                    if fwrps:
                        skid = .5555555555 * self.d['speedX'] / fwrps - .66124
                    strout = bargraph(skid, -.05, .4, 50, '*')
                elif k == 'slip':
                    fwrps = self.d['wheelSpinVel'][0]
                    slip  = 0
                    if fwrps:
                        slip = ((self.d['wheelSpinVel'][2] + self.d['wheelSpinVel'][3])
                                - (self.d['wheelSpinVel'][0] + self.d['wheelSpinVel'][1]))
                    strout = bargraph(slip, -5, 150, 50, '@')
                else:
                    strout = str(self.d.get(k, ''))
            out += '%s: %s\n' % (k, strout)
        return out


class DriverAction():
    def __init__(self):
        self.actionstr = ''
        self.d = {
            'accel':  0.2,
            'brake':  0,
            'clutch': 0,
            'gear':   1,
            'steer':  0,
            'focus':  [-90, -45, 0, 45, 90],
            'meta':   0,
        }

    def clip_to_limits(self):
        self.d['steer']  = clip(self.d['steer'],  -1, 1)
        self.d['brake']  = clip(self.d['brake'],   0, 1)
        self.d['accel']  = clip(self.d['accel'],   0, 1)
        self.d['clutch'] = clip(self.d['clutch'],  0, 1)
        if self.d['gear'] not in [-1, 0, 1, 2, 3, 4, 5, 6]:
            self.d['gear'] = 0
        if self.d['meta'] not in [0, 1]:
            self.d['meta'] = 0
        if (type(self.d['focus']) is not list
                or min(self.d['focus']) < -180
                or max(self.d['focus']) > 180):
            self.d['focus'] = 0

    def __repr__(self):
        self.clip_to_limits()
        out = ''
        for k in self.d:
            out += '(' + k + ' '
            v = self.d[k]
            if type(v) is not list:
                out += '%.3f' % v
            else:
                out += ' '.join([str(x) for x in v])
            out += ')'
        return out

    def fancyout(self):
        out = ''
        od  = self.d.copy()
        od.pop('gear',  '')
        od.pop('meta',  '')
        od.pop('focus', '')
        for k in sorted(od):
            if k in ('clutch', 'brake', 'accel'):
                strout = '%6.3f %s' % (od[k], bargraph(od[k], 0, 1, 50, k[0].upper()))
            elif k == 'steer':
                strout = '%6.3f %s' % (od[k], bargraph(od[k] * -1, -1, 1, 50, 'S'))
            else:
                strout = str(od[k])
            out += '%s: %s\n' % (k, strout)
        return out


# =============================================================================
# Misc Utility
# =============================================================================

def destringify(s):
    if not s: return s
    if type(s) is str:
        try:
            return float(s)
        except ValueError:
            print('Could not find a value in %s' % s)
            return s
    elif type(s) is list:
        if len(s) < 2:
            return destringify(s[0])
        else:
            return [destringify(i) for i in s]


# =============================================================================
# drive_example — V2 reactive policy + optional profile look-ahead
# =============================================================================
#
# Change vs snakeoil3_v2.py: ONE additional block between corner_target
# computation and the braking block.  All other lines are byte-identical.
#
# Profile look-ahead block (NEW):
#   Compute lookahead_m from current speed.
#   Call PROFILE.lookup(distFromStart, lookahead_m).
#   If a valid profile limit is returned, take the minimum with corner_target.
#   If lookup returns None (fallback condition), leave corner_target unchanged.
#   In both cases the braking block and all safety controls below are unaffected.

def drive_example(c):
    S, R = c.S.d, c.R.d
    target_speed = 90   # unchanged from V2

    # Steer To Corner
    R['steer'] = S['angle'] * 15 / PI
    # Steer To Center
    R['steer'] -= S['trackPos'] * .10

    # Throttle Control — V2: magnitude steer (symmetric)
    corner_target = max(V_MIN, target_speed - abs(R['steer']) * CORNER_FACTOR)

    # --- Profile Look-Ahead (NEW vs V2) --------------------------------------
    # Compute a speed-dependent look-ahead distance, then query the profile
    # for the minimum conservative speed in that forward window.
    # Only lower corner_target; never raise it.
    # Fall back to reactive (no-op) if PROFILE.valid is False or lookup returns
    # None (gap bin, position lost, or profile not loaded).
    _dist    = S.get('distFromStart', None)
    _speed   = S['speedX']
    if _dist is not None and _speed >= 0:
        _lookahead = max(PROFILE_LOOKAHEAD_MIN_M,
                         _speed * PROFILE_LOOKAHEAD_S / 3.6)  # km/h → m/s
        _prof_limit = PROFILE.lookup(_dist, _lookahead)
        if _prof_limit is not None:
            corner_target = min(corner_target, _prof_limit)
    # --- End Profile Look-Ahead ----------------------------------------------

    if S['speedX'] < corner_target:
        R['accel'] += .01
    else:
        R['accel'] -= .01
    if S['speedX'] < 10:
        R['accel'] += 1 / (S['speedX'] + .1)

    # Prospective Braking — unchanged from V2
    _track = S.get('track', [])
    if isinstance(_track, list) and len(_track) == 19:
        _ahead = _track[9]
        if _ahead < 0:
            _ahead = 200.0
    else:
        _ahead = 200.0

    _overspeed = S['speedX'] - corner_target
    if _ahead < BRAKE_LOOKAHEAD and _overspeed > 0:
        R['brake'] = min(BRAKE_MAX, R['brake'] + BRAKE_RAMP)
        R['accel'] = 0.0
    else:
        R['brake'] = max(0.0, R['brake'] - BRAKE_RAMP)

    # Traction Control System — unchanged
    if ((S['wheelSpinVel'][2] + S['wheelSpinVel'][3]) -
            (S['wheelSpinVel'][0] + S['wheelSpinVel'][1]) > 5):
        R['accel'] -= .2

    # Automatic Transmission — unchanged
    R['gear'] = 1
    if S['speedX'] > 50:  R['gear'] = 2
    if S['speedX'] > 80:  R['gear'] = 3
    if S['speedX'] > 110: R['gear'] = 4
    if S['speedX'] > 140: R['gear'] = 5
    if S['speedX'] > 170: R['gear'] = 6
    return


# =============================================================================
# MAIN
# =============================================================================

if __name__ == '__main__':
    #### DO NOT HARDCODE THE PORT
    C = Client()

    logger = RaceLogger(
        version=V1_VERSION,
        track_label=C.trackname,
        car_label=C.carname,
        change_description=C.change_desc,
    )

    exit_status = 'step_limit'

    try:
        for step in range(C.maxSteps, 0, -1):
            C.get_servers_input()

            if not C.so:
                exit_status = 'server_shutdown'
                break

            drive_example(C)

            C.R.clip_to_limits()
            logger.on_step(C)
            C.respond_to_server()

    except KeyboardInterrupt:
        print('\nInterrupted by user (Ctrl+C).')
        exit_status = 'interrupted'
    except Exception as exc:
        print('Unexpected error: %s' % exc)
        exit_status = 'error'
        raise
    finally:
        logger.on_finish(status=exit_status)
        if C.so:
            C.shutdown()
