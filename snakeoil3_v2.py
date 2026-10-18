#!/usr/bin/python
# snakeoil3_v2.py
# Derived from snakeoil3_v1.py (protected measured baseline).
# V2 changes: (1) corner-aware target speed using |steer| magnitude [prior],
#             (2) prospective braking from track_9 sensor [this edit].
# V1 is unchanged.  snakeoil3_gym.py (original) is unchanged.
#
# Chris X Edwards <snakeoil@xed.ch>  (original snakeoil base)
# Snake Oil is a Python library for interfacing with a TORCS
# race car simulator which has been patched with the server
# extentions used in the Simulated Car Racing competitions.
# http://scr.geccocompetitions.com/
#
# Run command:
#   python snakeoil3_v1.py [options]
#   python snakeoil3_v1.py --help
#   python snakeoil3_v1.py --port 3001 --track road1 --car unknown --debug
#
# TORCS must already be running and configured before this script starts.
# This script will NOT launch or kill TORCS automatically (Windows-safe).
# If the server is not reachable within MAX_CONNECT_ATTEMPTS attempts
# (each with a 1-second timeout), the script exits with instructions.
#
# Metadata notes:
#   --track   is a user-chosen LABEL for logging; it does not select the
#             track inside TORCS (do that in the TORCS UI / XML config).
#   --car     is a user-chosen car label; pass via --car or it defaults to
#             'unknown'.  TORCS car is set in the race config, not here.
#   version   is set to the V1_VERSION constant below.
#
# Sensor angle order used for track_0..track_18:
#   -45,-19,-12,-7,-4,-2.5,-1.7,-1,-0.5,0,0.5,1,1.7,2.5,4,7,12,19,45
#
# Changes vs snakeoil3_v1.py (baseline):
#   drive_example ONLY — two sequential additions, each documented separately:
#
#   [1] Throttle condition: signed steer → magnitude steer (prior accepted change)
#     V1: target_speed - (R['steer'] * 50)          [signed: left faster than right]
#     V2: max(V_MIN, target_speed - abs(steer)*50)   [magnitude: symmetric]
#     Evidence: V1 runs e183d441/845391a7 showed left-corner overspeed to 103–112 km/h.
#     V2 run 73f02f12: no large-steer events; V1 run ada5927d completed one lap 162.5 s.
#
#   [2] Braking block: prospective brake from track_9 sensor (this edit)
#     Activates brake when speedX > corner_target AND track_9 < BRAKE_LOOKAHEAD.
#     Releases accel while braking.  Ramps gradually; never exceeds BRAKE_MAX.
#     Treats track_9 < 0 (off-track sentinel) and missing sensor as "no signal".
#     Steering gains, gear thresholds, connection, logging: UNCHANGED.
#
#   All thresholds are tunable hypotheses; see constants below.

import socket
import sys
import getopt
import os
import time

from race_logging import RaceLogger

PI = 3.14159265359

data_size = 2**17

# Version label written into every CSV row.
V1_VERSION = 'v11'

# Maximum UDP receive attempts before giving up and asking for manual action.
# Each attempt has a 1-second socket timeout, so this is also the max wait
# in seconds before the script exits cleanly (no infinite loop on Windows).
MAX_CONNECT_ATTEMPTS = 30

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
    '''Draws a simple asciiart bar graph. Very handy for
    visualizing what's going on with the data.
    x= Value from sensor, mn= minimum plottable value,
    mx= maximum plottable value, w= width of plot in chars,
    c= the character to plot with.'''
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
        self.change_desc    = 'v11: lookahead-steer+speed-centre+phys-gate-fix'
        self.stage          = 3
        self.debug          = False
        self.maxSteps       = 100000   # 50 steps/second

        self.parse_the_command_line()

        # Constructor keyword args override command-line (for programmatic use)
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

    # ------------------------------------------------------------------
    # Windows-compatible connection setup
    # No pkill, no 'sh autostart.sh', no trailing '&'.
    # The user must start TORCS manually before running this script.
    # ------------------------------------------------------------------
    def setup_connection(self):
        try:
            self.so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error as emsg:
            print('Error: Could not create socket: %s' % emsg)
            sys.exit(-1)

        self.so.settimeout(1)

        # Sensor angles sent in the init handshake.
        # These determine which physical angles track_0..track_18 correspond to.
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
                print('  3. Re-run this script: python snakeoil3_v1.py --port %d' % self.port)
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
        '''Parse the server string.'''
        self.servstr = server_string.strip()[:-1]
        sslisted = self.servstr.strip().lstrip('(').rstrip(')').split(')(')
        for i in sslisted:
            w = i.split(' ')
            self.d[w[0]] = destringify(w[1:])

    def __repr__(self):
        return self.fancyout()

    def fancyout(self):
        '''Specialty output for useful ServerState monitoring.'''
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
    '''What the driver is intending to do (i.e. send to the server).
    Composes something like this for the server:
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus 0)(meta 0) or
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus -90 -45 0 45 90)(meta 0)'''
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
        '''Specialty output for useful monitoring of bot's effectors.'''
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


# == Misc Utility Functions ==================================================

def destringify(s):
    '''makes a string into a value or a list of strings into a list of
    values (if possible)'''
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


# == drive_example ===========================================================
# V2 change vs V1 — THROTTLE CONDITION ONLY.
# All other lines are byte-for-byte identical to V1.
#
# V1 condition (signed steer):
#   if S['speedX'] < target_speed - (R['steer'] * 50):
#   Problem: steer<0 (left) raises the threshold; steer>0 (right) lowers it.
#   At steer=-0.27 the car was permitted to run at 103.5 km/h in a left corner.
#
# V2 condition (magnitude steer):
#   corner_target = max(V_MIN, target_speed - abs(R['steer']) * CORNER_FACTOR)
#   if S['speedX'] < corner_target:
#   Both directions are now treated identically.
#   At |steer|=0.27 the corner target is 90 - 13.5 = 76.5 km/h regardless of sign.
#
# Constants (V2 only):
#   CORNER_FACTOR = 50   — same coefficient as V1's 50, now applied symmetrically
#   V_MIN         = 40   — floor: never command below 40 km/h even at full lock
#
# NOTE on 1/(speedX+.1): if speedX == -0.1 exactly the denominator is zero.
#   Has not occurred in any recorded run.  Documented repair (apply separately):
#     Replace:  R['accel'] += 1 / (S['speedX'] + .1)
#     With:     if S['speedX'] > -0.09:
#                   R['accel'] += 1 / (S['speedX'] + .1)

# V4 — Conditional straight-line 240 km/h logic
#
# SPEED TARGET SELECTION (two conditions must BOTH be true for 240 km/h):
#
#   Condition A — Current geometry (car is on a straight RIGHT NOW):
#     |steer| < STRAIGHT_STEER_MAX  AND  |angle| < STRAIGHT_ANGLE_MAX
#     steer: commanded steering input (near-zero = straight heading)
#     angle: car body angle relative to track axis (near-zero = aligned)
#
#   Condition B — Look-ahead corridor clear (no braking zone approaching):
#     Scan the nine forward-facing track sensors within the ±2.5° cone
#     (indices 5..13).  Take their MINIMUM.  If min < STRAIGHT_LOOKAHEAD_M,
#     a wall or corner mouth is within stopping distance → NOT safe to hold 240.
#
#   Both true  → effective_target = STRAIGHT_TARGET (240 km/h)
#   Either false → effective_target = corner_target  (safe fallback)
#
# SENSOR INDEX REFERENCE for angles -45…+45 degrees:
#   idx  0   1   2   3   4     5     6    7     8    9   10   11   12   13   14  15  16  17  18
#   deg -45 -19 -12  -7  -4  -2.5 -1.7  -1  -0.5   0 +0.5  +1 +1.7 +2.5  +4  +7 +12 +19 +45
#   Forward ±2.5° cone = indices 5..13 (9 sensors).
#
# THRESHOLD RATIONALE:
#   STRAIGHT_TARGET      = 240    IBM AI Race Challenge goal.
#   STRAIGHT_STEER_MAX   = 0.05   < 0.05 = trivial steering; v2 data: median |steer|
#                                   was ~0.04 on open sections, >0.15 in corners.
#   STRAIGHT_ANGLE_MAX   = 0.05   ~2.9 deg body misalignment; trips fallback before
#                                   the car is pointing toward a barrier.
#   STRAIGHT_LOOKAHEAD_M = 150    At 240 km/h (66.7 m/s) physics braking distance
#                                   ≈ 150–200 m. Any forward sensor < 150 m means
#                                   the corner mouth is already inside stopping dist.
#
# FALLBACK (corner_target) formula — unchanged from V3:
#   corner_target = max(V_MIN, STRAIGHT_TARGET - |steer| * CORNER_FACTOR)
#   |steer|=0.0  → 240 km/h (open straight, also caught by Condition A)
#   |steer|=0.5  → max(60, 240-100) = 140 km/h  (moderate bend)
#   |steer|=0.9  → max(60, 240-180) =  60 km/h  (tight hairpin)
#
# All other constants (gear thresholds, BRAKE_*) unchanged from V3.

# V11 -- LOOKAHEAD STEERING + SPEED-SCALED CENTRE GAIN + PHYSICS GATE FIX
#
# Three targeted improvements, each grounded in v10 telemetry:
#
# 1. LOOKAHEAD STEERING (adopted from v3)
#    v10 used: steer = angle*15/PI - trackPos*0.10
#    Problem: at 200+ km/h the weighted-bearing term (aim) adds 3-10x more
#    correction than the base formula. Without it the car wanders 0.5
#    trackPos off-centre at speed, generating unnecessary steer that slows it.
#    Fix: add -aim*PI/180*LOOKAHEAD_GAIN  (v3 lookahead_gain=2.0)
#    aim = weighted average bearing of track beams (by d^2), degrees.
#    Positive aim = road bends right -> steer right (subtract from steer).
#
# 2. SPEED-SCALED CENTRE GAIN
#    v10 used: trackPos * 0.10 (constant).
#    Problem: at 250 km/h a 0.10 gain barely corrects a 0.3 trackPos error.
#    Fix: scale centre gain with speed. At 50 km/h: 0.10. At 250 km/h: 0.30.
#    centre_gain = CENTRE_GAIN_BASE + CENTRE_GAIN_SPEED * (speedX / 250)
#    Capped at CENTRE_GAIN_MAX to avoid overcorrection at 300 km/h.
#
# 3. PHYSICS GATE FIX (phys_allowed as safety net only)
#    v10 problem: phys_allowed limited speed 17% of steps outside real corners
#    because track sensors shorten 150-200 m BEFORE the corner edge curves away.
#    The _SPEED_ZONES table already knows the braking profile for every corner.
#    Fix: only apply phys_allowed when plan_target > PHYS_GATE_THRESHOLD (150 km/h)
#    meaning the plan says we are on a straight. In corners the plan drives.
#    This stops the physics gate from double-braking zones the table already handles.
#
# WHAT IS UNCHANGED:
#    - Physics braking formula (_brake_allowed)
#    - _SPEED_ZONES table, _plan_target()
#    - RPM gearbox, proportional pedal, ABS, TCS

import math as _math
from math import exp, log

# Track sensor angles -- must match init handshake (Client.setup_connection)
_TRACK_ANGLES = [-45,-19,-12,-7,-4,-2.5,-1.7,-1,-0.5,0,0.5,1,1.7,2.5,4,7,12,19,45]

STRAIGHT_SPEED     = 300   # km/h -- throttle target; physics/plan limits corners
CORNER_FACTOR      = 50    # km/h reduction per unit |steer|
V_MIN              = 40    # minimum speed floor km/h

# Steering gains
ANGLE_GAIN         = 15.0  # steer per radian of body angle (v2 proven)
LOOKAHEAD_GAIN     = 2.0   # steer per radian of weighted road bearing (v3 value)
CENTRE_GAIN_BASE   = 0.10  # centre correction at low speed (v2 proven)
CENTRE_GAIN_SPEED  = 0.20  # additional gain added linearly up to 250 km/h
CENTRE_GAIN_MAX    = 0.35  # cap to prevent overcorrection at 300 km/h

# Physics braking constants (v3 proven)
CORNER_SPEED       = 78.0
BRAKE_DECEL        = 14.0
BRAKE_AERO         = 0.0065
BRAKE_DECEL_MAX    = 30.0
BRAKE_MARGIN       = 15.0
BRAKE_GAIN         = 0.05
ABS_RATIO          = 0.8
ABS_CUT            = 0.5
PHYS_GATE_THRESHOLD = 150  # km/h -- only apply phys_allowed above this plan_target

# RPM gearbox (car1-ow1.xml)
_GEAR_RATIOS = [3.9, 2.9, 2.3, 1.87, 1.68, 1.54]
_FINAL_DRIVE = 4.5
_WHEEL_RADIUS = 0.315
UPSHIFT_RPM   = 18600
DOWNSHIFT_RPM = 15000

# Distance-keyed speed zones (unchanged from v9/v10)
_SPEED_ZONES = [
    (  400,  500,  65, 160),
    (  700,  800,  70, 120),
    ( 1000, 1100,  72, 100),
    ( 1500, 1600,  74, 110),
    ( 1900, 2000,  67, 140),
    ( 2400, 2600,  40, 190),   # HAIRPIN 1
    ( 2600, 2900,  75,   0),
    ( 2900, 3000,  73,  80),
    ( 3200, 3350,  38, 190),   # HAIRPIN 2
    ( 3350, 3400,  70,   0),
]


def _plan_target(dist):
    best = STRAIGHT_SPEED
    for (z_start, z_end, apex, buf) in _SPEED_ZONES:
        ramp_start = z_start - buf
        if ramp_start <= dist < z_start and buf > 0:
            frac = (dist - ramp_start) / float(buf)
            t = STRAIGHT_SPEED + frac * (apex - STRAIGHT_SPEED)
            if t < best: best = t
        elif z_start <= dist < z_end:
            if apex < best: best = apex
    return best


def _brake_allowed(ahead_m):
    d  = max(0.0, ahead_m - BRAKE_MARGIN)
    v0 = CORNER_SPEED / 3.6
    c  = BRAKE_AERO;  am = BRAKE_DECEL
    a0 = am + c * v0 * v0
    if a0 >= BRAKE_DECEL_MAX:
        return (v0*v0 + 2*BRAKE_DECEL_MAX*d)**0.5 * 3.6
    dc = log(BRAKE_DECEL_MAX / a0) / (2 * c)
    if d <= dc:
        return ((a0 * exp(2*c*d) - am) / c)**0.5 * 3.6
    return ((BRAKE_DECEL_MAX - am)/c + 2*BRAKE_DECEL_MAX*(d - dc))**0.5 * 3.6


def drive_example(c):
    '''Drive function -- V11: lookahead-steer + speed-centre + phys-gate-fix.'''
    S, R = c.S.d, c.R.d

    # ------------------------------------------------------------------ #
    # Steering                                                            #
    # Three components:                                                   #
    #   1. angle*ANGLE_GAIN/PI  -- align body with track axis             #
    #   2. -aim*PI/180*LOOKAHEAD_GAIN  -- steer toward open road ahead    #
    #   3. -trackPos*centre_gain  -- return to centre (speed-scaled)      #
    # ------------------------------------------------------------------ #
    angle = float(S.get('angle', 0.0))
    track_pos = float(S.get('trackPos', 0.0))
    speed = float(S.get('speedX', 0.0))

    # Component 1: body alignment
    steer = angle * ANGLE_GAIN / PI

    # Component 2: lookahead toward open road
    _track = S.get('track', [])
    _track_ok = isinstance(_track, list) and len(_track) == 19
    aim = 0.0
    if _track_ok:
        weights = [max(d, 0.0)**2 for d in _track]
        ws = sum(weights)
        if ws > 0:
            aim = sum(w * a for w, a in zip(weights, _TRACK_ANGLES)) / ws
        steer -= aim * _math.pi / 180.0 * LOOKAHEAD_GAIN

    # Component 3: centre correction, gain scales with speed
    centre_gain = min(CENTRE_GAIN_MAX,
                      CENTRE_GAIN_BASE + CENTRE_GAIN_SPEED * (speed / 250.0))
    steer -= track_pos * centre_gain

    R['steer'] = max(-1.0, min(1.0, steer))

    # ------------------------------------------------------------------ #
    # Speed target: zone plan + sensor fallback + physics gate            #
    # ------------------------------------------------------------------ #
    dist = float(S.get('distFromStart', 0.0) or 0.0)
    plan_target = _plan_target(dist)
    sensor_limit = max(V_MIN, STRAIGHT_SPEED - abs(R['steer']) * CORNER_FACTOR)
    zone_limit   = min(plan_target, sensor_limit)

    # Physics gate: only active on open straights (plan says we should be fast)
    # In corners the zone plan already encodes the braking profile.
    if _track_ok:
        _ahead = max(
            _track[8] if _track[8] >= 0 else 0.0,
            _track[9] if _track[9] >= 0 else 0.0,
            _track[10] if _track[10] >= 0 else 0.0,
        ) or 200.0
    else:
        _ahead = 200.0

    if plan_target > PHYS_GATE_THRESHOLD:
        # On a straight: physics gate guards against unexpected obstacles
        allowed_speed = min(zone_limit, _brake_allowed(_ahead))
    else:
        # In a corner: zone plan is the braking profile; physics gate inactive
        allowed_speed = zone_limit

    # ------------------------------------------------------------------ #
    # Throttle                                                            #
    # ------------------------------------------------------------------ #
    if speed < allowed_speed:
        R['accel'] += .05
    else:
        R['accel'] -= .01
    if speed < 10:
        R['accel'] += 1.0 / (speed + 0.1)
    R['accel'] = max(0.0, min(1.0, R['accel']))

    c._diag_corner_target = allowed_speed

    # ------------------------------------------------------------------ #
    # Proportional brake pedal + ABS                                     #
    # ------------------------------------------------------------------ #
    overspeed = speed - allowed_speed
    if overspeed > 0:
        R['brake'] = min(1.0, overspeed * BRAKE_GAIN)
        R['accel'] = 0.0
    else:
        R['brake'] = 0.0

    if R['brake'] > 0 and speed > 20:
        slowest_wheel = min(S['wheelSpinVel']) * _WHEEL_RADIUS
        if slowest_wheel < ABS_RATIO * speed / 3.6:
            R['brake'] *= ABS_CUT

    c._diag_overspeed    = overspeed
    c._diag_brake_active = int(overspeed > 0)

    # ------------------------------------------------------------------ #
    # TCS                                                                 #
    # ------------------------------------------------------------------ #
    w = S['wheelSpinVel']
    if (w[2] + w[3]) - (w[0] + w[1]) > 5:
        R['accel'] -= .2

    # ------------------------------------------------------------------ #
    # RPM gearbox (6 gears, v3 model)                                    #
    # ------------------------------------------------------------------ #
    gear = int(S['gear'])
    if gear < 1 or speed < 10:
        gear = 1
    else:
        axle_rpm = ((w[2] + w[3]) / 2.0
                    * _GEAR_RATIOS[gear - 1] * _FINAL_DRIVE * 60.0 / (2.0 * PI))
        if gear < 6 and axle_rpm > UPSHIFT_RPM:
            gear += 1
        elif gear > 2:
            lower_eng_rpm = float(S.get('rpm', 0)) * _GEAR_RATIOS[gear-2] / _GEAR_RATIOS[gear-1]
            if lower_eng_rpm < DOWNSHIFT_RPM:
                gear -= 1
    R['gear'] = gear


# == MAIN ====================================================================

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

            # If server closed the socket during get_servers_input, stop.
            if not C.so:
                exit_status = 'server_shutdown'
                break

            drive_example(C)

            # clip_to_limits is called inside DriverAction.__repr__, but we
            # need the clipped values for logging *before* repr is called.
            C.R.clip_to_limits()

            # Log this step (after actions are clipped, before sending).
            # drive_example stores diagnostics on C._diag_* attributes.
            logger.on_step(
                C,
                rpm=float(C.S.d.get('rpm', 0.0) or 0.0),
                corner_target=getattr(C, '_diag_corner_target', 0.0),
                overspeed_kmh=getattr(C, '_diag_overspeed',    0.0),
                brake_active=getattr(C,  '_diag_brake_active', 0),
            )

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
