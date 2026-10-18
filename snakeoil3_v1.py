#!/usr/bin/python
# snakeoil3_v1.py
# Derived from snakeoil3_gym.py (original, preserved unchanged).
# V1 adds: Windows-compatible connection wait, CSV telemetry logging via
# race_logging.py.  drive_example is identical to the original.
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
# Change description vs original:
#   - Windows-compatible bounded connection wait (no pkill / sh / &)
#   - CSV logging via race_logging.RaceLogger
#   - drive_example: target_speed reduced 300→90 (conservative test hypothesis)
#   - --car CLI option added
#   - -nodamage / -nolaptime flags removed (not used here)
#
# This file is the baseline candidate.  It is NOT a confirmed baseline
# until it completes a full lap on the target machine.

import socket
import sys
import getopt
import os
import time

from race_logging import RaceLogger

PI = 3.14159265359

data_size = 2**17

# Version label written into every CSV row.
V1_VERSION = 'v1'

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
        self.change_desc    = 'baseline'
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
# Change from snakeoil3_gym.py (original):
#   target_speed: 300 → 90  (single conservative hypothesis; not a proven fix)
#   All other logic identical: angle multiplier 15/PI, accel step ±0.01,
#   no active braking, same gear thresholds.
# NOTE on 1/(speedX+.1): if speedX == -0.1 exactly the denominator is zero
#   and a ZeroDivisionError will be raised.  This has NOT occurred in any
#   recorded run yet.  If it does, the documented repair is:
#     Replace:  R['accel'] += 1 / (S['speedX'] + .1)
#     With:     if S['speedX'] > -0.09:
#                   R['accel'] += 1 / (S['speedX'] + .1)
#   That change must be made separately and documented as its own repair.

def drive_example(c):
    '''This is only an example. It will get around the track but the
    correct thing to do is write your own `drive()` function.'''
    S, R = c.S.d, c.R.d
    target_speed = 90   # reduced from 300; conservative test hypothesis

    # Steer To Corner
    R['steer'] = S['angle'] * 15 / PI
    # Steer To Center
    R['steer'] -= S['trackPos'] * .10

    # Throttle Control
    if S['speedX'] < target_speed - (R['steer'] * 50):
        R['accel'] += .01
    else:
        R['accel'] -= .01
    if S['speedX'] < 10:
        R['accel'] += 1 / (S['speedX'] + .1)

    # Traction Control System
    if ((S['wheelSpinVel'][2] + S['wheelSpinVel'][3]) -
            (S['wheelSpinVel'][0] + S['wheelSpinVel'][1]) > 5):
        R['accel'] -= .2

    # Automatic Transmission
    R['gear'] = 1
    if S['speedX'] > 50:  R['gear'] = 2
    if S['speedX'] > 80:  R['gear'] = 3
    if S['speedX'] > 110: R['gear'] = 4
    if S['speedX'] > 140: R['gear'] = 5
    if S['speedX'] > 170: R['gear'] = 6
    return


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

            # Log this step (after actions are clipped, before sending)
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
