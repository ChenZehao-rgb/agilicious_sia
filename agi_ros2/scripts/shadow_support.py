"""Pure, testable navigation and read-only MSP evidence transformations."""
import math
import struct


def fresh(now, stamp, limit):
    return math.isfinite(stamp) and 0 <= now - stamp <= limit


def ecef(lat, lon, height):
    lat, lon = math.radians(lat), math.radians(lon)
    n = 6378137.0 / math.sqrt(1 - 6.69437999014e-3 * math.sin(lat)**2)
    return ((n + height) * math.cos(lat) * math.cos(lon),
            (n + height) * math.cos(lat) * math.sin(lon),
            (n * (1 - 6.69437999014e-3) + height) * math.sin(lat))


def local_position(origin, sample, basis):
    lat, lon, height = origin
    # For relative MSL, compute horizontal geodesy on the ellipsoid surface.
    h0, h1 = (height, sample[2]) if basis == 'ellipsoid' else (0, 0)
    delta = [b - a for a, b in zip(ecef(lat, lon, h0), ecef(sample[0], sample[1], h1))]
    lat, lon = math.radians(lat), math.radians(lon)
    x, y, z = delta
    east = -math.sin(lon)*x + math.cos(lon)*y
    north = -math.sin(lat)*math.cos(lon)*x - math.sin(lat)*math.sin(lon)*y + math.cos(lat)*z
    up = math.cos(lat)*math.cos(lon)*x + math.cos(lat)*math.sin(lon)*y + math.sin(lat)*z
    return east, north, up if basis == 'ellipsoid' else sample[2] - height


class OriginBuilder:
    def __init__(self, duration=3.0, count=30, speed=0.3):
        if not math.isfinite(duration) or duration <= 0 or count < 2 or not math.isfinite(speed) or speed <= 0:
            raise ValueError('invalid origin acquisition parameters')
        self.duration, self.count, self.speed = duration, count, speed
        self.samples = []
        self.origin = None
        self.basis = None
        self.previous = None
        self.source_session = None

    def add(self, t, source_session, lat, lon, height, basis, velocity):
        if not all(math.isfinite(x) for x in (t, lat, lon, height, *velocity)):
            self.samples.clear()
            return None
        if abs(lat) > 90 or abs(lon) > 180 or basis not in ('ellipsoid', 'msl'):
            self.samples.clear()
            return None
        if source_session != self.source_session:
            self.samples.clear()
            self.previous = None
            self.source_session = source_session
        if self.previous is not None and t <= self.previous:
            return None
        if self.previous is not None and t - self.previous > .3:
            self.samples.clear()
        self.previous = t
        if self.origin is not None:
            return local_position(self.origin, (lat, lon, height), basis) if basis == self.basis else None
        if math.sqrt(sum(v*v for v in velocity)) > self.speed:
            self.samples.clear()
            return None
        if basis != self.basis:
            self.samples.clear()
        self.basis = basis
        self.samples.append((t, lat, lon, height))
        # Bound acquisition memory while retaining a window longer than the requested duration.
        while len(self.samples) > max(self.count * 10, 10000):
            self.samples.pop(0)
        if len(self.samples) < self.count or t - self.samples[0][0] < self.duration:
            return None
        # Circular longitude mean also handles a dateline origin.
        latitude = sum(s[1] for s in self.samples) / len(self.samples)
        longitude = math.degrees(math.atan2(sum(math.sin(math.radians(s[2])) for s in self.samples),
                                             sum(math.cos(math.radians(s[2])) for s in self.samples)))
        altitude = sum(s[3] for s in self.samples) / len(self.samples)
        self.origin = latitude, longitude, altitude
        self.samples.clear()
        return local_position(self.origin, (lat, lon, height), basis)


def u16(data, offset=0):
    return struct.unpack_from('<H', data, offset)[0]


class MspEvidence:
    """Only API 1.48 / BTFL layout is verified against the local firmware.

    Unknown versions never grant authority. Config errors latch for this session.
    Actual mode flags use BOXIDS positions, not permanent-ID bit positions.
    """
    CONFIG_CODES = (1, 2, 3, 34, 238, 64, 44, 119, 111, 125)

    def __init__(self, expected):
        self.expected = expected
        self.session = None
        self.frames = {}
        self.settings = {}
        self.failed = False
        self.reason = 'Waiting for configuration readback'
        self.last_event = float('nan')

    def accept(self, code, payload, stamp, session, name='', event='rx'):
        if session != self.session:
            self.session = session
            self.frames.clear()
            self.settings.clear()
            self.failed = False
        if event in ('error', 'timeout', 'transport_error'):
            self.failed = True
            self.reason = 'MSP transport/request failure; restart required'
            return
        if event != 'rx' or not math.isfinite(stamp) or stamp <= 0:
            return
        self.last_event = stamp
        payload = bytes(payload)
        if code == 0x3010:
            if name not in ('msp_override_channels_mask', 'msp_override_failsafe'):
                return
            try:
                key, value = payload.decode('ascii').strip('\x00').split('=', 1)
                if key.strip() != name:
                    raise ValueError('setting reply mismatch')
                value = value.strip()
                if name in self.settings and self.settings[name][0] != value:
                    raise ValueError('configuration changed')
                self.settings[name] = value, stamp
            except (UnicodeError, ValueError):
                self.failed = True
                self.reason = 'Invalid or changed setting readback'
            return
        if code in self.CONFIG_CODES and code in self.frames and self.frames[code][0] != payload:
            self.failed = True
            self.reason = 'Configuration changed; restart required'
        if code not in self.frames or stamp > self.frames[code][1]:
            self.frames[code] = payload, stamp

    def configuration(self, now):
        if self.failed:
            return False, self.reason
        if any(c not in self.frames for c in self.CONFIG_CODES) or len(self.settings) != 2:
            return False, 'Missing configuration readback'
        if any(not fresh(now, self.frames[c][1], 3.0) for c in self.CONFIG_CODES):
            return False, 'Configuration readback stale'
        if any(not fresh(now, s[1], 3.0) for s in self.settings.values()):
            return False, 'Override setting readback stale'
        f = {c: p[0] for c, p in self.frames.items()}
        try:
            if f[1] != bytes((0, 1, 48)) or f[2] != b'BTFL' or len(f[3]) != 3:
                return False, 'Unsupported FC/API: require BTFL API 1.48'
            if self.settings['msp_override_channels_mask'][0] != '15':
                return False, 'Override mask must be 15 (AETR only)'
            if self.settings['msp_override_failsafe'][0] != 'OFF':
                return False, 'Override must not maintain receiver link/failsafe'
            if len(f[44]) < 24 or u16(f[44], 3) != 1500 or u16(f[44], 5) != self.expected['min_check']:
                return False, 'Invalid RX configuration/midrc'
            if list(f[64]) != self.expected['rx_map']:
                return False, 'Receiver map mismatch'
            if len(set(f[119])) != len(f[119]) or not {0, 27, 50}.issubset(f[119]):
                return False, 'Required BOXIDS absent or duplicated'
            ranges, extra = f[34], f[238]
            if len(ranges) % 4 or not extra or len(extra) != 1 + 3*extra[0] or extra[0]*4 != len(ranges):
                return False, 'Malformed mode ranges'
            active = {}
            for i in range(len(ranges)//4):
                box, aux, lo, hi = ranges[4*i:4*i+4]
                if extra[1+3*i] != box:
                    return False, 'Mode range metadata mismatch'
                if lo < hi and box in (0, 27, 50):
                    if extra[2+3*i] != 0 or extra[3+3*i] != 0:
                        return False, 'Linked/AND authority modes unsupported'
                    active.setdefault(box, []).append((aux, 900+25*lo, 900+25*hi))
            for box, aux in ((0, 0), (50, 1), (27, 2)):
                if active.get(box) != [(aux, self.expected['aux_low'], self.expected['aux_high'])]:
                    return False, 'ARM/AUTO/KILL range mismatch'
            rates = f[111]
            if len(rates) < 23 or rates[22] != 3 or rates[14] != 0:
                return False, 'ACTUAL rates required'
            center = [rates[0]*10, rates[12]*10, rates[11]*10]
            maximum = [rates[i]*10 for i in (2, 3, 4)]
            expo = [rates[i] for i in (1, 13, 10)]
            if any(u16(rates, 16+2*i) < maximum[i] for i in range(3)):
                return False, 'Rate limits truncate configured ACTUAL curve'
            if (center != self.expected['center_rate_deg_s'] or maximum != self.expected['max_rate_deg_s'] or
                    expo != self.expected['expo_percent']):
                return False, 'ACTUAL rate readback mismatch'
            if len(f[125]) < 5 or list(f[125][:2]) != [self.expected['deadband'], self.expected['yaw_deadband']]:
                return False, 'Deadband readback mismatch'
        except (IndexError, struct.error, KeyError):
            return False, 'Truncated configuration'
        return True, 'Configuration verified'

    def snapshot(self, now, battery_timeout=1.5):
        verified, reason = self.configuration(now)
        result = dict(config_verified=verified, receiver_valid=False, armed=False, auto_switch=False,
                      kill=True, rc_link=False, battery_voltage=float('nan'), rc_stamp=0., status_stamp=0.,
                      battery_stamp=0., transport_healthy=not self.failed and fresh(now, self.last_event, .2), reason=reason)
        if 130 in self.frames:
            p, t = self.frames[130]
            result['battery_stamp'] = t
            if len(p) >= 11 and p[0] > 0 and fresh(now, t, battery_timeout):
                volts = u16(p, 9) * .01
                if volts > 0:
                    result['battery_voltage'] = volts
        if 105 not in self.frames or 101 not in self.frames:
            return result
        rc, rt = self.frames[105]
        status, st = self.frames[101]
        result.update(rc_stamp=rt, status_stamp=st)
        if not verified or not fresh(now, rt, .1) or not fresh(now, st, .1):
            if verified:
                result['reason'] = 'RC/STATUS stale'
            return result
        try:
            if len(rc) < 14 or len(rc) % 2 or len(status) < 16:
                raise ValueError('Truncated RC/STATUS')
            channels = struct.unpack('<'+'H'*(len(rc)//2), rc)
            if any(not 900 <= c <= 2100 for c in channels[4:7]):
                raise ValueError('Invalid authority AUX')
            extra = status[15]
            if extra > 15 or len(status) < 23 + extra or status[16+extra] != 30:
                raise ValueError('Unsupported STATUS flags layout')
            modes = status[6:10] + status[16:16+extra]
            flags = struct.unpack_from('<I', status, 17+extra)[0]
            ids = self.frames[119][0]
            def mode(box):
                index = ids.index(box)
                if index//8 >= len(modes):
                    raise ValueError('Truncated mode bits')
                return bool(modes[index//8] & (1 << (index % 8)))
            def high(index):
                return self.expected['aux_low'] <= channels[index] < self.expected['aux_high']
            rc_link = not (flags & ((1 << 1) | (1 << 2) | (1 << 4))) and not mode(27)
            kill = high(6) or mode(27) or not rc_link
            # A mismatched AUTO observation cannot synthesize a rising edge.
            if high(5) != mode(50):
                raise ValueError('AUX/FC AUTO disagreement')
            result.update(receiver_valid=True, armed=high(4) and mode(0), auto_switch=high(5),
                          kill=kill, rc_link=rc_link, reason='Physical receiver evidence decoded')
        except (IndexError, ValueError, struct.error) as error:
            result['reason'] = str(error)
        return result
