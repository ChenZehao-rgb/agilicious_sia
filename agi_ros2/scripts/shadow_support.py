"""Pure, testable navigation and read-only MSP evidence transformations."""
import math
import struct


def fresh(now, stamp, limit):
    return math.isfinite(stamp) and 0 <= now - stamp <= limit


def navigation_accuracy_ok(accuracies, limits):
    """Unknown receiver accuracy or an unfilled flight threshold cannot grant readiness."""
    return (len(accuracies) == len(limits) == 3 and
            all(math.isfinite(value) and math.isfinite(limit) and 0 < value <= limit
                for value, limit in zip(accuracies, limits)))


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


def evidence_config_path(runtime_config, bridge_config):
    if runtime_config and bridge_config:
        raise ValueError('runtime_config and bridge_config cannot both be set')
    if not runtime_config and not bridge_config:
        raise ValueError('runtime_config or bridge_config is required')
    return runtime_config or bridge_config


class MspEvidence:
    """Only API 1.48 / BTFL layout is verified against the local firmware.

    Unknown versions never grant authority. Config errors latch for this session.
    Actual mode flags use BOXIDS positions, not permanent-ID bit positions.
    """
    CONFIG_CODES = (1, 2, 3, 34, 238, 64, 44, 119, 111, 125)
    OVERRIDE_SETTINGS = ('msp_override_channels_mask', 'msp_override_failsafe', 'msp_override_timeout_ms')
    # These modes change the meaning of AETR or add another outer controller.
    CONFLICTING_MODES = {1, 2, 3, 5, 6, 7, 11, 12, 17, 29, 30, 35, 46, 47, 49, 56}

    def __init__(self, expected):
        self.expected = expected
        self.expected_override_timeout_ms = expected.get('expected_override_timeout_ms', 50)
        if type(self.expected_override_timeout_ms) is not int or self.expected_override_timeout_ms != 50:
            raise ValueError('expected_override_timeout_ms must be 50 for the hardware safety policy')
        self.aux_indices = tuple(expected.get(name, default) for name, default in (
            ('arm_aux', 0), ('auto_aux', 1), ('kill_aux', 2)))
        if (len(set(self.aux_indices)) != 3 or
                any(type(index) is not int or not 0 <= index < 14 for index in self.aux_indices)):
            raise ValueError('arm_aux, auto_aux and kill_aux must be distinct zero-based AUX indices 0..13')
        self.arm_channel, self.auto_channel, self.kill_channel = (4 + index for index in self.aux_indices)
        self.session = None
        self.frames = {}
        self.settings = {}
        self.failed = False
        self.reason = 'Waiting for configuration readback'
        self.last_event = float('nan')
        self.receiver_snapshot = None

    def accept(self, code, payload, stamp, session, name='', event='rx'):
        if session != self.session:
            self.session = session
            self.frames.clear()
            self.settings.clear()
            self.failed = False
            self.receiver_snapshot = None
        if event in ('error', 'timeout', 'transport_error'):
            self.failed = True
            self.reason = 'MSP transport/request failure; restart required'
            return
        if event != 'rx' or not math.isfinite(stamp) or stamp <= 0:
            return
        self.last_event = stamp
        payload = bytes(payload)
        if code == 0x3010:
            if name not in self.OVERRIDE_SETTINGS:
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
        if any(c not in self.frames for c in self.CONFIG_CODES) or len(self.settings) != len(self.OVERRIDE_SETTINGS):
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
            if self.settings['msp_override_timeout_ms'][0] != str(self.expected_override_timeout_ms):
                return False, 'Override timeout must be 50 ms; configure compatible FC firmware'
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
            for box, aux in zip((0, 50, 27), self.aux_indices):
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
                      battery_stamp=0., transport_healthy=not self.failed and fresh(now, self.last_event, .2),
                      imu_ready=False, control_mode_ok=False, pid_profile=255, rate_profile=255,
                      override_timeout_ms=0, reason=reason)
        if 'msp_override_timeout_ms' in self.settings:
            try:
                value = int(self.settings['msp_override_timeout_ms'][0])
                if 0 <= value <= 0xffffffff:
                    result['override_timeout_ms'] = value
            except ValueError:
                pass
        if 130 in self.frames:
            p, t = self.frames[130]
            result['battery_stamp'] = t
            if len(p) >= 11 and p[0] > 0 and fresh(now, t, battery_timeout):
                volts = u16(p, 9) * .01
                if volts > 0:
                    result['battery_voltage'] = volts
        if 105 not in self.frames or 150 not in self.frames:
            return result
        rc, rt = self.frames[105]
        status, st = self.frames[150]
        result.update(rc_stamp=rt, status_stamp=st)
        if not verified or not fresh(now, rt, .1) or not fresh(now, st, .1):
            if verified:
                result['reason'] = 'RC/STATUS stale'
            return result
        try:
            if len(rc) < 2 * (5 + max(self.aux_indices)) or len(rc) % 2 or len(status) < 16:
                raise ValueError('Truncated RC/STATUS_EX')
            channels = struct.unpack('<'+'H'*(len(rc)//2), rc)
            if any(not 900 <= channels[index] <= 2100 for index in (self.arm_channel, self.auto_channel, self.kill_channel)):
                raise ValueError('Invalid authority AUX')
            extra = status[15]
            if extra > 15 or len(status) < 23 + extra or status[16+extra] != 30:
                raise ValueError('Unsupported STATUS_EX flags layout')
            modes = status[6:10] + status[16:16+extra]
            flags = struct.unpack_from('<I', status, 17+extra)[0]
            result.update(pid_profile=status[10], rate_profile=status[14])
            profiles_ok = (status[10] == self.expected.get('expected_pid_profile', 0) and
                           status[14] == self.expected.get('expected_rate_profile', 0))
            if not profiles_ok:
                result['config_verified'] = False
                raise ValueError('PID/rate profile differs from hardware configuration')
            ids = self.frames[119][0]
            def mode(box):
                index = ids.index(box)
                if index//8 >= len(modes):
                    raise ValueError('Truncated mode bits')
                return bool(modes[index//8] & (1 << (index % 8)))
            def high(index):
                return self.expected['aux_low'] <= channels[index] < self.expected['aux_high']
            rc_link = not (flags & ((1 << 1) | (1 << 2) | (1 << 4))) and not mode(27)
            kill = high(self.kill_channel) or mode(27) or not rc_link
            sensor_mask = u16(status, 4)
            imu_ready = (sensor_mask & 0x21) == 0x21 and not (flags & ((1 << 12) | (1 << 23)))
            # RC and STATUS are separate requests. On a rising transition, keep
            # the last coherent physical low until both agree. Never manufacture
            # a low at startup or renew the old snapshot's acquisition times.
            # Falling transitions, KILL, ARM low and RX loss revoke immediately.
            if high(self.auto_channel) != mode(50):
                previous = self.receiver_snapshot
                if (not kill and rc_link and imu_ready and
                        previous is not None and not previous['auto_switch'] and not previous['kill'] and
                        fresh(now, previous['rc_stamp'], .1) and fresh(now, previous['status_stamp'], .1) and
                        (not previous['armed'] or (high(self.arm_channel) and mode(0)))):
                    result.update(previous)
                    result.update(imu_ready=imu_ready, control_mode_ok=True,
                                  reason='Waiting for coherent AUTO rise; retaining fresh physical low')
                    return result
                raise ValueError('AUX/FC AUTO disagreement')
            conflicting = sorted(box for box in self.CONFLICTING_MODES if box in ids and mode(box))
            control_mode_ok = not high(self.auto_channel) or not conflicting
            result.update(receiver_valid=True, armed=high(self.arm_channel) and mode(0), auto_switch=high(self.auto_channel),
                          kill=kill, rc_link=rc_link, imu_ready=imu_ready, control_mode_ok=control_mode_ok,
                          reason=('Physical receiver evidence decoded' if control_mode_ok else
                                  'AUTO conflicts with FC mode IDs: ' + ','.join(map(str, conflicting))))
            self.receiver_snapshot = {key: result[key] for key in (
                'receiver_valid', 'armed', 'auto_switch', 'kill', 'rc_link', 'rc_stamp', 'status_stamp')}
        except (IndexError, ValueError, struct.error) as error:
            result['reason'] = str(error)
        return result
