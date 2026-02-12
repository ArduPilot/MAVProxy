#!/usr/bin/env python3
'''
rate limiting and geofencing checks for MAVLink commands
'''

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from pymavlink import mavutil

import math
import os
import time
import json
from datetime import datetime
from collections import deque

class CommandGuardModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(CommandGuardModule, self).__init__(mpstate, "command_guard")

        # Settings
        self.guard_settings = mp_settings.MPSettings([
            ('enabled', bool, True),

            ('rate_limit_enabled', bool, True),
            ('rate_limit_hz', int, 10),
            ('rate_limit_burst', int, 20),

            ('geofence_enabled', bool, False),
            ('geofence_lat', float, 0.0),
            ('geofence_lon', float, 0.0),
            ('geofence_radius', int, 1000),
            ('geofence_max_alt', int, 120),

            ('log_enabled', bool, True),
            ('log_dir', str, 'safety_logs'),
        ])

        self.add_completion_function(
            '(SAFETYSETTING)',
            self.guard_settings.completion
        )

        # Runtime State
        self._tokens = float(self.guard_settings.rate_limit_burst)
        self._last_refill = time.time()

        self._recent_commands = deque(maxlen=100)

        self._home_alt = None
        self._geofence_initialized = False

        self.stats = {
            'messages_seen': 0,
            'commands_seen': 0,
            'commands_blocked': 0,
            'rate_limited': 0,
            'geofence_blocked': 0,
            'start_time': time.time(),
        }

        self._log_file = None
        if self.guard_settings.log_enabled:
            self._init_logging()

        # CLI Commands
        self.add_command(
            'command_guard',
            self.cmd_command_guard,
            'command guard module commands',
            ['status', 'stats', 'enable', 'disable', 'set']
        )

    def idle_task(self):
        """Refill tokens for rate limiting"""
        if not self.guard_settings.rate_limit_enabled:
            return

        now = time.time()
        elapsed = now - self._last_refill
        self._last_refill = now

        refill = elapsed * self.guard_settings.rate_limit_hz
        self._tokens = min(
            self.guard_settings.rate_limit_burst,
            self._tokens + refill
        )

    def mavlink_packet(self, msg):
        """Process incoming MAVLink packets from vehicle"""
        if not self.guard_settings.enabled:
            return

        self.stats['messages_seen'] += 1
        msg_type = msg.get_type()

        # Only handle incoming HOME_POSITION from vehicle
        if msg_type == 'HOME_POSITION':
            self._handle_home_position(msg)
            return

    def master_send(self, msg):
        """Intercept and validate outgoing commands before sending to vehicle"""
        if not self.guard_settings.enabled:
            return True

        msg_type = msg.get_type()

        if not self._is_command(msg_type):
            return True

        self.stats['commands_seen'] += 1

        if not self._check_rate_limit():
            self.stats['rate_limited'] += 1
            self.stats['commands_blocked'] += 1
            self._log_event(msg_type, 'rate_limit')
            print(f"Command blocked: rate limit exceeded")
            return False

        if self.guard_settings.geofence_enabled:
            if not self._check_geofence(msg):
                self.stats['geofence_blocked'] += 1
                self.stats['commands_blocked'] += 1
                self._log_event(msg_type, 'geofence')
                print(f"Command blocked: geofence violation")
                return False

        self._recent_commands.append(time.time())
        return True

    def _is_command(self, msg_type):
        return msg_type in {
            'COMMAND_LONG',
            'COMMAND_INT',
            'MISSION_ITEM',
            'MISSION_ITEM_INT',
            'SET_MODE',
            'PARAM_SET',
            'SET_POSITION_TARGET_GLOBAL_INT',
        }
    
    def _check_rate_limit(self):
        if not self.guard_settings.rate_limit_enabled:
            return True

        if self._tokens < 1.0:
            return False

        self._tokens -= 1.0
        return True
    
    def _check_geofence(self, msg):
        """Check geofence for global mission items and position target commands"""
        msg_type = msg.get_type()

        if 'MISSION_ITEM' not in msg_type and 'POSITION_TARGET' not in msg_type:
            return True

        if msg_type == 'SET_POSITION_TARGET_GLOBAL_INT':
            if hasattr(msg, 'lat_int') and hasattr(msg, 'lon_int'):
                lat = msg.lat_int / 1e7
                lon = msg.lon_int / 1e7
                
                distance = self._haversine(
                    self.guard_settings.geofence_lat,
                    self.guard_settings.geofence_lon,
                    lat,
                    lon
                )
                
                if distance > self.guard_settings.geofence_radius:
                    return False
                
                if hasattr(msg, 'alt'):
                    if msg.alt > self.guard_settings.geofence_max_alt:
                        return False
            return True

        # Handle MISSION_ITEM and MISSION_ITEM_INT
        if not hasattr(msg, 'frame'):
            return True

        if msg.frame not in (
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        ):
            return True
        
        if msg_type == 'MISSION_ITEM_INT':
            lat = msg.x / 1e7
            lon = msg.y / 1e7
        else:
            lat = msg.x
            lon = msg.y

        distance = self._haversine(
            self.guard_settings.geofence_lat,
            self.guard_settings.geofence_lon,
            lat,
            lon
        )

        if distance > self.guard_settings.geofence_radius:
            return False

        if hasattr(msg, 'z'):
            altitude = msg.z
            if msg.frame in (
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            ) and self._home_alt is not None:
                altitude += self._home_alt

            if altitude > self.guard_settings.geofence_max_alt:
                return False

        return True
    
    def _handle_home_position(self, msg):
        if hasattr(msg, 'altitude'):
            self._home_alt = msg.altitude / 1000.0

        if self.guard_settings.geofence_enabled and not self._geofence_initialized:
            lat = msg.latitude / 1e7
            lon = msg.longitude / 1e7

            # Ignore invalid home positions at (0,0)
            if abs(lat) > 1e-6 or abs(lon) > 1e-6:
                self.guard_settings.geofence_lat = lat
                self.guard_settings.geofence_lon = lon
                self._geofence_initialized = True

    def _haversine(self, lat1, lon1, lat2, lon2):
        """Return distance in meters between two GPS points"""
        R = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = (math.sin(dphi / 2) ** 2 +
             math.cos(phi1) * math.cos(phi2) *
             math.sin(dlambda / 2) ** 2)

        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    # Logging
    def _init_logging(self):
        path = os.path.normpath(self.guard_settings.log_dir)
        os.makedirs(path, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_file = os.path.join(path, f'safety_{ts}.log')

    def _log_event(self, msg_type, reason):
        """Log a blocked command event to the log file"""
        if not self.guard_settings.log_enabled or not self._log_file:
            return

        record = {
            'time': datetime.utcnow().isoformat(),
            'message_type': msg_type,
            'blocked_reason': reason,
        }

        try:
            with open(self._log_file, 'a') as f:
                f.write(json.dumps(record) + '\n')
        except OSError:
            pass

    # CLI Commands
    def cmd_command_guard(self, args):
        if not args:
            self._cmd_status()
            return

        cmd = args[0]

        if cmd == 'enable':
            self.guard_settings.enabled = True
            print("Safety checks enabled")
        elif cmd == 'disable':
            self.guard_settings.enabled = False
            print("Safety checks disabled")
        elif cmd == 'status':
            self._cmd_status()
        elif cmd == 'stats':
            self._cmd_stats()
        elif cmd == 'set' and len(args) >= 3:
            self.guard_settings.set(args[1], args[2])
            print(f"Set {args[1]} = {args[2]}")
        else:
            print("Usage: command_guard <status|stats|enable|disable|set>")

    def _cmd_status(self):
        print(f"Enabled: {self.guard_settings.enabled}")
        print(f"Rate limit: {self.guard_settings.rate_limit_hz} Hz "
              f"(tokens {self._tokens:.1f})")
        print(f"Geofence enabled: {self.guard_settings.geofence_enabled}")

    def _cmd_stats(self):
        uptime = int(time.time() - self.stats['start_time'])
        print(f"Uptime: {uptime}s")
        for key, value in self.stats.items():
            if key != 'start_time':
                print(f"{key}: {value}")


def init(mpstate):
    return CommandGuardModule(mpstate)