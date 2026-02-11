#!/usr/bin/env python3
'''security module - command validation and threat detection'''

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from pymavlink import mavutil
import time
from collections import deque
import os
import json
from datetime import datetime
import math

class SecurityModule(mp_module.MPModule):
    def __init__(self, mpstate):
        '''initialize security module'''
        super(SecurityModule, self).__init__(mpstate, "security", "security validation")
        
        self.security_settings = mp_settings.MPSettings([
            ('enabled', bool, True),
            ('rate_limit_enabled', bool, True),
            ('rate_limit_max', int, 10),
            ('rate_limit_burst', int, 20),
            ('geofence_enabled', bool, False),
            ('geofence_center_lat', float, 0.0),
            ('geofence_center_lon', float, 0.0),
            ('geofence_radius', int, 1000),
            ('geofence_max_alt', int, 120),
            ('log_enabled', bool, True),
            ('log_path', str, 'security_logs/'),
        ])
        
        self.add_completion_function('(SECURITYSETTING)', self.security_settings.completion)
        
        self.stats = {'messages': 0, 'commands': 0, 'allowed': 0, 'blocked': 0, 
                      'rate_hits': 0, 'geo_violations': 0, 'start': time.time()}
        
        self.rate_tokens = self.security_settings.rate_limit_burst
        self.rate_last_update = time.time()
        self.command_history = deque(maxlen=100)
        self.home_position = None
        self.home_alt = None
        self.log_file = None
        self.geofence_center_set = False
        
        if self.security_settings.log_enabled:
            self._setup_logging()
        
        self.add_command('security', self.cmd_security, "security commands",
                        ['status', 'enable', 'disable', 'stats', 'set'])
    def _setup_logging(self):
        '''set up security logging'''
        log_dir = self.security_settings.log_path
        # Sanitize path to prevent traversal
        log_dir = os.path.normpath(log_dir)
        if log_dir.startswith('..') or os.path.isabs(log_dir):
            print("Security: Invalid log path, using default")
            log_dir = 'security_logs/'
        if not os.path.exists(log_dir):
            try:
                os.makedirs(log_dir)
            except OSError as e:
                print("Security: Failed to create log directory: %s" % e)
                log_dir = './'
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = os.path.join(log_dir, 'security_%s.log' % timestamp)
    
    def idle_task(self):
        '''called periodically'''
        if self.security_settings.rate_limit_enabled:
            now = time.time()
            time_passed = now - self.rate_last_update
            tokens_to_add = time_passed * self.security_settings.rate_limit_max
            self.rate_tokens = min(self.security_settings.rate_limit_burst,
                                  self.rate_tokens + tokens_to_add)
            self.rate_last_update = now
    
    def mavlink_packet(self, msg):
        '''process incoming MAVLink messages'''
        if not self.security_settings.enabled:
            return
        
        self.stats['messages'] += 1
        msg_type = msg.get_type()
        
        if msg_type == 'HOME_POSITION':
            self.home_position = (msg.latitude / 1e7, msg.longitude / 1e7)
            if hasattr(msg, 'altitude'):
                self.home_alt = msg.altitude / 1000.0
            if self.security_settings.geofence_enabled and not self.geofence_center_set:
                self.security_settings.geofence_center_lat = self.home_position[0]
                self.security_settings.geofence_center_lon = self.home_position[1]
                self.geofence_center_set = True
        
        if self._is_command(msg_type):
            self.stats['commands'] += 1
            if self._validate_command(msg):
                self.stats['allowed'] += 1
            else:
                self.stats['blocked'] += 1
                if self.security_settings.log_enabled:
                    self._log_event(msg_type, 'BLOCKED')
    
    def _is_command(self, msg_type):
        '''check if message type is a command'''
        return msg_type in ['COMMAND_LONG', 'COMMAND_INT', 'MISSION_ITEM', 
                           'MISSION_ITEM_INT', 'SET_POSITION_TARGET_GLOBAL_INT',
                           'SET_MODE', 'PARAM_SET']
    
    def _validate_command(self, msg):
        '''validate command returns True if allowed, False if blocked'''
        if self.security_settings.rate_limit_enabled:
            if self.rate_tokens < 1.0:
                self.stats['rate_hits'] += 1
                return False
            self.rate_tokens -= 1.0
            self.command_history.append(time.time())
        
        if self.security_settings.geofence_enabled and not self._check_geofence(msg):
            self.stats['geo_violations'] += 1
            return False
        
        return True
    
    def _check_geofence(self, msg):
        '''check if command violates geofence'''
        msg_type = msg.get_type()
        if 'MISSION_ITEM' in msg_type and hasattr(msg, 'x') and hasattr(msg, 'frame'):
            # Only check lat/lon for global frames (0-6 are global frame types)
            if msg.frame > 6:
                return True  # Skip check for local frames
            dist = self._calc_distance(self.security_settings.geofence_center_lat,
                                      self.security_settings.geofence_center_lon, msg.x, msg.y)
            if dist > self.security_settings.geofence_radius:
                return False        
            if hasattr(msg, 'alt'):
                alt_to_check = msg.alt
                frame = getattr(msg, 'coordinate_frame', getattr(msg, 'frame', None))
                if frame == mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT and self.home_alt is not None:
                    alt_to_check = msg.alt + self.home_alt
                if alt_to_check > self.security_settings.geofence_max_alt:
                    return False
        
        if 'MISSION_ITEM' in msg_type and hasattr(msg, 'x'):
            dist = self._calc_distance(self.security_settings.geofence_center_lat,
                                      self.security_settings.geofence_center_lon, msg.x, msg.y)
            if dist > self.security_settings.geofence_radius:
                return False
        
        return True
    
    def _calc_distance(self, lat1, lon1, lat2, lon2):
        '''calculate distance between coordinates in meters'''
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    def _log_event(self, msg_type, status):
        '''log security event'''
        if not self.security_settings.log_enabled:
            return
        
        if self.log_file is None or not hasattr(self, 'log_file'):
            self._setup_logging()
        
        if not self.log_file or not isinstance(self.log_file, (str, os.PathLike)):
            print("Security: Log file not properly configured, using fallback")
            self.log_file = 'security_fallback.log'
        
        try:
            entry = {'time': datetime.now().isoformat(), 'type': msg_type, 'status': status}
            with open(self.log_file, 'a') as f:
                f.write(json.dumps(entry) + '\n')
        except (TypeError, OSError, IOError) as e:
            print("Security: Failed to write log to %s: %s" % (self.log_file, str(e)))
    
    def cmd_security(self, args):
        '''handle security commands'''
        if len(args) == 0:
            print("Usage: security <status|enable|disable|stats|set>")
            return
        
        cmd = args[0].lower()
        
        if cmd == 'status':
            self._cmd_status()
        elif cmd == 'enable':
            self.security_settings.enabled = True
            print("Security enabled")
        elif cmd == 'disable':
            self.security_settings.enabled = False
            print("Security disabled")
        elif cmd == 'stats':
            self._cmd_stats()
        elif cmd == 'set':
            if len(args) < 3:
                print("Usage: security set <setting> <value>")
                return
            try:
                setting_name = args[1]
                self.security_settings.set(setting_name, args[2])
                if setting_name in ('geofence_center_lat', 'geofence_center_lon'):
                    self.geofence_center_set = True
                print("Set %s = %s" % (setting_name, args[2]))
            except Exception as e:
                print("Error: %s" % str(e))
        else:
            print("Usage: security <status|enable|disable|stats|set>")
    
    def _cmd_status(self):
        '''display security status'''
        print("Security: %s" % ("enabled" if self.security_settings.enabled else "disabled"))
        print("Rate limiting: %s (%d cmd/s, %.1f tokens)" % (
            "enabled" if self.security_settings.rate_limit_enabled else "disabled",
            self.security_settings.rate_limit_max, self.rate_tokens))
        print("Geofencing: %s (%dm radius)" % (
            "enabled" if self.security_settings.geofence_enabled else "disabled",
            self.security_settings.geofence_radius))
        if self.security_settings.geofence_enabled:
            print("Geofence center: %.6f, %.6f, max alt: %dm" % (
                self.security_settings.geofence_center_lat,
                self.security_settings.geofence_center_lon,
                self.security_settings.geofence_max_alt))
    
    def _cmd_stats(self):
        '''display security statistics'''
        runtime = time.time() - self.stats['start']
        print("Runtime: %dh%dm" % (int(runtime // 3600), int((runtime % 3600) // 60)))
        print("Messages: %d, Commands: %d, Allowed: %d, Blocked: %d" % (
            self.stats['messages'], self.stats['commands'], 
            self.stats['allowed'], self.stats['blocked']))
        print("Rate hits: %d, Geofence violations: %d" % (
            self.stats['rate_hits'], self.stats['geo_violations']))

def init(mpstate):
    '''initialize module'''
    return SecurityModule(mpstate)
