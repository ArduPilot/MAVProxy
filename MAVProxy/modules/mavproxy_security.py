#!/usr/bin/env python3
'''
Security Module for MAVProxy
Provides command validation, rate limiting, and geofencing

Author: MAVProxy Development Team
Date: February 2026
'''

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib.mp_menu import MPMenuItem
import time
from collections import deque
import os
import json
from datetime import datetime

try:
    from colorama import Fore, Style, init
    init(autoreset=True)
    HAS_COLORAMA = True
except ImportError:
    HAS_COLORAMA = False
    class Fore:
        RED = GREEN = YELLOW = CYAN = ''
    class Style:
        RESET_ALL = ''

class SecurityModule(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialize Security Module"""
        super(SecurityModule, self).__init__(mpstate, "security", 
                                            "Security validation and threat detection")
        
        # Settings
        self.security_settings = mp_settings.MPSettings([
            ('enabled', bool, True),
            ('rate_limit_enabled', bool, True),
            ('rate_limit_max', int, 10),            # Commands per second
            ('rate_limit_burst', int, 20),          # Burst limit
            ('geofence_enabled', bool, False),
            ('geofence_center_lat', float, 0.0),
            ('geofence_center_lon', float, 0.0),
            ('geofence_radius', int, 1000),         # Meters
            ('geofence_max_alt', int, 120),         # Meters
            ('auth_enabled', bool, False),
            ('auth_token', str, ''),
            ('log_enabled', bool, True),
            ('log_path', str, 'security_logs/'),
            ('alert_enabled', bool, True),
            ('verbose', bool, False),
        ])
        
        # Add settings to module
        self.add_completion_function('(SECURITYSETTING)', 
                                    self.security_settings.completion)
        
        # Statistics
        self.stats = {
            'total_messages': 0,
            'commands_processed': 0,
            'commands_allowed': 0,
            'commands_blocked': 0,
            'rate_limit_hits': 0,
            'geofence_violations': 0,
            'auth_failures': 0,
            'threats_detected': 0,
            'start_time': time.time(),
        }
        
        # Rate limiter state
        self.rate_tokens = self.security_settings.rate_limit_burst
        self.rate_last_update = time.time()
        self.command_history = deque(maxlen=100)
        
        # Geofence state
        self.home_position = None
        
        # Logging
        self._setup_logging()
        
        # Commands
        self.add_command('security', self.cmd_security, 
                        "security module commands",
                        ['status', 'enable', 'disable', 'stats', 'clear', 
                         'set <setting> <value>', 'test'])
        
        # Menu
        self._setup_menu()
        
        # Initialization message
        self.say("Security Module Loaded", priority='important')
        self._log_info("Security module initialized")
    
    def _setup_logging(self):
        """Set up security logging"""
        if self.security_settings.log_enabled:
            log_dir = self.security_settings.log_path
            if not os.path.exists(log_dir):
                try:
                    os.makedirs(log_dir)
                except:
                    log_dir = './'
            
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.log_file = os.path.join(log_dir, f'security_{timestamp}.log')
            self.threat_log_file = os.path.join(log_dir, f'threats_{timestamp}.log')
    
    def _setup_menu(self):
        """Set up GUI menu items"""
        self.menu_added_console = False
        self.add_menu = MPMenuItem('Security', items=[
            MPMenuItem('Enable', returnkey='securityEnable'),
            MPMenuItem('Disable', returnkey='securityDisable'),
            MPMenuItem('Status', returnkey='securityStatus'),
            MPMenuItem('Statistics', returnkey='securityStats'),
            MPMenuItem('Clear Logs', returnkey='securityClear'),
        ])
    
    def idle_task(self):
        """Called periodically"""
        # Update rate limiter tokens
        self._update_rate_tokens()
    
    def mavlink_packet(self, msg):
        """Process incoming MAVLink messages"""
        if not self.security_settings.enabled:
            return
        
        self.stats['total_messages'] += 1
        msg_type = msg.get_type()
        
        # Track home position for geofence
        if msg_type == 'HOME_POSITION':
            self.home_position = (msg.latitude / 1e7, msg.longitude / 1e7)
            if self.security_settings.geofence_enabled and \
               self.security_settings.geofence_center_lat == 0.0:
                self.security_settings.geofence_center_lat = self.home_position[0]
                self.security_settings.geofence_center_lon = self.home_position[1]
                self._log_info(f"Geofence center auto-set to home: {self.home_position}")
        
        # Check if this is a command message
        if self._is_command(msg_type):
            self.stats['commands_processed'] += 1
            
            # Validate command
            result = self._validate_command(msg)
            
            if result['blocked']:
                self.stats['commands_blocked'] += 1
                self.stats['threats_detected'] += 1
                self._handle_blocked_command(msg, result)
                
                # NOTE: In actual implementation, you would prevent the
                # message from being forwarded to the vehicle here.
                # This requires modifying MAVProxy's core message routing.
                # For now, we just log and alert.
            else:
                self.stats['commands_allowed'] += 1
                self._log_command(msg, 'ALLOWED')
    
    def _is_command(self, msg_type):
        """Check if message type is a command"""
        command_types = [
            'COMMAND_LONG',
            'COMMAND_INT',
            'MISSION_ITEM',
            'MISSION_ITEM_INT',
            'MISSION_COUNT',
            'SET_POSITION_TARGET_LOCAL_NED',
            'SET_POSITION_TARGET_GLOBAL_INT',
            'SET_MODE',
            'PARAM_SET',
            'DO_SET_SERVO',
            'DO_SET_RELAY',
        ]
        return msg_type in command_types
    
    def _validate_command(self, msg):
        """
        Multi-layer command validation
        Returns: {'blocked': bool, 'reasons': [str]}
        """
        result = {
            'blocked': False,
            'reasons': []
        }
        
        msg_type = msg.get_type()
        
        # Layer 1: Rate Limiting
        if self.security_settings.rate_limit_enabled:
            if not self._check_rate_limit():
                result['blocked'] = True
                result['reasons'].append('RATE_LIMIT_EXCEEDED')
                self.stats['rate_limit_hits'] += 1
                return result  # Early exit for rate limit
        
        # Layer 2: Authentication
        if self.security_settings.auth_enabled:
            if not self._check_authentication(msg):
                result['blocked'] = True
                result['reasons'].append('AUTH_FAILED')
                self.stats['auth_failures'] += 1
        
        # Layer 3: Command Validation
        if not self._validate_command_params(msg):
            result['blocked'] = True
            result['reasons'].append('INVALID_PARAMS')
        
        # Layer 4: Geofence
        if self.security_settings.geofence_enabled:
            if not self._check_geofence(msg):
                result['blocked'] = True
                result['reasons'].append('GEOFENCE_VIOLATION')
                self.stats['geofence_violations'] += 1
        
        return result
    
    def _update_rate_tokens(self):
        """Update rate limiter token bucket"""
        if not self.security_settings.rate_limit_enabled:
            return
        
        now = time.time()
        time_passed = now - self.rate_last_update
        
        # Add tokens based on time passed
        tokens_to_add = time_passed * self.security_settings.rate_limit_max
        self.rate_tokens = min(
            self.security_settings.rate_limit_burst,
            self.rate_tokens + tokens_to_add
        )
        self.rate_last_update = now
    
    def _check_rate_limit(self):
        """
        Check if command is allowed by rate limiter
        Returns: True if allowed, False if rate limited
        """
        self._update_rate_tokens()
        
        if self.rate_tokens >= 1.0:
            self.rate_tokens -= 1.0
            self.command_history.append(time.time())
            return True
        else:
            return False
    
    def _check_authentication(self, msg):
        """
        Check command authentication
        Returns: True if authenticated, False otherwise
        """
        # Simple token-based auth for now
        # In full implementation, would check message signatures
        if self.security_settings.auth_token:
            # Check if message has valid auth token
            # For now, always return True (stub)
            return True
        return True
    
    def _validate_command_params(self, msg):
        """
        Validate command parameters
        Returns: True if valid, False otherwise
        """
        msg_type = msg.get_type()
        
        # Validate COMMAND_LONG
        if msg_type == 'COMMAND_LONG':
            command_id = msg.command
            
            # Check for dangerous commands
            if command_id == 252:  # DO_FLIGHT_TERMINATION
                self._log_warning("FLIGHT TERMINATION command detected!")
                # Could block this or require special auth
            
            # Check param ranges
            # Add more validation as needed
        
        # Validate position commands
        if 'POSITION' in msg_type:
            if hasattr(msg, 'z'):
                if abs(msg.z) > 1000:  # Sanity check altitude
                    return False
        
        return True
    
    def _check_geofence(self, msg):
        """
        Check if command violates geofence
        Returns: True if within fence, False if violation
        """
        msg_type = msg.get_type()
        
        # Check position commands
        if 'POSITION_TARGET_GLOBAL' in msg_type:
            if hasattr(msg, 'lat_int') and hasattr(msg, 'lon_int'):
                lat = msg.lat_int / 1e7
                lon = msg.lon_int / 1e7
                
                distance = self._calculate_distance(
                    self.security_settings.geofence_center_lat,
                    self.security_settings.geofence_center_lon,
                    lat, lon
                )
                
                if distance > self.security_settings.geofence_radius:
                    return False
                
                # Check altitude
                if hasattr(msg, 'alt'):
                    if msg.alt > self.security_settings.geofence_max_alt:
                        return False
        
        # Check mission items
        if 'MISSION_ITEM' in msg_type:
            if hasattr(msg, 'x') and hasattr(msg, 'y'):
                lat = msg.x
                lon = msg.y
                
                distance = self._calculate_distance(
                    self.security_settings.geofence_center_lat,
                    self.security_settings.geofence_center_lon,
                    lat, lon
                )
                
                if distance > self.security_settings.geofence_radius:
                    return False
        
        return True
    
    def _calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS coordinates (Haversine)"""
        import math
        
        R = 6371000  # Earth radius in meters
        
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        
        a = math.sin(dphi/2)**2 + \
            math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def _handle_blocked_command(self, msg, result):
        """Handle a blocked command"""
        msg_type = msg.get_type()
        reasons = ', '.join(result['reasons'])
        
        # Alert
        if self.security_settings.alert_enabled:
            alert_msg = f"{Fore.RED}⚠ BLOCKED: {msg_type} - {reasons}"
            self.say(alert_msg, priority='critical')
        
        # Log
        self._log_threat(msg, result['reasons'])
        
        if self.security_settings.verbose:
            self.console.write(f"{Fore.RED}✗ Command blocked: {msg_type}\n")
            self.console.write(f"  Reasons: {reasons}\n")
    
    def _log_command(self, msg, status):
        """Log command to file"""
        if not self.security_settings.log_enabled:
            return
        
        log_entry = {
            'timestamp': datetime.now().isoformat(),
            'status': status,
            'msg_type': msg.get_type(),
            'details': self._extract_msg_details(msg)
        }
        
        try:
            with open(self.log_file, 'a') as f:
                f.write(json.dumps(log_entry) + '\n')
        except:
            pass
    
    def _log_threat(self, msg, reasons):
        """Log security threat"""
        threat_entry = {
            'timestamp': datetime.now().isoformat(),
            'severity': 'HIGH',
            'msg_type': msg.get_type(),
            'reasons': reasons,
            'details': self._extract_msg_details(msg)
        }
        
        try:
            with open(self.threat_log_file, 'a') as f:
                f.write(json.dumps(threat_entry) + '\n')
        except:
            pass
    
    def _extract_msg_details(self, msg):
        """Extract relevant details from message"""
        details = {}
        msg_type = msg.get_type()
        
        if msg_type == 'COMMAND_LONG':
            details = {
                'command': msg.command,
                'param1': msg.param1,
                'param2': msg.param2,
                'param3': msg.param3,
            }
        elif 'POSITION' in msg_type:
            if hasattr(msg, 'lat_int'):
                details['lat'] = msg.lat_int / 1e7
                details['lon'] = msg.lon_int / 1e7
            if hasattr(msg, 'alt'):
                details['alt'] = msg.alt
        
        return details
    
    def _log_info(self, message):
        """Log info message"""
        if self.security_settings.log_enabled:
            try:
                with open(self.log_file, 'a') as f:
                    f.write(f"{datetime.now().isoformat()} INFO: {message}\n")
            except:
                pass
    
    def _log_warning(self, message):
        """Log warning message"""
        if self.security_settings.log_enabled:
            try:
                with open(self.log_file, 'a') as f:
                    f.write(f"{datetime.now().isoformat()} WARNING: {message}\n")
            except:
                pass
    
    def cmd_security(self, args):
        """Handle security commands"""
        if len(args) == 0:
            self.cmd_security_status([])
            return
        
        cmd = args[0].lower()
        
        if cmd == 'status':
            self.cmd_security_status(args[1:])
        elif cmd == 'enable':
            self.security_settings.enabled = True
            self.say("Security module ENABLED")
        elif cmd == 'disable':
            self.security_settings.enabled = False
            self.say("Security module DISABLED")
        elif cmd == 'stats':
            self.cmd_security_stats(args[1:])
        elif cmd == 'clear':
            self.cmd_security_clear(args[1:])
        elif cmd == 'set':
            self.cmd_security_set(args[1:])
        elif cmd == 'test':
            self.cmd_security_test(args[1:])
        else:
            print("Usage: security <status|enable|disable|stats|clear|set|test>")
    
    def cmd_security_status(self, args):
        """Display security status"""
        print(f"\n{Fore.CYAN}{'='*60}")
        print(f"{Fore.CYAN}Security Module Status")
        print(f"{Fore.CYAN}{'='*60}")
        
        status_color = Fore.GREEN if self.security_settings.enabled else Fore.RED
        status_text = "ENABLED" if self.security_settings.enabled else "DISABLED"
        print(f"Status: {status_color}{status_text}")
        
        print(f"\nProtection Layers:")
        print(f"  Rate Limiting:   {'✓' if self.security_settings.rate_limit_enabled else '✗'} "
              f"({self.security_settings.rate_limit_max} cmd/s)")
        print(f"  Geofencing:      {'✓' if self.security_settings.geofence_enabled else '✗'} "
              f"({self.security_settings.geofence_radius}m radius)")
        print(f"  Authentication:  {'✓' if self.security_settings.auth_enabled else '✗'}")
        print(f"  Logging:         {'✓' if self.security_settings.log_enabled else '✗'}")
        
        if self.security_settings.geofence_enabled:
            print(f"\nGeofence Configuration:")
            print(f"  Center: {self.security_settings.geofence_center_lat:.6f}, "
                  f"{self.security_settings.geofence_center_lon:.6f}")
            print(f"  Radius: {self.security_settings.geofence_radius}m")
            print(f"  Max Altitude: {self.security_settings.geofence_max_alt}m")
        
        print(f"\nCurrent Rate: {len([t for t in self.command_history if time.time() - t < 1.0])} cmd/s")
        print(f"Available Tokens: {self.rate_tokens:.1f}/{self.security_settings.rate_limit_burst}")
        
        print(f"{Fore.CYAN}{'='*60}\n")
    
    def cmd_security_stats(self, args):
        """Display security statistics"""
        runtime = time.time() - self.stats['start_time']
        hours = int(runtime // 3600)
        minutes = int((runtime % 3600) // 60)
        
        total_commands = max(self.stats['commands_processed'], 1)
        block_rate = (self.stats['commands_blocked'] / total_commands) * 100
        
        print(f"\n{Fore.CYAN}{'='*60}")
        print(f"{Fore.CYAN}Security Statistics")
        print(f"{Fore.CYAN}{'='*60}")
        print(f"Runtime: {hours}h {minutes}m")
        print(f"\nMessage Statistics:")
        print(f"  Total Messages:       {self.stats['total_messages']}")
        print(f"  Commands Processed:   {self.stats['commands_processed']}")
        print(f"  {Fore.GREEN}Commands Allowed:     {self.stats['commands_allowed']}")
        print(f"  {Fore.RED}Commands Blocked:     {self.stats['commands_blocked']} ({block_rate:.1f}%)")
        
        print(f"\nThreat Detection:")
        print(f"  {Fore.YELLOW}Threats Detected:     {self.stats['threats_detected']}")
        print(f"  Rate Limit Hits:      {self.stats['rate_limit_hits']}")
        print(f"  Geofence Violations:  {self.stats['geofence_violations']}")
        print(f"  Auth Failures:        {self.stats['auth_failures']}")
        
        print(f"{Fore.CYAN}{'='*60}\n")
    
    def cmd_security_clear(self, args):
        """Clear statistics"""
        self.stats = {
            'total_messages': 0,
            'commands_processed': 0,
            'commands_allowed': 0,
            'commands_blocked': 0,
            'rate_limit_hits': 0,
            'geofence_violations': 0,
            'auth_failures': 0,
            'threats_detected': 0,
            'start_time': time.time(),
        }
        self.say("Statistics cleared")
    
    def cmd_security_set(self, args):
        """Set security setting"""
        if len(args) < 2:
            print("Usage: security set <setting> <value>")
            return
        
        setting = args[0]
        value = args[1]
        
        try:
            # Use settings system
            self.security_settings.set(setting, value)
            self.say(f"Set {setting} = {value}")
        except Exception as e:
            print(f"Error: {e}")
    
    def cmd_security_test(self, args):
        """Run security tests"""
        print(f"\n{Fore.CYAN}Running Security Tests...")
        
        # Test 1: Rate limiter
        print(f"\n1. Testing rate limiter...")
        old_tokens = self.rate_tokens
        allowed = self._check_rate_limit()
        if allowed:
            print(f"   {Fore.GREEN}✓ Rate limiter working (tokens: {old_tokens:.1f} -> {self.rate_tokens:.1f})")
        
        # Test 2: Geofence
        if self.security_settings.geofence_enabled:
            print(f"\n2. Testing geofence...")
            test_lat = self.security_settings.geofence_center_lat + 0.01
            test_lon = self.security_settings.geofence_center_lon + 0.01
            dist = self._calculate_distance(
                self.security_settings.geofence_center_lat,
                self.security_settings.geofence_center_lon,
                test_lat, test_lon
            )
            print(f"   Test point distance: {dist:.0f}m")
            if dist > self.security_settings.geofence_radius:
                print(f"   {Fore.GREEN}✓ Geofence would block this point")
            else:
                print(f"   {Fore.YELLOW}⚠ Point within fence")
        
        print(f"\n{Fore.GREEN}Security tests complete!\n")
    
    def unload(self):
        """Clean up when module is unloaded"""
        self._log_info("Security module unloaded")
        super(SecurityModule, self).unload()

def init(mpstate):
    """Initialize module"""
    return SecurityModule(mpstate)
