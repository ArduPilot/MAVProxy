from paho.mqtt import MQTTException
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_module
import paho.mqtt.client as mqtt
import json
import numbers
import sys

class MqttModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(MqttModule, self).__init__(mpstate, "mqtt", "mqtt publisher")
        self.connected = False
        self.client = mqtt.Client()
        self.mqtt_settings = mp_settings.MPSettings(
            [('ip', str, '127.0.0.1'),
             ('port', int, '1883'),
             ('name', str, 'mavproxy'),
             ('prefix', str, ''),
             ('username', str, ''),
             ('password', str, '')
             ])
        self.add_command('mqtt', self.mqtt_command, "mqtt module", ['connect', 'set (MQTTSETTING)', 'disconnect'])
        self.add_completion_function('(MQTTSETTING)', self.mqtt_settings.completion)

    def mavlink_packet(self, m):
        """handle an incoming mavlink packet"""
        if not self.connected:
            return
        try:
            data = self.convert_to_dict(m)
            self.client.publish(f'{self.mqtt_settings.prefix}/{m.get_type()}', json.dumps(data))
        except MQTTException as e:
            self.connected = False
            print(f'mqtt: Exception occurred: {e}')

    def connect(self):
        """connect to mqtt broker"""
        try:
            if self.mqtt_settings.username != '':
                if self.mqtt_settings.password != '':
                    self.client.username_pw_set(self.mqtt_settings.username, password=self.mqtt_settings.password)
                else:
                    self.client.username_pw_set(self.mqtt_settings.username, password=None)
            self.client.reinitialise(client_id=self.mqtt_settings.name)
            print(f'connecting to {self.mqtt_settings.ip}:{self.mqtt_settings.port}')
            self.client.connect(self.mqtt_settings.ip, int(self.mqtt_settings.port), 30)
        except MQTTException as e:
            print(f'mqtt: could not establish connection: {e}')
            return
        self.connected = True
        print('mqtt: connected...')

    def disconnect(self):
        """disconnect"""
        self.client.disconnect()
        self.connected = False
        print('mqtt: disconnected')

    def mqtt_command(self, args):
        """control behaviour of the module"""
        if len(args) == 0:
            print(self.usage())
        elif args[0] == 'set':
            self.mqtt_settings.command(args[1:])
        elif args[0] == 'connect':
            self.connect()
        elif args[0] == 'disconnect':
            self.disconnect()

    def usage(self):
        """show help on command line options"""
        return "Usage: mqtt <set|connect|disconnect>"

    def convert_to_dict(self, message):
        """converts mavlink message to python dict"""
        result = {}
        for field in message._fieldnames:
            value = getattr(message, field)
            if isinstance(value, numbers.Number):
                result[field] = value
            else:
                result[field] = str(value)
        return result


def init(mpstate):
    """initialise module"""
    return MqttModule(mpstate)
