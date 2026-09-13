"""MAVLink camera definition metadata and binary extended parameter values."""

import locale
import lzma
import math
import operator
import re
import struct
from urllib.parse import urlsplit
from urllib.request import build_opener, HTTPRedirectHandler

from defusedxml import ElementTree as ET
from defusedxml.common import DefusedXmlException

# MAV_PARAM_EXT_TYPE, little-endian wire representation.
TYPES = {name: (index + 1, fmt) for index, (name, fmt) in enumerate([
    ('uint8', 'B'), ('int8', 'b'), ('uint16', 'H'), ('int16', 'h'),
    ('uint32', 'I'), ('int32', 'i'), ('uint64', 'Q'), ('int64', 'q'),
    ('float', 'f'), ('double', 'd')])}
TYPES['bool'] = TYPES['uint8']
TYPES['custom'] = (11, None)
MAX_DEFINITION_SIZE = 4 * 1024 * 1024


def decode_value(message):
    """Do not use pymavlink's decoded char[] string: it discards binary data."""
    raw = getattr(message, '_param_value_raw', None)
    if raw is None:
        raw = message.param_value
    if not isinstance(raw, (bytes, bytearray)):
        raise ValueError('extended parameter has no raw binary value; upgrade pymavlink to 2.4.38 or later')
    param_type = message.param_type
    if param_type == 11:
        return bytes(raw)
    formats = {v[0]: v[1] for v in TYPES.values()}
    if param_type not in formats:
        raise ValueError('unknown extended parameter type %s' % param_type)
    fmt = '<' + formats[param_type]
    value = struct.unpack(fmt, bytes(raw).ljust(128, b'\0')[:struct.calcsize(fmt)])[0]
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError('camera returned a non-finite parameter value')
    return value


def equal_value(a, b):
    # Both incoming values and XML options are canonical wire values. Even
    # adjacent float32 numbers may name different options or exclusion rules.
    return a == b


def _ulp(value, single=False):
    """Spacing of binary32/binary64 values, including subnormals."""
    if value == 0:
        return 2.0 ** (-149 if single else -1074)
    exponent = math.frexp(value)[1]
    return 2.0 ** max(exponent - (24 if single else 53),
                     -149 if single else -1074)


def definition_bytes(data):
    if len(data) > MAX_DEFINITION_SIZE:
        raise ValueError('camera definition exceeds size limit')
    if data.startswith(b'\xfd7zXZ\x00'):
        decoder = lzma.LZMADecompressor(memlimit=64 * 1024 * 1024)
        data = decoder.decompress(data, max_length=MAX_DEFINITION_SIZE + 1)
        if len(data) > MAX_DEFINITION_SIZE or not decoder.eof:
            raise ValueError('compressed camera definition exceeds limit or is incomplete')
    return data


class _DefinitionRedirectHandler(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        if urlsplit(newurl).scheme.lower() not in ('http', 'https'):
            raise ValueError('camera definition redirects must use HTTP or HTTPS')
        return super().redirect_request(req, fp, code, msg, headers, newurl)


def download_definition(uri):
    """Called in a worker thread, never on MAVProxy's packet-processing loop."""
    if urlsplit(uri).scheme.lower() not in ('http', 'https'):
        raise ValueError('camera definition downloads must use HTTP or HTTPS')
    with build_opener(_DefinitionRedirectHandler()).open(uri, timeout=15) as response:
        return definition_bytes(response.read(MAX_DEFINITION_SIZE + 1))


class Parameter:
    def __init__(self, element, translate):
        self.name = element.attrib['name']
        if not self.name or len(self.name.encode('ascii')) > 16:
            raise ValueError('invalid parameter name %s' % self.name)
        self.type = element.attrib['type'].lower()
        self.wire_type, self.fmt = TYPES[self.type]
        self.description = translate(element.findtext('description', self.name).strip())
        self.control = element.get('control', '1').lower() not in ('0', 'false')
        self.readonly = element.get('readonly', '0').lower() in ('1', 'true')
        self.writeonly = element.get('writeonly', '0').lower() in ('1', 'true')
        if self.type == 'custom':
            self.control = False
        self.default = None
        self.minimum = None
        self.maximum = None
        self.step = None
        self.updates = [n.text.strip() for n in element.findall('updates/update') if n.text]
        self.options = []
        self.exclusions = []
        self.ranges = []
        # Custom data has no numeric editor. Vendor-specific attributes must
        # not prevent unrelated numeric settings from loading.
        if self.fmt is None:
            return
        self.default = self.convert(element.get('default', '0'))
        for attribute, field in (('min', 'minimum'), ('max', 'maximum'), ('step', 'step')):
            if attribute in element.attrib:
                raw = element.get(attribute)
                converted = self.convert(raw)
                # Keep XML bounds/steps at double precision for UI labels and
                # grid calculations; only values/options use wire precision.
                setattr(self, field, float(raw) if self.type in ('float', 'double') else converted)
        if self.minimum is not None and self.maximum is not None and self.minimum > self.maximum:
            raise ValueError('minimum exceeds maximum')
        if self.step is not None and self.step <= 0:
            raise ValueError('step must be positive')
        for option in element.findall('options/option'):
            value = self.convert(option.attrib['value'])
            self.options.append((translate(option.attrib['name']), value))
            # Some Workswell files contain nested <exclusions> wrappers.
            self.exclusions.append((value, [n.text.strip() for n in
                                           option.findall('.//exclude') if n.text]))
            for limit in option.findall('parameterranges/parameterrange'):
                self.ranges.append((value, limit.attrib['parameter'],
                                    limit.get('condition', ''),
                                    [(translate(n.attrib['name']), n.attrib['value'])
                                     for n in limit.findall('roption')]))
        if self.type == 'bool' and not self.options:
            self.options = [(translate('Off'), 0), (translate('On'), 1)]

    def convert(self, value):
        if self.fmt is None:
            raise ValueError('custom parameters do not have numeric values')
        if self.type in ('float', 'double'):
            result = float(value)
            if not math.isfinite(result):
                raise ValueError('value must be finite')
        else:
            result = int(value)
            if isinstance(value, float) and value != result:
                raise ValueError('value must be an integer')
            if self.type == 'bool' and result not in (0, 1):
                raise ValueError('boolean must be 0 or 1')
        try:
            # Also canonicalise float options to their exact wire value.
            return struct.unpack('<' + self.fmt, struct.pack('<' + self.fmt, result))[0]
        except (struct.error, OverflowError) as error:
            raise ValueError('value outside %s range' % self.type) from error

    def validate(self, value, options):
        value = self.convert(value)
        if self.minimum is not None and value < self.convert(self.minimum):
            raise ValueError('minimum is %s' % self.minimum)
        if self.maximum is not None and value > self.convert(self.maximum):
            raise ValueError('maximum is %s' % self.maximum)
        if options and not any(equal_value(value, v) for _, v in options):
            raise ValueError('value is not an available option')
        if self.step and self.minimum is not None:
            if self.type not in ('float', 'double'):
                on_grid = (value - self.minimum) % self.step == 0
            else:
                steps = (value - self.minimum) / self.step
                if not math.isfinite(steps):
                    raise ValueError('step count exceeds numeric range')
                offset = round(steps) * self.step
                grid_value = self.minimum + offset
                # Allow wire rounding and double-precision grid arithmetic,
                # not a tolerance which grows with the number of steps.
                tolerance = (0.5 * _ulp(value, self.type == 'float') +
                             2 * (_ulp(self.minimum) + _ulp(offset)))
                on_grid = abs(value - grid_value) <= tolerance
            if not on_grid:
                raise ValueError('value must use steps of %s from %s' % (self.step, self.minimum))
        return value

    def encode(self, value):
        return struct.pack('<' + self.fmt, self.convert(value)).ljust(128, b'\0')


class CameraDefinition:
    def __init__(self, data, locale_name=None):
        try:
            root = ET.fromstring(definition_bytes(data), forbid_dtd=True)
        except (ET.ParseError, DefusedXmlException) as error:
            raise ValueError('invalid camera definition XML: %s' % error) from error
        if root.tag != 'mavlinkcamera' or root.find('definition') is None:
            raise ValueError('not a MAVLink camera definition')
        self.vendor = root.findtext('definition/vendor', '')
        self.model = root.findtext('definition/model', '')
        self.version = root.find('definition').get('version', '')
        self.parameters = {}
        translations = {}
        language = (locale_name or locale.getlocale()[0] or 'en_US').lower().replace('-', '_')
        locales = root.findall('localization/locale')
        matched = [n for n in locales if n.get('name', '').lower().replace('-', '_') == language]
        if not matched:
            matched = [n for n in locales if n.get('name', '').lower().split('_')[0] == language.split('_')[0]]
        if matched:
            translations = {n.get('original'): n.get('translated') for n in matched[0].findall('strings')}
        for element in root.findall('parameters/parameter'):
            try:
                param = Parameter(element, lambda s: translations.get(s, s))
            except (KeyError, TypeError, ValueError, UnicodeError) as error:
                raise ValueError('invalid camera parameter %s: %s' % (element.get('name', '?'), error)) from error
            if param.name in self.parameters:
                raise ValueError('duplicate camera parameter %s' % param.name)
            self.parameters[param.name] = param
        for param in self.parameters.values():
            for _, target, _, options in param.ranges:
                if target in self.parameters:
                    target_param = self.parameters[target]
                    if target_param.fmt is None:
                        continue
                    options[:] = [(label, target_param.convert(value)) for label, value in options]

    def condition(self, condition, values):
        """QGC conditions are comparisons joined left-to-right by AND/OR."""
        if not condition.strip():
            return True
        parts = re.split(r'\s+(AND|OR)\s+', condition.strip(), flags=re.I)
        result = None
        operations = {'=': equal_value, '==': equal_value, '!=': lambda a, b: not equal_value(a, b),
                      '>': operator.gt, '<': operator.lt, '>=': operator.ge, '<=': operator.le}
        for index in range(0, len(parts), 2):
            match = re.fullmatch(r'\s*([A-Za-z0-9_]+)\s*(!=|==|>=|<=|=|>|<)\s*(\S+)\s*', parts[index])
            if not match:
                return False
            name, op, value = match.groups()
            if name not in values or name not in self.parameters:
                return False
            try:
                test = operations[op](values[name], self.parameters[name].convert(value))
            except (TypeError, ValueError):
                return False
            if result is None:
                result = test
            elif parts[index - 1].upper() == 'AND':
                result = result and test
            else:
                result = result or test
        return result

    def controls(self, values):
        excluded = set()
        options = {p.name: p.options for p in self.parameters.values()}
        limited = set()
        for param in self.parameters.values():
            if param.name not in values:
                continue
            current = values[param.name]
            for value, names in param.exclusions:
                if equal_value(current, value):
                    excluded.update(names)
            for value, target, condition, choices in param.ranges:
                if (target not in limited and equal_value(current, value) and
                        self.condition(condition, values)):
                    options[target] = choices
                    limited.add(target)
        return {p.name: options[p.name] for p in self.parameters.values()
                if p.control and p.name not in excluded}
