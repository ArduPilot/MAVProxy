'''log download: LOG_DATA must stay inside the log being downloaded'''

import os
from types import SimpleNamespace

import pytest
from pymavlink import mavutil

from MAVProxy.modules import mavproxy_log

LOG_DATA_LEN = 90


class Sent(object):
    '''what the module sends to the vehicle, decoded again'''

    def __init__(self):
        self.parser = mavutil.mavlink.MAVLink(None)
        self.messages = []

    def write(self, buf):
        self.messages.extend(self.parser.parse_buffer(bytes(buf)) or [])

    def of_type(self, name):
        return [m for m in self.messages if m.get_type() == name]


class Vehicle(object):
    '''packs messages as a vehicle would and parses them as MAVProxy does,
    so the module sees what comes off the wire'''

    def __init__(self, contents):
        self.contents = contents
        self.mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        self.parser = mavutil.mavlink.MAVLink(None)

    def wire(self, msg):
        return self.parser.parse_buffer(msg.pack(self.mav))[0]

    def log_entry(self, log_id, size):
        return self.wire(mavutil.mavlink.MAVLink_log_entry_message(
            log_id, 1, log_id, 0, size))

    def log_data(self, log_id, ofs, count=None, data=None):
        if data is None:
            data = self.contents[ofs:ofs+LOG_DATA_LEN]
        if count is None:
            count = len(data)
        data = list(bytearray(data).ljust(LOG_DATA_LEN, b'\0'))
        return self.wire(mavutil.mavlink.MAVLink_log_data_message(
            log_id, ofs, count, data))


@pytest.fixture
def log(tmp_path):
    sent = Sent()
    mpstate = SimpleNamespace(
        command_map={},
        completions={},
        settings=SimpleNamespace(target_system=1, target_component=1),
        console=SimpleNamespace(set_status=lambda *args, **kwargs: None),
        master=lambda: SimpleNamespace(mav=mavutil.mavlink.MAVLink(sent)),
    )
    module = mavproxy_log.init(mpstate)
    module.sent = sent
    module.path = str(tmp_path / 'log1.bin')
    return module


def contents(size):
    return bytes(i % 251 for i in range(size))


def start(log, vehicle, size=None):
    if size is not None:
        log.mavlink_packet(vehicle.log_entry(1, size))
    log.cmd_log(['download', '1', log.path])


def finished(log):
    return log.download_file is None and len(log.sent.of_type('LOG_REQUEST_END')) == 1


def read(path):
    with open(path, 'rb') as f:
        return f.read()


def test_huge_offset_does_not_grow_file(log):
    vehicle = Vehicle(contents(200))
    start(log, vehicle, 200)
    log.mavlink_packet(vehicle.log_data(1, 20000000, data=b'x' * LOG_DATA_LEN))
    log.download_file.flush()
    assert os.path.getsize(log.path) == 0
    assert log.download_set == set()
    # the download carries on as if the packet had never come
    for ofs in range(0, 200, LOG_DATA_LEN):
        log.mavlink_packet(vehicle.log_data(1, ofs))
    assert finished(log)
    assert read(log.path) == vehicle.contents


def test_data_for_another_log_is_ignored(log):
    vehicle = Vehicle(contents(200))
    start(log, vehicle, 200)
    log.mavlink_packet(vehicle.log_entry(2, 50000000))
    log.mavlink_packet(vehicle.log_data(2, 40000000, data=b'x' * LOG_DATA_LEN))
    log.mavlink_packet(vehicle.log_data(2, 0, data=b'y' * LOG_DATA_LEN))
    log.download_file.flush()
    assert os.path.getsize(log.path) == 0
    assert log.download_set == set()
    for ofs in range(0, 200, LOG_DATA_LEN):
        log.mavlink_packet(vehicle.log_data(1, ofs))
    assert finished(log)
    assert read(log.path) == vehicle.contents


def test_missing_data_requests_stay_inside_the_log(log):
    vehicle = Vehicle(contents(900))
    start(log, vehicle, 900)
    log.mavlink_packet(vehicle.log_data(1, 0))
    log.mavlink_packet(vehicle.log_data(1, 0xFFFFFF00, data=b'x' * LOG_DATA_LEN))
    log.mavlink_packet(vehicle.log_data(1, 900, data=b'x' * LOG_DATA_LEN))
    log.handle_log_data_missing()
    requests = log.sent.of_type('LOG_REQUEST_DATA')[1:]
    assert [(r.ofs, r.count) for r in requests] == [(LOG_DATA_LEN, 0xFFFFFFFF)]


def test_log_that_grew_since_listing_stops_at_listed_size(log):
    vehicle = Vehicle(contents(200))
    start(log, vehicle, 100)
    for ofs in range(0, 200, LOG_DATA_LEN):
        log.mavlink_packet(vehicle.log_data(1, ofs))
    assert finished(log)
    assert read(log.path) == vehicle.contents[:100]


@pytest.mark.parametrize('grown', [False, True])
def test_end_of_log_reply_finishes_download(log, grown):
    '''a log a whole number of packets long ends with a reply at its size:
    empty, or more data if the log has grown since it was listed'''
    vehicle = Vehicle(contents(270 if grown else 180))
    start(log, vehicle, 180)
    for ofs in (0, 90):
        log.mavlink_packet(vehicle.log_data(1, ofs))
    assert not finished(log)
    log.handle_log_data_missing()
    request = log.sent.of_type('LOG_REQUEST_DATA')[-1]
    assert request.ofs == 180
    log.mavlink_packet(vehicle.log_data(1, request.ofs))
    assert finished(log)
    assert read(log.path) == vehicle.contents[:180]


def test_end_of_log_reply_waits_for_missing_data(log):
    '''the short last packet came before a lost one was sent again'''
    vehicle = Vehicle(contents(200))
    start(log, vehicle, 200)
    log.mavlink_packet(vehicle.log_data(1, 0))
    log.mavlink_packet(vehicle.log_data(1, 180))
    assert not finished(log)
    # nothing past the end finishes the download while a packet is missing
    log.mavlink_packet(vehicle.log_data(1, 270))
    assert not finished(log)
    log.handle_log_data_missing()
    request = log.sent.of_type('LOG_REQUEST_DATA')[-1]
    assert (request.ofs, request.count) == (90, 90)
    log.mavlink_packet(vehicle.log_data(1, 90))
    assert not finished(log)
    log.handle_log_data_missing()
    request = log.sent.of_type('LOG_REQUEST_DATA')[-1]
    assert request.ofs == 270
    log.mavlink_packet(vehicle.log_data(1, request.ofs))
    assert finished(log)
    assert read(log.path) == vehicle.contents


def test_download_without_log_list(log):
    '''with no LOG_ENTRY there is no size to check against'''
    vehicle = Vehicle(contents(200))
    start(log, vehicle)
    for ofs in range(0, 200, LOG_DATA_LEN):
        log.mavlink_packet(vehicle.log_data(1, ofs))
    assert finished(log)
    assert read(log.path) == vehicle.contents
