"""NAV_CONTROLLER_PROGRESS in the console's status, without starting a console window."""
from types import SimpleNamespace
from unittest import mock

import pytest
from pymavlink import mavutil

from MAVProxy.modules.mavproxy_console import ConsoleModule
from MAVProxy.modules.mavproxy_link import delayedPackets
from MAVProxy.modules.lib import wxconsole

NAN = float('nan')


@pytest.fixture
def console():
    module = ConsoleModule.__new__(ConsoleModule)
    master = mock.Mock()
    module.mpstate = SimpleNamespace(
        console=mock.Mock(spec=wxconsole.MessageConsole),
        settings=SimpleNamespace(dist_unit='m'),
        master=lambda: master,
    )
    module.module = mock.Mock(return_value=mock.Mock(new_sysid_timestamp=0))
    module.last_param_sysid_timestamp = 0
    module.message_is_from_primary_vehicle = mock.Mock(return_value=True)
    module.probe_for_flight_information = mock.Mock()
    module.user_added = {}
    module.last_nav_controller_progress = None
    module.last_unload_check_time = 0
    module.unload_check_interval = 1e9
    module.nav_progress_requests = {}
    module.message_interval_requests = {}
    module.flight_information = {}
    return module


def progress(seq=3, percentage_complete=255, distance_remaining=NAN, time_remaining=NAN,
             count_remaining=NAN, mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION):
    return SimpleNamespace(
        get_type=lambda: 'NAV_CONTROLLER_PROGRESS', get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1, seq=seq, percentage_complete=percentage_complete,
        distance_remaining=distance_remaining, time_remaining=time_remaining,
        count_remaining=count_remaining, mission_type=mission_type)


def shown(console):
    return console.console.set_status.call_args.args


@pytest.mark.parametrize('msg,want', [
    (progress(distance_remaining=42.7), 'Progress 42m'),
    (progress(time_remaining=4, percentage_complete=20), 'Progress 0:04 (20%)'),
    (progress(time_remaining=75), 'Progress 1:15'),
    (progress(count_remaining=0.6, percentage_complete=40), 'Progress count 0.6 (40%)'),
    (progress(), 'Progress --'),
    (progress(seq=0, distance_remaining=10, percentage_complete=80,
              mission_type=mavutil.mavlink.MAV_MISSION_TYPE_ALL), 'Progress 10m (80%)'),
    (progress(distance_remaining=10, mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE), 'Progress --'),
])
def test_progress_shown(console, msg, want):
    console.mavlink_packet(msg)
    assert shown(console) == ('WPProgress', want)


def test_stale_progress_cleared(console):
    with mock.patch('time.time', return_value=100):
        console.mavlink_packet(progress(distance_remaining=10))
    with mock.patch('time.time', return_value=102):
        console.idle_task()
    assert shown(console) == ('WPProgress', 'Progress 10m')
    with mock.patch('time.time', return_value=104):
        console.idle_task()
    assert shown(console) == ('WPProgress', 'Progress --')
    console.console.set_status.reset_mock()
    with mock.patch('time.time', return_value=110):
        console.idle_task()
    console.console.set_status.assert_not_called()


@pytest.fixture
def development_dialect(monkeypatch):
    # NAV_CONTROLLER_PROGRESS is in development.xml, which the default
    # ardupilotmega dialect does not include; MAVProxy uses "all"
    monkeypatch.setattr(mavutil.mavlink, 'MAVLINK_MSG_ID_NAV_CONTROLLER_PROGRESS', 405, raising=False)


def vehicle_message(kind, sysid=1, compid=1, **fields):
    return SimpleNamespace(get_type=lambda: kind, get_srcSystem=lambda: sysid,
                           get_srcComponent=lambda: compid, **fields)


def requests(console):
    return [c.args for c in console.master.mav.command_long_send.call_args_list
            if c.args[2] == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and
            c.args[4] == mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_PROGRESS]


def heartbeat():
    return vehicle_message('HEARTBEAT', autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)


def ack(result):
    return vehicle_message('COMMAND_ACK', command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, result=result)


def at(t, fn, *args):
    with mock.patch('time.time', return_value=t):
        fn(*args)


def test_progress_requested_until_answered(console, development_dialect):
    at(100, console.request_nav_controller_progress, heartbeat())
    at(101, console.request_nav_controller_progress, heartbeat())
    assert len(requests(console)) == 1
    assert requests(console)[0][:2] == (1, 1)
    # the request or its ACK was lost: ask again
    at(111, console.request_nav_controller_progress, heartbeat())
    assert len(requests(console)) == 2
    at(112, console.handle_command_ack, ack(mavutil.mavlink.MAV_RESULT_ACCEPTED))
    at(130, console.request_nav_controller_progress, heartbeat())
    assert len(requests(console)) == 2


def test_progress_message_answers_the_request(console, development_dialect):
    at(100, console.request_nav_controller_progress, heartbeat())
    at(101, console.request_nav_controller_progress, progress(distance_remaining=10))
    at(130, console.request_nav_controller_progress, heartbeat())
    assert len(requests(console)) == 1


def test_progress_requested_after_reboot(console, development_dialect):
    at(100, console.request_nav_controller_progress, heartbeat())
    at(101, console.handle_command_ack, ack(mavutil.mavlink.MAV_RESULT_ACCEPTED))
    for (t, boot_ms) in (102, 50000), (103, 51000), (104, 48000), (115, 52000):
        # 48000 is a sample from a slower link, not a reboot
        at(t, console.request_nav_controller_progress, vehicle_message('SYSTEM_TIME', time_boot_ms=boot_ms))
    assert len(requests(console)) == 1
    # boot time well behind: the vehicle rebooted and forgot the request
    at(116, console.request_nav_controller_progress, vehicle_message('SYSTEM_TIME', time_boot_ms=3000))
    assert len(requests(console)) == 2


def test_progress_requested_from_vehicles_not_shown(console, development_dialect):
    console.message_is_from_primary_vehicle = mock.Mock(return_value=False)
    console.console.is_alive = mock.Mock(return_value=True)
    console.handle_heartbeat_anysource = mock.Mock()
    other = vehicle_message('HEARTBEAT', sysid=2, autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)
    at(100, console.mavlink_packet, other)
    assert [r[:2] for r in requests(console)] == [(2, 1)]


def test_flight_information_probe_not_confused_by_progress_ack(console, development_dialect, monkeypatch):
    monkeypatch.setattr(mavutil.mavlink, 'MAVLINK_MSG_ID_FLIGHT_INFORMATION', 264, raising=False)
    console.probe_for_flight_information = ConsoleModule.probe_for_flight_information.__get__(console)
    at(100, console.probe_for_flight_information, 1, 1)
    # only one request at a time, as an ACK does not say which message it is for
    at(100, console.request_nav_controller_progress, heartbeat())
    assert requests(console) == []
    at(101, console.handle_command_ack, ack(mavutil.mavlink.MAV_RESULT_ACCEPTED))
    assert console.flight_information[1].supported is True
    at(101, console.request_nav_controller_progress, heartbeat())
    assert len(requests(console)) == 1
    # a vehicle without the message denies it
    at(102, console.handle_command_ack, ack(mavutil.mavlink.MAV_RESULT_DENIED))
    assert console.flight_information[1].supported is True
    at(130, console.request_nav_controller_progress, heartbeat())
    assert len(requests(console)) == 1


def test_progress_delayed_link_dropped():
    assert 'NAV_CONTROLLER_PROGRESS' in delayedPackets


def test_infinite_time_remaining_not_shown(console):
    console.mavlink_packet(progress(time_remaining=float('inf'), percentage_complete=10))
    assert shown(console) == ('WPProgress', 'Progress (10%)')


def test_progress_not_requested_from_non_autopilots(console, development_dialect):
    console.request_nav_controller_progress(vehicle_message('HEARTBEAT', autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID))
    console.request_nav_controller_progress(vehicle_message('SYSTEM_TIME', time_boot_ms=100))
    assert requests(console) == []
