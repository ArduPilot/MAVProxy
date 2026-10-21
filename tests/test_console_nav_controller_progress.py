"""NAV_CONTROLLER_PROGRESS in the console's status, without starting a console window."""
from types import SimpleNamespace
from unittest import mock

import pytest
from pymavlink import mavutil

from MAVProxy.modules.mavproxy_console import ConsoleModule
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
    module.nav_progress_boot_ms = {}
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


def test_progress_requested_once_and_after_reboot(console, development_dialect):
    heartbeat = vehicle_message('HEARTBEAT', autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)
    console.request_nav_controller_progress(heartbeat)
    console.request_nav_controller_progress(heartbeat)
    assert len(requests(console)) == 1
    assert requests(console)[0][:2] == (1, 1)
    console.request_nav_controller_progress(vehicle_message('SYSTEM_TIME', time_boot_ms=5000))
    console.request_nav_controller_progress(vehicle_message('SYSTEM_TIME', time_boot_ms=6000))
    assert len(requests(console)) == 1
    # boot time going backwards: the vehicle rebooted and forgot the request
    console.request_nav_controller_progress(vehicle_message('SYSTEM_TIME', time_boot_ms=100))
    assert len(requests(console)) == 2


def test_progress_not_requested_from_non_autopilots(console, development_dialect):
    console.request_nav_controller_progress(vehicle_message('HEARTBEAT', autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID))
    console.request_nav_controller_progress(vehicle_message('SYSTEM_TIME', time_boot_ms=100))
    assert requests(console) == []
