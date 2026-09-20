"""Camera/gimbal names in the Vehicle menu, without starting a console window."""
from types import SimpleNamespace
from unittest import mock

import pytest
from pymavlink import mavutil

from MAVProxy.modules.mavproxy_console import ConsoleModule
from MAVProxy.modules.lib import wxconsole


@pytest.fixture
def console():
    module = ConsoleModule.__new__(ConsoleModule)
    module.mpstate = SimpleNamespace(console=mock.Mock(spec=wxconsole.MessageConsole))
    module.vehicle_list = [1]
    module.vehicle_name_by_sysid = {1: 'Plane'}
    module.vehicle_heartbeats = {}
    module.component_name = {}
    module.component_model = {}
    module.vehicle_menu = SimpleNamespace(items=[])
    module.menu = mock.Mock()
    param = mock.Mock(new_sysid_timestamp=0)
    param.get_component_id_list.return_value = [1, 100, 154]
    module.module = mock.Mock(return_value=param)
    module.last_param_sysid_timestamp = 0
    module.message_is_from_primary_vehicle = mock.Mock(return_value=False)
    return module


def message(kind, component=100, **fields):
    return SimpleNamespace(get_type=lambda: kind, get_srcSystem=lambda: 1,
                           get_srcComponent=lambda: component, **fields)


@pytest.mark.parametrize('information_first', [False, True])
@pytest.mark.parametrize('kind,component,mav_type', [
    ('CAMERA_INFORMATION', 100, mavutil.mavlink.MAV_TYPE_CAMERA),
    ('GIMBAL_DEVICE_INFORMATION', 154, mavutil.mavlink.MAV_TYPE_GIMBAL)])
@pytest.mark.parametrize('encode', [str, lambda s: s.encode(), lambda s: list(s.encode())])
def test_information_names_replace_heartbeat_labels(console, information_first, kind, component, mav_type, encode):
    heartbeat = message('HEARTBEAT', component, type=mav_type)
    info = message(kind, component, vendor_name=encode('ArduPilot\0ignored'),
                   model_name=encode('SIYI A8 mini\0'))
    for msg in ([info, heartbeat] if information_first else [heartbeat, info]):
        console.mavlink_packet(msg)
    label = 'ArduPilot-SIYI A8 mini (%s)' % console.component_type_string(heartbeat)
    assert console.component_name[1][component] == label
    assert 'SysID 1[%u]: %s' % (component, label) in [
        item.name for item in console.vehicle_menu.items]
    console.console.set_menu.reset_mock()
    console.mavlink_packet(heartbeat)
    console.mavlink_packet(info)
    console.console.set_menu.assert_not_called()


def test_blank_information_preserves_camera_fallback(console):
    console.mavlink_packet(message('HEARTBEAT', type=mavutil.mavlink.MAV_TYPE_CAMERA))
    console.mavlink_packet(message('CAMERA_INFORMATION', vendor_name=[0] * 32, model_name=[0] * 32))
    assert console.component_name[1][100] == 'Camera'


def test_camera_name_updates_without_renaming_other_components(console):
    console.mavlink_packet(message('CAMERA_INFORMATION', vendor_name=b'ArduPilot', model_name=b'Old'))
    console.mavlink_packet(message('CAMERA_INFORMATION', 101, vendor_name=b'Workswell', model_name=b'WIRIS'))
    console.mavlink_packet(message('CAMERA_INFORMATION', vendor_name=b'ArduPilot', model_name=b'SIYI A8 mini'))
    assert console.component_name[1] == {100: 'ArduPilot-SIYI A8 mini', 101: 'Workswell-WIRIS'}
    assert console.vehicle_name_by_sysid[1] == 'Plane'


def test_single_component_menu_uses_camera_name(console):
    console.module.return_value.get_component_id_list.return_value = [100]
    console.vehicle_name_by_sysid[1] = 'Camera'
    console.mavlink_packet(message('CAMERA_INFORMATION', vendor_name=b'ArduPilot', model_name=b'SIYI A8 mini'))
    console.mavlink_packet(message('HEARTBEAT', type=mavutil.mavlink.MAV_TYPE_CAMERA))
    assert [item.name for item in console.vehicle_menu.items] == ['SysID 1: ArduPilot-SIYI A8 mini (Camera)']


def test_type_suffix_comes_from_heartbeat_not_information_message(console):
    console.mavlink_packet(message('CAMERA_INFORMATION', vendor_name=b'ArduPilot', model_name=b'SIYI A8 mini'))
    console.mavlink_packet(message('HEARTBEAT', type=mavutil.mavlink.MAV_TYPE_GIMBAL))
    assert console.component_name[1][100] == 'ArduPilot-SIYI A8 mini (Gimbal)'
    console.mavlink_packet(message('HEARTBEAT', type=mavutil.mavlink.MAV_TYPE_CAMERA))
    assert console.component_name[1][100] == 'ArduPilot-SIYI A8 mini (Camera)'


def test_autopilot_relayed_camera_information_keeps_vehicle_name(console):
    # ArduPilot sends CAMERA_INFORMATION for CAMn_TYPE=Mount cameras from component 1
    console.mavlink_packet(message('HEARTBEAT', 1, type=mavutil.mavlink.MAV_TYPE_FIXED_WING,
                                   autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA))
    assert console.component_name[1][1] == 'Plane'
    with mock.patch.object(console, 'update_vehicle_menu') as update:
        console.mavlink_packet(message('CAMERA_INFORMATION', 1, vendor_name=b'Siyi', model_name=b'ZR10'))
        console.mavlink_packet(message('CAMERA_INFORMATION', 1, vendor_name=b'Topotek', model_name=b'KHY10'))
        console.mavlink_packet(message('HEARTBEAT', 1, type=mavutil.mavlink.MAV_TYPE_FIXED_WING,
                                       autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA))
    assert console.component_name[1][1] == 'Plane'
    update.assert_not_called()


def test_flight_controller_heartbeat_reverts_camera_name(console):
    # a companion autopilot on another component is also not a camera
    console.mavlink_packet(message('CAMERA_INFORMATION', 191, vendor_name=b'Siyi', model_name=b'ZR10'))
    assert console.component_name[1][191] == 'Siyi-ZR10'
    console.mavlink_packet(message('HEARTBEAT', 191, type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                   autopilot=mavutil.mavlink.MAV_AUTOPILOT_PX4))
    assert console.component_name[1][191] == 'Copter'
    # cameras report no autopilot and keep their advertised name
    console.mavlink_packet(message('CAMERA_INFORMATION', vendor_name=b'Siyi', model_name=b'ZR10'))
    console.mavlink_packet(message('HEARTBEAT', type=mavutil.mavlink.MAV_TYPE_CAMERA,
                                   autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID))
    assert console.component_name[1][100] == 'Siyi-ZR10 (Camera)'
    # an advertised camera or gimbal type wins over a sloppy autopilot field
    console.mavlink_packet(message('HEARTBEAT', type=mavutil.mavlink.MAV_TYPE_CAMERA,
                                   autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA))
    assert console.component_name[1][100] == 'Siyi-ZR10 (Camera)'
    console.mavlink_packet(message('GIMBAL_DEVICE_INFORMATION', 154, vendor_name='Siyi', model_name='ZR10'))
    console.mavlink_packet(message('HEARTBEAT', 154, type=mavutil.mavlink.MAV_TYPE_GIMBAL,
                                   autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA))
    assert console.component_name[1][154] == 'Siyi-ZR10 (Gimbal)'
