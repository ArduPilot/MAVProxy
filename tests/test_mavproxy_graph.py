#!/usr/bin/env python3
"""Unit tests for the graph module's onboard-log field streaming.

AP_FLAKE8_CLEAN
"""

import contextlib
import io
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

os.environ.setdefault("MAVLINK20", "1")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from pymavlink import mavutil  # noqa: E402

from MAVProxy.modules import mavproxy_graph  # noqa: E402


class FakeMav:
    def __init__(self):
        self.log_stream_requests = []

    def log_stream_named_value_send(self, target_system, target_component,
                                    msg_name, field_name, instance,
                                    name, interval_us):
        self.log_stream_requests.append(
            (target_system, target_component, msg_name, field_name, instance,
             name, interval_us))


class FakeMaster:
    def __init__(self):
        self.mav = FakeMav()
        self.messages = {}


class FakeMPState:
    def __init__(self):
        self.command_map = {}
        self.completions = {}
        self.completion_functions = {}
        self.settings = SimpleNamespace(target_system=1, target_component=1)
        self._master = FakeMaster()
        self.vehicle_masters = {}
        self.link_map = {}

    def master(self, target_sysid=-1):
        if target_sysid != -1 and target_sysid in self.vehicle_masters:
            return self.vehicle_masters[target_sysid]
        return self._master

    def foreach_mav(self, sysid, compid, closure):
        for master in self.link_map.get((sysid, compid), []):
            closure(master.mav)


class Message:
    def __init__(self, message_type, src_system=1, **fields):
        self._message_type = message_type
        self._src_system = src_system
        self.__dict__.update(fields)

    def get_type(self):
        return self._message_type

    def get_srcSystem(self):
        return self._src_system


@unittest.skipUnless(
    'interval_us' in getattr(
        getattr(mavutil.mavlink, 'MAVLink_log_stream_named_value_message', None),
        'fieldnames', []),
    "pymavlink dialect does not have LOG_STREAM_NAMED_VALUE")
@mock.patch("MAVProxy.modules.lib.live_graph.LiveGraph")
class LogStreamTest(unittest.TestCase):
    def setUp(self):
        self.state = FakeMPState()
        self.module = mavproxy_graph.GraphModule(self.state)

    def requests(self):
        return self.state._master.mav.log_stream_requests

    def graph(self, *fields):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.module.cmd_graph(list(fields))
        return out.getvalue()

    def test_field_is_requested_and_plotted(self, live_graph):
        self.graph("ATT.Roll")
        self.assertEqual(self.requests(), [(1, 1, b'ATT', b'Roll', -1, b'ATT.Roll', 0)])
        self.module.mavlink_packet(
            Message("NAMED_VALUE_FLOAT", name="ATT.Roll", value=1.5))
        live_graph.return_value.add_values.assert_called_with([1.5])

    def test_instanced_field(self, live_graph):
        self.graph("XKF4[1].SS")
        self.assertEqual(self.requests(), [(1, 1, b'XKF4', b'SS', 1, b'XKF4[1].SS', 0)])
        self.module.mavlink_packet(
            Message("NAMED_VALUE_INT", name="XKF4[1].SS", value=231))
        live_graph.return_value.add_values.assert_called_with([231])

    def test_mavlink_message_is_not_requested(self, live_graph):
        self.graph("WIND.speed")
        self.assertEqual(self.requests(), [])

    def test_truncation_collision_streams_under_generated_name(self, live_graph):
        live_graph.side_effect = lambda *args, **kwargs: mock.Mock()
        output = self.graph("IMU[0].GyrX", "IMU[0].GyrY", "IMU[0].GyrZ")
        self.assertEqual(self.requests(), [
            (1, 1, b'IMU', b'GyrX', 0, b'IMU[0].Gyr', 0),
            (1, 1, b'IMU', b'GyrY', 0, b'IMU[0].G~1', 0),
            (1, 1, b'IMU', b'GyrZ', 0, b'IMU[0].G~2', 0),
        ])
        self.assertIn("streaming IMU[0].GyrY as IMU[0].G~1", output)
        # each field is plotted from its own stream:
        self.module.mavlink_packet(
            Message("NAMED_VALUE_FLOAT", name="IMU[0].Gyr", value=0.25))
        graph = self.module.graphs[0]
        graph.livegraph.add_values.assert_called_with([0.25, None, None])
        self.module.mavlink_packet(
            Message("NAMED_VALUE_FLOAT", name="IMU[0].G~2", value=-0.5))
        graph.livegraph.add_values.assert_called_with([0.25, None, -0.5])
        # a second graph of an already-streamed field reuses the stream:
        self.graph("IMU[0].GyrY")
        self.assertEqual(len(self.requests()), 3)
        self.module.mavlink_packet(
            Message("NAMED_VALUE_FLOAT", name="IMU[0].G~1", value=0.125))
        self.module.graphs[1].livegraph.add_values.assert_called_with([0.125])

    def test_key_is_built_from_parsed_pieces(self, live_graph):
        self.graph("XKF4[01].SS")
        self.assertEqual(self.requests(), [(1, 1, b'XKF4', b'SS', 1, b'XKF4[1].SS', 0)])
        self.module.mavlink_packet(
            Message("NAMED_VALUE_INT", name="XKF4[1].SS", value=231))
        live_graph.return_value.add_values.assert_called_with([231])

    def test_oversize_instance_is_refused(self, live_graph):
        output = self.graph("XKF4[40000].SS")
        self.assertEqual(self.requests(), [])
        self.assertIn("too large", output)

    def test_label_does_not_hide_the_field(self, live_graph):
        self.graph("ATT.Roll<Roll>")
        self.assertEqual(self.requests(), [(1, 1, b'ATT', b'Roll', -1, b'ATT.Roll', 0)])
        self.module.mavlink_packet(
            Message("NAMED_VALUE_FLOAT", name="ATT.Roll", value=1.5))
        live_graph.return_value.add_values.assert_called_with([1.5])

    def test_no_link_requests_when_one_appears(self, live_graph):
        self.state._master = None
        self.graph("ATT.Roll")
        self.assertEqual(len(self.module.graphs), 1)
        self.state._master = FakeMaster()
        self.module.idle_task()
        self.assertEqual(self.requests(), [(1, 1, b'ATT', b'Roll', -1, b'ATT.Roll', 0)])

    def test_stalled_stream_is_rerequested(self, live_graph):
        self.graph("ATT.Roll")
        stream = self.module.log_streams["ATT.Roll"]
        self.module.idle_task()
        self.assertEqual(len(self.requests()), 1)
        stream.last_request_time -= 10
        self.module.idle_task()
        self.assertEqual(len(self.requests()), 2)
        # a stream with fresh data is not re-requested:
        self.module.mavlink_packet(
            Message("NAMED_VALUE_FLOAT", name="ATT.Roll", value=1.5))
        stream.last_request_time -= 10
        self.module.idle_task()
        self.assertEqual(len(self.requests()), 2)
        # a stream no open graph wants is stopped, its name staying
        # reserved for the rest of the session:
        self.module.graphs = []
        stream.last_value_time -= 10
        self.module.idle_task()
        self.assertEqual(self.requests()[-1], (1, 1, b'ATT', b'Roll', -1, b'ATT.Roll', -1))
        self.assertIn("ATT.Roll", self.module.log_streams)
        self.module.idle_task()
        self.assertEqual(len(self.requests()), 3)
        stream.stop_time -= 10
        self.module.idle_task()
        self.assertIn("ATT.Roll", self.module.log_streams)
        self.assertEqual(len(self.requests()), 3)

    def test_stream_with_no_values_backs_off(self, live_graph):
        self.graph("ATT.Roll")
        stream = self.module.log_streams["ATT.Roll"]
        stream.last_request_time -= 4
        self.module.idle_task()
        self.assertEqual(len(self.requests()), 2)
        # the retry interval grows while no value has ever arrived:
        stream.last_request_time -= 4
        self.module.idle_task()
        self.assertEqual(len(self.requests()), 2)
        stream.last_request_time -= 3
        self.module.idle_task()
        self.assertEqual(len(self.requests()), 3)
        # a value arriving resets the interval:
        self.module.mavlink_packet(
            Message("NAMED_VALUE_FLOAT", name="ATT.Roll", value=1.5))
        self.assertEqual(stream.request_interval, 3)

    def test_unload_stops_streams(self, live_graph):
        self.graph("ATT.Roll")
        self.module.unload()
        self.assertEqual(self.requests()[-1], (1, 1, b'ATT', b'Roll', -1, b'ATT.Roll', -1))
        self.assertEqual(self.module.log_streams, {})

    def test_stop_waits_for_a_link(self, live_graph):
        self.graph("ATT.Roll")
        self.module.graphs = []
        self.state._master = None
        self.module.idle_task()
        self.assertIn("ATT.Roll", self.module.log_streams)
        self.state._master = FakeMaster()
        self.module.idle_task()
        self.assertEqual(self.requests(), [(1, 1, b'ATT', b'Roll', -1, b'ATT.Roll', -1)])
        self.assertIn("ATT.Roll", self.module.log_streams)

    def test_never_requested_stream_is_dropped_silently(self, live_graph):
        self.state._master = None
        self.graph("ATT.Roll")
        self.module.graphs = []
        self.state._master = FakeMaster()
        self.module.idle_task()
        self.assertEqual(self.requests(), [])
        self.assertNotIn("ATT.Roll", self.module.log_streams)

    def test_stop_is_resent_while_values_arrive(self, live_graph):
        live_graph.side_effect = lambda *args, **kwargs: mock.Mock()
        self.graph("IMU[0].GyrX")
        stream = self.module.log_streams["IMU[0].Gyr"]
        self.module.graphs = []
        stream.last_value_time -= 10
        self.module.idle_task()
        self.assertEqual(self.requests()[-1][-1], -1)
        # values still arriving (a lost stop): the stop is re-sent and
        # the name is not handed to another field meanwhile:
        self.module.mavlink_packet(
            Message("NAMED_VALUE_FLOAT", name="IMU[0].Gyr", value=0.25))
        stream.last_request_time -= 4
        self.module.idle_task()
        self.assertEqual(self.requests()[-2:], [
            (1, 1, b'IMU', b'GyrX', 0, b'IMU[0].Gyr', -1),
            (1, 1, b'IMU', b'GyrX', 0, b'IMU[0].Gyr', -1),
        ])
        self.graph("IMU[0].GyrY")
        self.assertEqual(self.requests()[-1], (1, 1, b'IMU', b'GyrY', 0, b'IMU[0].G~1', 0))

    def test_regraphed_field_restarts_stopping_stream(self, live_graph):
        self.graph("ATT.Roll")
        self.module.graphs = []
        stream = self.module.log_streams["ATT.Roll"]
        stream.last_value_time -= 10
        self.module.idle_task()
        self.assertEqual(self.requests()[-1][-1], -1)
        self.graph("ATT.Roll")
        self.assertEqual(self.requests()[-1], (1, 1, b'ATT', b'Roll', -1, b'ATT.Roll', 0))
        self.assertIsNone(stream.stop_time)

    def test_regraph_reuses_generated_name(self, live_graph):
        live_graph.side_effect = lambda *args, **kwargs: mock.Mock()
        self.graph("IMU[0].GyrX")
        self.graph("IMU[0].GyrY")
        self.assertEqual(self.requests()[-1], (1, 1, b'IMU', b'GyrY', 0, b'IMU[0].G~1', 0))
        # close the GyrX graph; its stream is stopped but its name
        # stays reserved:
        self.module.graphs = self.module.graphs[1:]
        gyrx = self.module.log_streams["IMU[0].Gyr"]
        gyrx.last_value_time -= 10
        self.module.idle_task()
        self.assertEqual(self.requests()[-1][-1], -1)
        # re-graphing GyrY reuses its stream rather than opening a
        # second stream of the same field:
        count = len(self.requests())
        self.graph("IMU[0].GyrY")
        self.assertEqual(len(self.requests()), count)
        self.module.mavlink_packet(
            Message("NAMED_VALUE_FLOAT", name="IMU[0].G~1", value=0.5))
        self.module.graphs[-1].livegraph.add_values.assert_called_with([0.5])

    def test_stopped_name_is_not_reused_for_another_field(self, live_graph):
        live_graph.side_effect = lambda *args, **kwargs: mock.Mock()
        self.graph("IMU[0].GyrX")
        stream = self.module.log_streams["IMU[0].Gyr"]
        self.module.graphs = []
        stream.last_value_time -= 10
        self.module.idle_task()
        stream.stop_time -= 100
        self.module.idle_task()
        # however long the stream has been quiet, its name is not
        # handed to a different field:
        self.graph("IMU[0].GyrY")
        self.assertEqual(self.requests()[-1], (1, 1, b'IMU', b'GyrY', 0, b'IMU[0].G~1', 0))

    def test_refused_name_is_retried_under_generated_name(self, live_graph):
        self.graph("IMU[0].AccZ")
        self.assertEqual(self.requests()[-1], (1, 1, b'IMU', b'AccZ', 0, b'IMU[0].Acc', 0))
        # the vehicle refuses the name (eg. a previous MAVProxy session
        # left it streaming a different field); a leftover value from
        # that stream arriving first does not defeat the retry, and the
        # leftover stream is stopped rather than left running under the
        # retired name:
        self.module.mavlink_packet(
            Message("NAMED_VALUE_FLOAT", name="IMU[0].Acc", value=-9.8))
        self.module.mavlink_packet(
            Message("STATUSTEXT", text="Unable to stream IMU[0].Acc"))
        self.assertEqual(self.requests()[-2:], [
            (1, 1, b'IMU', b'AccZ', 0, b'IMU[0].Acc', -1),
            (1, 1, b'IMU', b'AccZ', 0, b'IMU[0].A~1', 0),
        ])
        # the previous session also left IMU[0].A~1 streaming; the
        # second refusal is retried too (no stop: no value has arrived
        # under the new name):
        self.module.mavlink_packet(
            Message("STATUSTEXT", text="Unable to stream IMU[0].A~1"))
        self.assertEqual(self.requests()[-1], (1, 1, b'IMU', b'AccZ', 0, b'IMU[0].A~2', 0))
        # the foreign streams' values no longer plot; the renamed
        # stream's do:
        live_graph.return_value.add_values.reset_mock()
        self.module.mavlink_packet(
            Message("NAMED_VALUE_FLOAT", name="IMU[0].Acc", value=-9.8))
        self.module.mavlink_packet(
            Message("NAMED_VALUE_FLOAT", name="IMU[0].A~1", value=-0.001))
        live_graph.return_value.add_values.assert_not_called()
        self.module.mavlink_packet(
            Message("NAMED_VALUE_FLOAT", name="IMU[0].A~2", value=0.5))
        live_graph.return_value.add_values.assert_called_with([0.5])
        # the retries are bounded (eg. a genuinely-bad field is refused
        # whatever it is named); the final rename stops A~2, which had
        # produced a value:
        for n in range(2, 20):
            self.module.mavlink_packet(
                Message("STATUSTEXT", text="Unable to stream IMU[0].A~%u" % n))
        self.assertEqual(self.requests()[-2:], [
            (1, 1, b'IMU', b'AccZ', 0, b'IMU[0].A~2', -1),
            (1, 1, b'IMU', b'AccZ', 0, b'IMU[0].A~3', 0),
        ])
        self.assertEqual(len(self.requests()), 6)

    def test_values_matched_by_source_vehicle(self, live_graph):
        self.graph("ATT.Roll")
        stream = self.module.log_streams["ATT.Roll"]
        # a same-named stream on another vehicle neither plots...
        self.module.mavlink_packet(
            Message("NAMED_VALUE_FLOAT", src_system=2, name="ATT.Roll", value=9.9))
        live_graph.return_value.add_values.assert_not_called()
        self.assertEqual(stream.last_value_time, 0)
        # ...nor masks the stall which moves the stream to the newly
        # selected vehicle:
        self.state.settings.target_system = 2
        stream.last_request_time -= 4
        self.module.idle_task()
        self.assertEqual(self.requests()[-1], (2, 1, b'ATT', b'Roll', -1, b'ATT.Roll', 0))

    def test_stop_with_component_zero_uses_autopilot_links(self, live_graph):
        self.state.settings.target_component = 0
        self.graph("ATT.Roll")
        link2 = FakeMaster()
        self.state.link_map = {(1, 1): [self.state._master, link2]}
        self.module.graphs = []
        stream = self.module.log_streams["ATT.Roll"]
        stream.last_value_time -= 10
        self.module.idle_task()
        stop = (1, 0, b'ATT', b'Roll', -1, b'ATT.Roll', -1)
        self.assertEqual(self.requests()[-1], stop)
        self.assertEqual(link2.mav.log_stream_requests, [stop])

    def test_stop_covers_every_link_to_the_vehicle(self, live_graph):
        self.graph("ATT.Roll")
        link2 = FakeMaster()
        self.state.link_map = {(1, 1): [self.state._master, link2]}
        self.module.graphs = []
        stream = self.module.log_streams["ATT.Roll"]
        stream.last_value_time -= 10
        self.module.idle_task()
        stop = (1, 1, b'ATT', b'Roll', -1, b'ATT.Roll', -1)
        self.assertEqual(self.requests()[-1], stop)
        self.assertEqual(link2.mav.log_stream_requests, [stop])

    def test_stop_uses_the_streams_link(self, live_graph):
        self.graph("ATT.Roll")
        vehicle1_link = FakeMaster()
        self.state.vehicle_masters[1] = vehicle1_link
        self.state.settings.target_system = 2
        self.module.graphs = []
        stream = self.module.log_streams["ATT.Roll"]
        stream.last_value_time -= 10
        self.module.idle_task()
        self.assertEqual(self.requests(), [(1, 1, b'ATT', b'Roll', -1, b'ATT.Roll', 0)])
        self.assertEqual(vehicle1_link.mav.log_stream_requests,
                         [(1, 1, b'ATT', b'Roll', -1, b'ATT.Roll', -1)])

    def test_stop_goes_to_the_streams_vehicle(self, live_graph):
        self.graph("ATT.Roll")
        self.state.settings.target_system = 2
        self.module.graphs = []
        stream = self.module.log_streams["ATT.Roll"]
        stream.last_value_time -= 10
        self.module.idle_task()
        self.assertEqual(self.requests()[-1], (1, 1, b'ATT', b'Roll', -1, b'ATT.Roll', -1))

    def test_stream_follows_selected_vehicle(self, live_graph):
        self.graph("ATT.Roll")
        self.state.settings.target_system = 2
        stream = self.module.log_streams["ATT.Roll"]
        stream.last_request_time -= 4
        self.module.idle_task()
        # stopped on the old vehicle, requested from the new one:
        self.assertEqual(self.requests()[-2:], [
            (1, 1, b'ATT', b'Roll', -1, b'ATT.Roll', -1),
            (2, 1, b'ATT', b'Roll', -1, b'ATT.Roll', 0),
        ])


if __name__ == '__main__':
    unittest.main()
