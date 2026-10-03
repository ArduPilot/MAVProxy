'''what mavflightview reads out of a log'''

import struct

import pytest

HERE = (-35.363262, 149.165238)


def mavflightview():
    pytest.importorskip("cv2")
    pytest.importorskip("wx")
    import importlib.util
    import os
    path = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                        'MAVProxy', 'tools', 'mavflightview.py')
    # MAVProxy/tools is not a package, so the tool is loaded by path
    spec = importlib.util.spec_from_file_location('mavflightview', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_tlog(path, messages):
    '''write messages to a telemetry log, a second apart.  A message is
    sent by vehicle 1's autopilot unless it is given as (component, message)
    or (system, component, message)'''
    from pymavlink import mavutil
    usec = 1700000000 * 1000000
    with open(path, 'wb') as f:
        for m in messages:
            (system, component) = (1, 1)
            if isinstance(m, tuple) and len(m) == 2:
                (component, m) = m
            elif isinstance(m, tuple):
                (system, component, m) = m
            mav = mavutil.mavlink.MAVLink(None, srcSystem=system,
                                          srcComponent=component)
            usec += 1000000
            f.write(struct.pack('>Q', usec) + m.pack(mav))


def mission_item_int(seq, lat, lon, alt, command=None, params=(0, 0, 0, 0)):
    from pymavlink import mavutil
    mavlink = mavutil.mavlink
    if command is None:
        command = mavlink.MAV_CMD_NAV_WAYPOINT
    return mavlink.MAVLink_mission_item_int_message(
        255, 0, seq, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command, 0, 1, params[0], params[1], params[2], params[3],
        int(lat * 1e7), int(lon * 1e7), alt)


def mission_count(count):
    from pymavlink import mavutil
    return mavutil.mavlink.MAVLink_mission_count_message(255, 0, count)


def mission_clear_all():
    from pymavlink import mavutil
    return mavutil.mavlink.MAVLink_mission_clear_all_message(1, 1)


def heartbeat():
    from pymavlink import mavutil
    mavlink = mavutil.mavlink
    return mavlink.MAVLink_heartbeat_message(
        mavlink.MAV_TYPE_FIXED_WING, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        0, 0, 0, 3)


def param_value(name, value, count=1, index=0):
    from pymavlink import mavutil
    return mavutil.mavlink.MAVLink_param_value_message(
        name.encode(), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        count, index)


def global_position_int(lat, lon):
    from pymavlink import mavutil
    return mavutil.mavlink.MAVLink_global_position_int_message(
        0, int(lat * 1e7), int(lon * 1e7), 100000, 100000, 0, 0, 0, 0)


def view(tmp_path, messages, types=None, mission=None):
    '''run mavflightview over a telemetry log of messages.  Returns what
    mavflightview_mav does, and the options it ran with'''
    from pymavlink import mavutil
    tool = mavflightview()
    path = str(tmp_path / 'flight.tlog')
    write_tlog(path, messages)
    options = tool.mavflightview_options()
    options.types = types
    options.mission = mission
    mlog = mavutil.mavlink_connection(path)
    return (tool.mavflightview_mav(mlog, options), options)


class Message(object):
    '''a stand-in for a message from a log, with only the fields given'''

    def __init__(self, type, **fields):
        self._type = type
        self.__dict__.update(fields)

    def get_type(self):
        return self._type


class TestTlogMission(object):
    """a telemetry log carries its mission only as the MISSION_ITEM_INTs of
    a download, with none of the CMD messages a dataflash log has"""

    def mission(self, tmp_path, messages, types=None, mission=None):
        return view(tmp_path, messages, types, mission)[0][1]

    def items(self, n=3, lon=0.0, alt=50):
        '''the n items of a mission'''
        ret = [mission_item_int(0, HERE[0], HERE[1], 584)]
        for seq in range(1, n):
            ret.append(mission_item_int(
                seq, HERE[0] + seq * 0.001, HERE[1] + lon, alt + seq))
        return ret

    def log(self, mission):
        return ([global_position_int(HERE[0], HERE[1])] + mission +
                [global_position_int(HERE[0] + 0.001, HERE[1])])

    def altitudes(self, wp):
        return [wp.wp(i).z for i in range(wp.count())]

    def test_mission_items_are_read(self, tmp_path):
        wp = self.mission(tmp_path, self.log(self.items()))
        assert wp.count() == 3
        assert wp.wp(2).x == pytest.approx(HERE[0] + 0.002)
        assert wp.wp(2).y == pytest.approx(HERE[1])
        assert self.altitudes(wp) == [584, 51, 52]

    def test_mission_items_are_read_whatever_types_are_plotted(self, tmp_path):
        wp = self.mission(tmp_path, self.log(self.items()),
                          types='GLOBAL_POSITION_INT')
        assert wp.count() == 3

    def test_a_download_is_read(self, tmp_path):
        wp = self.mission(tmp_path, self.log([mission_count(3)] + self.items()))
        assert self.altitudes(wp) == [584, 51, 52]

    def test_items_arriving_out_of_order_are_all_kept(self, tmp_path):
        # a download where some items had to be asked for again
        items = self.items(4)
        items = [mission_count(4), items[3], items[1], items[0], items[2]]
        wp = self.mission(tmp_path, self.log(items))
        assert [wp.wp(i).seq for i in range(wp.count())] == [0, 1, 2, 3]
        assert self.altitudes(wp) == [584, 51, 52, 53]

    def test_nothing_is_made_up_for_items_never_seen(self, tmp_path):
        # a download which never finished: item 2 did not arrive
        items = self.items(4)
        items = [mission_count(4), items[0], items[1], items[3]]
        wp = self.mission(tmp_path, self.log(items))
        assert self.altitudes(wp) == [584, 51]

    def test_a_download_which_never_finished_leaves_the_last_whole_one(
            self, tmp_path):
        items = ([mission_count(3)] + self.items() +
                 [mission_count(3)] + self.items(alt=100)[:2])
        wp = self.mission(tmp_path, self.log(items))
        assert self.altitudes(wp) == [584, 51, 52]

    def test_a_shorter_mission_later_leaves_nothing_of_the_longer_one(
            self, tmp_path):
        items = ([mission_count(4)] + self.items(4) +
                 [mission_count(2)] + self.items(2, alt=100))
        wp = self.mission(tmp_path, self.log(items))
        assert self.altitudes(wp) == [584, 101]

    def test_a_cleared_mission_is_not_drawn(self, tmp_path):
        items = [mission_count(3)] + self.items() + [mission_clear_all()]
        wp = self.mission(tmp_path, self.log(items))
        assert wp.count() == 0

    def test_a_mission_given_to_draw_is_left_alone(self, tmp_path):
        path = tmp_path / 'mission.txt'
        path.write_text(
            'QGC WPL 110\n'
            '0\t1\t0\t16\t0\t0\t0\t0\t%f\t%f\t584\t1\n'
            '1\t0\t3\t16\t0\t0\t0\t0\t%f\t%f\t77\t1\n' %
            (HERE[0], HERE[1], HERE[0] - 0.01, HERE[1]))
        items = [mission_count(3)] + self.items()
        wp = self.mission(tmp_path, self.log(items), mission=str(path))
        assert self.altitudes(wp) == [584, 77]

    def test_a_jump_to_nowhere_ends_the_mission_drawn(self, tmp_path):
        from pymavlink import mavutil
        tool = mavflightview()
        jump = mavutil.mavlink.MAVLink_mission_item_int_message(
            255, 0, 2, 0, mavutil.mavlink.MAV_CMD_DO_JUMP, 0, 1,
            20, 1, 0, 0, 0, 0, 0)
        log = [heartbeat(), global_position_int(HERE[0], HERE[1]),
               mission_item_int(0, HERE[0], HERE[1], 584),
               mission_item_int(1, HERE[0] + 0.01, HERE[1], 100), jump,
               mission_item_int(3, HERE[0] + 0.01, HERE[1] + 0.01, 100),
               global_position_int(HERE[0] + 0.01, HERE[1])]
        (ret, options) = view(tmp_path, log)
        (wp, mav_type) = (ret[1], ret[4])
        assert wp.count() == 4
        objects = tool.mission_objects(wp, options, mav_type, 'T')
        lines = [o for o in objects if type(o).__name__ == 'SlipPolygon']
        # the line runs to the jump and stops; the item after it, which
        # the vehicle never reaches, is drawn apart from it
        assert [len(line.points) for line in lines] == [2, 1]
        # and the live map is given the same mission to draw
        added = []

        class Map(object):
            def add_object(self, obj):
                added.append(obj)
        tool.display_waypoints(wp, Map())
        assert [o for o in added if type(o).__name__ == 'SlipPolygon']
        # the mission itself is left as it was
        assert wp.wp(2).param1 == 20


class TestLogMission(object):
    """the messages a log's mission is read from, one at a time: those which
    need MAVLink2 fields or a dataflash log to make"""

    def mission(self, messages):
        from pymavlink import mavwp
        mission = mavflightview().LogMission()
        for m in messages:
            mission.read(m)
        wp = mavwp.MAVWPLoader()
        mission.fill(wp)
        return [wp.wp(i).z for i in range(wp.count())]

    def item(self, seq, alt, mission_type=0):
        return Message('MISSION_ITEM', seq=seq, x=HERE[0], y=HERE[1], z=alt,
                       mission_type=mission_type)

    def count(self, count, mission_type=0):
        return Message('MISSION_COUNT', count=count,
                       mission_type=mission_type)

    def cmd(self, num, alt, total=None):
        fields = dict(CNum=num, CId=16, Prm1=0, Prm2=0, Prm3=0, Prm4=0,
                      Lat=HERE[0], Lng=HERE[1], Alt=alt)
        if total is not None:
            fields['CTot'] = total
        return Message('CMD', **fields)

    def test_fence_and_rally_points_are_not_the_mission(self):
        from pymavlink import mavutil
        mission = [self.count(2), self.item(0, 584), self.item(1, 50)]
        for mission_type in (mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                             mavutil.mavlink.MAV_MISSION_TYPE_RALLY):
            mission += [self.count(3, mission_type),
                        self.item(0, 1000, mission_type),
                        self.item(1, 1001, mission_type),
                        self.item(2, 1002, mission_type)]
        assert self.mission(mission) == [584, 50]

    def test_clearing_the_fence_leaves_the_mission(self):
        from pymavlink import mavutil
        fence = mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        mission = [self.count(2), self.item(0, 584), self.item(1, 50),
                   Message('MISSION_CLEAR_ALL', mission_type=fence)]
        assert self.mission(mission) == [584, 50]
        everything = mavutil.mavlink.MAV_MISSION_TYPE_ALL
        mission[-1] = Message('MISSION_CLEAR_ALL', mission_type=everything)
        assert self.mission(mission) == []

    def test_a_mavlink1_item_is_a_mission_item(self):
        item = Message('MISSION_ITEM', seq=0, x=HERE[0], y=HERE[1], z=584)
        assert self.mission([item]) == [584]

    def test_the_last_mission_a_dataflash_log_writes_out(self):
        dumps = ([self.cmd(n, 50 + n, 4) for n in range(4)] +
                 [self.cmd(n, 100 + n, 2) for n in range(2)])
        assert self.mission(dumps) == [100, 101]

    def test_dataflash_mission_with_no_count_of_its_items(self):
        # older logs have no CTot: each dump ends where the next starts
        dumps = ([self.cmd(n, 50 + n) for n in range(4)] +
                 [self.cmd(n, 100 + n) for n in range(2)])
        assert self.mission(dumps) == [100, 101]
        assert self.mission(dumps[:4]) == [50, 51, 52, 53]

    def test_a_cleared_dataflash_mission_is_not_drawn(self):
        # the logger writes "New mission" before each mission it writes
        # out, and nothing after it for one which has been cleared
        new_mission = Message('MSG', Message='New mission')
        dumps = ([new_mission] + [self.cmd(n, 50 + n, 4) for n in range(4)] +
                 [new_mission])
        assert self.mission(dumps) == []
        # other messages leave it alone, and the next mission is drawn
        dumps += [Message('MSG', Message='Mission: 1 WP')]
        dumps += [new_mission] + [self.cmd(n, 100 + n, 2) for n in range(2)]
        assert self.mission(dumps) == [100, 101]

    def test_a_cleared_dataflash_mission_is_not_drawn_from_a_log(self, tmp_path):
        # a dataflash log through mavflightview itself, which has to ask the
        # log for the logger's messages as well as the mission
        tool = mavflightview()
        messages = (
            [Message('POS', Lat=HERE[0], Lng=HERE[1], Alt=584.0),
             Message('MSG', Message='New mission')] +
            [self.cmd(n, 50 + n, 3) for n in range(3)] +
            [Message('MSG', Message='New mission'),
             Message('POS', Lat=HERE[0] + 0.01, Lng=HERE[1], Alt=600.0)])

        class Log(object):
            flightmode = 'AUTO'
            messages = {}

            def __init__(self):
                self.queue = []

            def rewind(self):
                self.queue = list(messages)

            def recv_match(self, type=None):
                while self.queue:
                    m = self.queue.pop(0)
                    m._timestamp = 1.0
                    if type is None or m.get_type() in type:
                        return m
                return None

            def check_condition(self, condition):
                return True
        options = tool.mavflightview_options()
        options.colour_source = None
        ret = tool.mavflightview_mav(Log(), options)
        assert ret[1].count() == 0


class TestTlogParameters(object):
    """a loiter with no radius of its own is drawn at WP_LOITER_RAD, which a
    telemetry log only carries in its PARAM_VALUEs"""

    def view(self, tmp_path, messages):
        return view(tmp_path, messages)

    def radius(self, tmp_path, messages):
        return self.view(tmp_path, messages)[1].default_circle_radius

    def log(self, params, with_heartbeat=True):
        from pymavlink import mavutil
        loiter = mission_item_int(
            1, HERE[0] + 0.01, HERE[1], 100,
            command=mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
            params=(3, 0, 0, 0))
        start = [heartbeat()] if with_heartbeat else []
        return (start + [global_position_int(HERE[0], HERE[1])] + params +
                [mission_item_int(0, HERE[0], HERE[1], 584), loiter,
                 global_position_int(HERE[0] + 0.01, HERE[1])])

    def test_parameters_sent_by_the_autopilot(self, tmp_path):
        # opening a log reads the first message of each type in it, so
        # WP_LOITER_RAD is not the first parameter, as in any real download
        log = self.log([param_value('AHRS_EKF_TYPE', 3, 2, 0),
                        param_value('WP_LOITER_RAD', 150, 2, 1)])
        assert self.radius(tmp_path, log) == 150

    def test_the_loiter_is_drawn_at_that_radius(self, tmp_path):
        log = self.log([param_value('AHRS_EKF_TYPE', 3, 2, 0),
                        param_value('WP_LOITER_RAD', 150, 2, 1)])
        (ret, options) = self.view(tmp_path, log)
        (wp, mav_type) = (ret[1], ret[4])
        objects = mavflightview().mission_objects(wp, options, mav_type, 'T')
        circles = [o for o in objects if type(o).__name__ == 'SlipCircle']
        assert [c.radius for c in circles] == [150]

    def test_parameters_mavproxy_fetched_over_ftp(self, tmp_path):
        from pymavlink import mavutil
        ftp = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
        # the autopilot sends a few of its own too, as they change
        log = self.log([(ftp, param_value('WP_RADIUS', 90, 2, 0)),
                        (ftp, param_value('WP_LOITER_RAD', 150, 2, 1)),
                        param_value('STAT_RUNTIME', 1000)])
        assert self.radius(tmp_path, log) == 150

    def test_the_autopilots_own_value_wins(self, tmp_path):
        from pymavlink import mavutil
        ftp = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
        log = self.log([(ftp, param_value('WP_LOITER_RAD', 150)),
                        param_value('WP_LOITER_RAD', 80)])
        assert self.radius(tmp_path, log) == 80

    def test_a_log_with_no_heartbeat(self, tmp_path):
        from pymavlink import mavutil
        ftp = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
        log = self.log([(ftp, param_value('WP_RADIUS', 90, 2, 0)),
                        (ftp, param_value('WP_LOITER_RAD', 150, 2, 1)),
                        param_value('STAT_RUNTIME', 1000)],
                       with_heartbeat=False)
        assert self.radius(tmp_path, log) == 150
        log = self.log([param_value('AHRS_EKF_TYPE', 3, 2, 0),
                        param_value('WP_LOITER_RAD', 120, 2, 1)],
                       with_heartbeat=False)
        assert self.radius(tmp_path, log) == 120

    def test_no_guessing_between_vehicles_with_no_heartbeat(self, tmp_path):
        from pymavlink import mavutil
        ftp = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
        log = self.log([(1, ftp, param_value('WP_RADIUS', 90, 2, 0)),
                        (1, ftp, param_value('WP_LOITER_RAD', 150, 2, 1)),
                        (2, ftp, param_value('WP_RADIUS', 90, 2, 0)),
                        (2, ftp, param_value('WP_LOITER_RAD', 60, 2, 1))],
                       with_heartbeat=False)
        assert self.radius(tmp_path, log) is None

    def test_the_vehicle_the_log_follows(self, tmp_path):
        from pymavlink import mavutil
        ftp = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
        log = self.log([(2, ftp, param_value('WP_RADIUS', 90, 2, 0)),
                        (2, ftp, param_value('WP_LOITER_RAD', 60, 2, 1)),
                        (1, ftp, param_value('WP_RADIUS', 90, 2, 0)),
                        (1, ftp, param_value('WP_LOITER_RAD', 150, 2, 1))])
        assert self.radius(tmp_path, log) == 150
