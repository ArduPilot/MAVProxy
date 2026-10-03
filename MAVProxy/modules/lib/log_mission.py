'''the mission a log holds, as it was downloaded, uploaded or written out

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil


# the messages a log carries its mission in
MISSION_TYPES = ['MISSION_COUNT', 'MISSION_CLEAR_ALL',
                 'MISSION_ITEM', 'MISSION_ITEM_INT', 'CMD', 'MSG']
# a MAVLink1 message has no mission_type, and is only ever about the mission
DEFAULT_MISSION_TYPE = mavutil.mavlink.MAV_MISSION_TYPE_MISSION


class LogMission(object):
    '''the last whole mission a log holds.  A telemetry log carries the
    missions downloaded from or uploaded to the vehicle, each a MISSION_COUNT
    and then its items, in whatever order they were asked for, and perhaps
    never all of them.  A dataflash log carries a CMD for each item of the
    mission whenever the vehicle writes the whole mission out: at the start
    of the log, and after it changes.

    mission_type is which of the tables the mission protocol carries to
    keep: the mission itself, or the rally points or fence, which travel in
    the same messages.  Only the mission is written out as CMD, so a table
    of another kind is whatever a telemetry log carries of it'''

    def __init__(self, mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION):
        self.mission_type = mission_type
        # seq -> MISSION_ITEM, for the last mission to arrive whole
        self.items = {}
        # (count, {seq: MISSION_ITEM}) for one arriving, while it does.  The
        # count is None where the log does not say how many items to expect
        self.transfer = None
        # how many times a table has arrived whole, or been cleared away:
        # the vehicle has said what it holds, even where that is the table
        # held already
        self.arrivals = 0

    def start(self, count):
        '''a mission of count items starts to arrive'''
        if self.transfer is not None and self.transfer[0] is None:
            # with no count to finish it, the last one ended where this starts
            self.items = self.transfer[1]
            self.arrivals += 1
        self.transfer = (count, {})
        if count == 0:
            self.clear()

    def clear(self):
        self.items = {}
        self.transfer = None
        self.arrivals += 1

    def add(self, item):
        if self.transfer is None:
            # an item which is not part of a mission seen to start arriving:
            # a log which starts part way through, or a single item changed
            self.items[item.seq] = item
            self.arrivals += 1
            return
        (count, items) = self.transfer
        if count is not None and item.seq >= count:
            return
        items[item.seq] = item
        if count is not None and len(items) == count:
            self.items = items
            self.transfer = None
            self.arrivals += 1

    def read(self, m, type=None):
        '''take in one of the MISSION_TYPES messages.  type is what to take
        it as, for a caller which has made one message stand in for another
        -- a telemetry log's STATUSTEXT for a dataflash MSG, say'''
        if type is None:
            type = m.get_type()
        if self.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_MISSION \
                and type in ('MSG', 'CMD'):
            # a dataflash log writes out the mission this way, and nothing
            # else the mission protocol carries
            return
        if type == 'MSG':
            if getattr(m, 'Message', None) == 'New mission':
                # the logger says so before it writes the mission out, and
                # a mission cleared away is written out as nothing more
                self.start(0)
            return
        if type == 'CMD':
            if m.CNum == 0:
                # the vehicle writes out the whole mission from home onwards
                self.start(getattr(m, 'CTot', None))
            # a log from before 2014 carries neither the frame nor the
            # parameters of an item
            params = [getattr(m, 'Prm%u' % i, 0.0) for i in range(1, 5)]
            self.add(mavutil.mavlink.MAVLink_mission_item_message(
                0, 0, m.CNum,
                getattr(m, 'Frame',
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT),
                m.CId, 0, 1, params[0], params[1], params[2], params[3],
                m.Lat, m.Lng, m.Alt))
            return
        # fence and rally points travel in the same messages
        mission_type = getattr(m, 'mission_type', DEFAULT_MISSION_TYPE)
        if type == 'MISSION_CLEAR_ALL':
            if mission_type in (self.mission_type,
                                mavutil.mavlink.MAV_MISSION_TYPE_ALL):
                self.clear()
            return
        if mission_type != self.mission_type:
            return
        if type == 'MISSION_COUNT':
            self.start(m.count)
        elif type == 'MISSION_ITEM_INT':
            self.add(mavutil.mavlink.MAVLink_mission_item_message(
                0, 0, m.seq, m.frame, m.command, m.current, m.autocontinue,
                m.param1, m.param2, m.param3, m.param4,
                m.x / 1.0e7, m.y / 1.0e7, m.z))
        else:
            self.add(m)

    def whole(self):
        '''the mission by sequence, from its first item as far as it runs
        without a gap.  Nothing is made up to stand in for an item never
        seen, and a mission which never arrived whole is not one: a log
        which starts part way through one carries no mission at all, since
        there is no flying, numbering or jumping through a mission whose
        first items are missing'''
        items = self.items
        if self.transfer is not None:
            (count, arriving) = self.transfer
            if count is None or not items:
                # the last one ran to the end of the log, or it never arrived
                # whole and nothing before it did
                items = arriving
        out = {}
        seq = 0
        while seq in items:
            out[seq] = items[seq]
            seq += 1
        return out

    def partial(self):
        '''whether what whole() gives is less than what arrived whole: the
        log ends part way through the only transfer it carries.  What did
        arrive is where the vehicle was told to go, and is worth drawing,
        but not worth flying: what would have followed it is not known'''
        if self.transfer is None:
            return False
        (count, arriving) = self.transfer
        return count is not None and not self.items and len(arriving) < count

    def fill(self, wp):
        '''add the mission to the MAVWPLoader wp'''
        for (_, item) in sorted(self.whole().items()):
            wp.add(item)
