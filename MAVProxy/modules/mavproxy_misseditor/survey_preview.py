'''Single pending survey preview, handed from the event thread to idle_task.'''

import threading


class SurveyPreview:
    key = 'mission-editor-survey'

    def __init__(self):
        self.lock = threading.Lock()
        self.points = ()
        self.maps = {}
        self.closed = False

    def set_points(self, points):
        with self.lock:
            if not self.closed:
                self.points = tuple(points)

    def draw(self, maps):
        '''Only called in the MAVProxy main thread; map images use RGB.'''
        with self.lock:
            points = self.points
        for old in list(self.maps):
            if old not in maps or not points:
                old.remove_object(self.key)
                del self.maps[old]
        if not points:
            return
        from MAVProxy.modules.mavproxy_map import mp_slipmap
        for display in maps:
            if self.maps.get(display) != points:
                display.add_object(mp_slipmap.SlipPolygon(
                    self.key, points, layer='Survey', colour=(0, 0, 255),
                    linewidth=2, arrow=True))
                self.maps[display] = points

    def close(self):
        with self.lock:
            self.closed = True
            self.points = ()
        self.draw([])
