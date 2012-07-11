TouchHandler = function(map, options) {
    if (map) {
        this.init(map, options);
    }
};

TouchHandler.prototype = {

    maxTapTime: 250,
    maxTapDistance: 30,
    maxDoubleTapDelay: 350,
    locations: {},
    taps: [],
    wasPinching: false,
    lastPinchCenter: null,

    init: function(map, options) {
        this.map = map;
        options = options || {};

        // Fail early if this isn't a touch device.
        if (!this.isTouchable()) return false;

        this._touchStartMachine = MM.bind(this.touchStartMachine, this);
        this._touchMoveMachine = MM.bind(this.touchMoveMachine, this);
        this._touchEndMachine = MM.bind(this.touchEndMachine, this);
        MM.addEvent(map.parent, 'touchstart',
                    this._touchStartMachine);
        MM.addEvent(map.parent, 'touchmove',
                    this._touchMoveMachine);
        MM.addEvent(map.parent, 'touchend',
                    this._touchEndMachine);

        this.options = {};
        this.options.snapToZoom = options.snapToZoom || true;
    },

    isTouchable: function() {
        var el = document.createElement('div');
        el.setAttribute('ongesturestart', 'return;');
        return (typeof el.ongesturestart === 'function');
    },

    remove: function() {
        // Fail early if this isn't a touch device.
        if (!this.isTouchable()) return false;

        MM.removeEvent(this.map.parent, 'touchstart',
                       this._touchStartMachine);
        MM.removeEvent(this.map.parent, 'touchmove',
                       this._touchMoveMachine);
        MM.removeEvent(this.map.parent, 'touchend',
                       this._touchEndMachine);
    },

    updateTouches: function(e) {
        for (var i = 0; i < e.touches.length; i += 1) {
            var t = e.touches[i];
            if (t.identifier in this.locations) {
                var l = this.locations[t.identifier];
                l.x = t.clientX;
                l.y = t.clientY;
                l.scale = e.scale;
            }
            else {
                this.locations[t.identifier] = {
                    scale: e.scale,
                    startPos: { x: t.clientX, y: t.clientY },
                    x: t.clientX,
                    y: t.clientY,
                    time: new Date().getTime()
                };
            }
        }
    },

    // Test whether touches are from the same source -
    // whether this is the same touchmove event.
    sameTouch: function(event, touch) {
        return (event && event.touch) &&
            (touch.identifier == event.touch.identifier);
    },

    touchStartMachine: function(e) {
        this.updateTouches(e);
        return MM.cancelEvent(e);
    },

    touchMoveMachine: function(e) {
        switch (e.touches.length) {
        case 1:
            this.onPanning(e.touches[0]);
            break;
        case 2:
            this.onPinching(e);
            break;
        }
        this.updateTouches(e);
        return MM.cancelEvent(e);
    },

    touchEndMachine: function(e) {
        var now = new Date().getTime();
        // round zoom if we're done pinching
        if (e.touches.length === 0 && this.wasPinching) {
            this.onPinched(this.lastPinchCenter);
        }

        // Look at each changed touch in turn.
        for (var i = 0; i < e.changedTouches.length; i += 1) {
            var t = e.changedTouches[i],
            loc = this.locations[t.identifier];
            // if we didn't see this one (bug?)
            // or if it was consumed by pinching already
            // just skip to the next one
            if (!loc || loc.wasPinch) {
                continue;
            }

            // we now know we have an event object and a
            // matching touch that's just ended. Let's see
            // what kind of event it is based on how long it
            // lasted and how far it moved.
            var pos = { x: t.clientX, y: t.clientY },
            time = now - loc.time,
            travel = MM.Point.distance(pos, loc.startPos);
            if (travel > this.maxTapDistance) {
                // we will to assume that the drag has been handled separately
            } else if (time > this.maxTapTime) {
                // close in space, but not in time: a hold
                pos.end = now;
                pos.duration = time;
                this.onHold(pos);
            } else {
                // close in both time and space: a tap
                pos.time = now;
                this.onTap(pos);
            }
        }

        // Weird, sometimes an end event doesn't get thrown
        // for a touch that nevertheless has disappeared.
        // Still, this will eventually catch those ids:

        var validTouchIds = {};
        for (var j = 0; j < e.touches.length; j++) {
            validTouchIds[e.touches[j].identifier] = true;
        }
        for (var id in this.locations) {
            if (!(id in validTouchIds)) {
                delete validTouchIds[id];
            }
        }

        return MM.cancelEvent(e);
    },

    onHold: function(hold) {
        // TODO
    },

    // Handle a tap event - mainly watch for a doubleTap
    onTap: function(tap) {
        if (this.taps.length &&
            (tap.time - this.taps[0].time) < this.maxDoubleTapDelay) {
            this.onDoubleTap(tap);
            this.taps = [];
            return;
        }
        this.taps = [tap];
    },

    // Handle a double tap by telling the drone to fly to the tapped
    // location.
    onDoubleTap: function(tap) {
	var target = this.map.pointLocation(new MM.Point(tap.x, tap.y));
	mmap.flyTo(target);
    },

    // Re-transform the actual map parent's CSS transformation
    onPanning: function(touch) {
    },

    onPinching: function(e) {
        // use the first two touches and their previous positions
        var t0 = e.touches[0],
        t1 = e.touches[1],
        p0 = new MM.Point(t0.clientX, t0.clientY),
        p1 = new MM.Point(t1.clientX, t1.clientY),
        l0 = this.locations[t0.identifier],
        l1 = this.locations[t1.identifier];

        // mark these touches so they aren't used as taps/holds
        l0.wasPinch = true;
        l1.wasPinch = true;

        // scale about the center of these touches
        var center = MM.Point.interpolate(p0, p1, 0.5);

        this.map.zoomBy(
            Math.log(e.scale) / Math.LN2 -
                Math.log(l0.scale) / Math.LN2);

        // pan from the previous center of these touches
        var prevCenter = MM.Point.interpolate(l0, l1, 0.5);

        this.wasPinching = true;
        this.lastPinchCenter = center;
    },

    // When a pinch event ends, round the zoom of the map.
    onPinched: function(p) {
        // TODO: easing
        if (this.options.snapToZoom) {
            var z = this.map.getZoom(), // current zoom
            tz = Math.round(z);     // target zoom
            this.map.zoomBy(tz - z);
        }
        this.wasPinching = false;
    }
};


// A handler that allows mouse-wheel zooming - zooming in
// when page would scroll up, and out when the page would scroll down.
MouseWheelHandler = function(map, precise) {
    // only init() if we get a map
    if (map) {
        this.init(map, precise);
        // allow (null, true) as constructor args
    } else if (arguments.length > 1) {
        this.precise = precise ? true : false;
    }
};

MouseWheelHandler.prototype = {
    precise: false,
    
    init: function(map) {
        this.map = map;
        this._mouseWheel = MM.bind(this.mouseWheel, this);
	
        this._zoomDiv = document.body.appendChild(document.createElement('div'));
        this._zoomDiv.style.cssText = 'visibility:hidden;top:0;height:0;width:0;overflow-y:scroll';
        var innerDiv = this._zoomDiv.appendChild(document.createElement('div'));
        innerDiv.style.height = '2000px';
        MM.addEvent(map.parent, 'mousewheel', this._mouseWheel);
    },
    
    remove: function() {
        MM.removeEvent(this.map.parent, 'mousewheel', this._mouseWheel);
        this._zoomDiv.parentNode.removeChild(this._zoomDiv);
    },
    
    mouseWheel: function(e) {
        var delta = 0;
        this.prevTime = this.prevTime || new Date().getTime();
	
        try {
            this._zoomDiv.scrollTop = 1000;
            this._zoomDiv.dispatchEvent(e);
            delta = 1000 - this._zoomDiv.scrollTop;
        } catch (error) {
            delta = e.wheelDelta || (-e.detail * 5);
        }
	
        // limit mousewheeling to once every 200ms
        var timeSince = new Date().getTime() - this.prevTime;
	
        if (Math.abs(delta) > 0 && (timeSince > 200) && !this.precise) {
            var point = MM.getMousePoint(e, this.map);
            this.map.zoomBy(delta > 0 ? 1 : -1);

            this.prevTime = new Date().getTime();
        } else if (this.precise) {
            this.map.zoomBy(delta * 0.001);
        }
	
        // Cancel the event so that the page doesn't scroll
        return MM.cancelEvent(e);
    }
};



// Handle double clicks by telling the drone to fly to that location.
DoubleClickHandler = function(map, options) {
    if (map !== undefined) {
        this.init(map, options);
    }
};

DoubleClickHandler.prototype = {

    init: function(map, options) {
        this.map = map;
        this._doubleClick = MM.bind(this.doubleClick, this);
        MM.addEvent(map.parent, 'dblclick', this._doubleClick);

	this.options = {};
    },

    remove: function() {
        MM.removeEvent(this.map.parent, 'dblclick', this._doubleClick);
    },

    doubleClick: function(e) {
        // Ensure that this handler is attached once.
        // Get the point on the map that was double-clicked

	var clickPoint = MM.getMousePoint(e, this.map);
	var target = this.map.pointLocation(clickPoint);
	mmap.flyTo(target);
        return MM.cancelEvent(e);
    }
};

