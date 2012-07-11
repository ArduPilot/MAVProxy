mmap = {};

mmap.location = null;
mmap.map = null;
mmap.mapLayer = null;
mmap.markerLayer = null;
mmap.waypointMarkerElement = null;
mmap.mapPanner = null;
mmap.lastMessageHandledTime = null;
mmap.adi = null;
mmap.statusTextSeq = null;
mmap.chimeAudio = new Audio('drone_chime.mp3');
mmap.clientWaypointSeq = null;
mmap._alt = null;

function MapPanner(map) {
    var theMapPanner = this;
    this.map = map;
    this.targetLocation = null;
    
    this.animateToCenter = function() {
        var need_more_animation = false;
        var curLocation = theMapPanner.map.getCenter();
        var nextLat = curLocation.lat + (theMapPanner.targetLocation.lat - curLocation.lat) * 0.2;
        var nextLon = curLocation.lon + (theMapPanner.targetLocation.lon - curLocation.lon) * 0.2;
        if (Math.abs(nextLat - curLocation.lat) < 0.00001) {
            nextLat = theMapPanner.targetLocation.lat;
        } else {
            need_more_animation = true;
        }
        if (Math.abs(nextLon - curLocation.lon) < 0.00001) {
            nextLon = theMapPanner.targetLocation.lon;
        } else {
            need_more_animation = true;
        }
        theMapPanner.map.setCenter(new MM.Location(nextLat, nextLon));
        if (need_more_animation) {
            MM.getFrame(this.animateToCenter);
        }
    };

    this.setCenter = function(location) {
        this.targetLocation = location;
        MM.getFrame(this.animateToCenter);
    };
}


function ADI(container) {
    var theADI = this;

    this.pitch = -90.0 * Math.PI / 180.0;
    this.roll = 90.0 * Math.PI / 180.0;
    this.targetPitch = 0.0;
    this.targetRoll = 0.0;

    var containerElement = document.getElementById(container);
    this.stage = new Kinetic.Stage({
        container: container,
        height: containerElement.offsetHeight,
        width: containerElement.offsetWidth
    });
    this.stage.setScale(containerElement.offsetWidth / 100.0,
                        containerElement.offsetHeight / 100.0);
    this.layer = new Kinetic.Layer();
    this.plane = new Kinetic.Path({
        x: 50,
        y: 50,
        data: 'm -26,0 15,0 3,3 M -1,0 l 2,0 M 26,0 l -15,0 -3,3',
        stroke: 'black',
        lineJoin: 'round',
        strokeWidth: 2.5,
        shadow: {
            color: 'black',
            blur: 5,
            offset: [1, 1],
            alpha: 0.5
        },
        scale: 1.2
    });
    // add the shape to the layer
    this.layer.add(this.plane);
    this.horizon = new Kinetic.Path({
        x: 50,
        y: 50,
        data: 'm -100,0 200,0',
        stroke: 'black',
        lineJoin: 'round',
        strokeWidth: 1.7,
        scale: 1
    });
    this.layer.add(this.horizon);
    // add the layer to the stage
    this.stage.add(this.layer);

    this.drawFromAttitude = function(pitch, roll) {
        var horizon_y = 50.0 + 50.0 * Math.sin(pitch);
        theADI.horizon.setY(horizon_y);
        theADI.horizon.setRotation(-roll);
        theADI.layer.draw();
    };

    this.drawFromAttitude(this.pitch, this.roll);
    
    this.animateToAttitude = function() {
        var need_more_animation = false;
        
        var curPitch = theADI.pitch;
        var nextPitch = curPitch + (theADI.targetPitch - curPitch) * 0.05;
        if (Math.abs(nextPitch - curPitch) < 0.001) {
            nextPitch = theADI.targetPitch;
        } else {
            need_more_animation = true;
        }
        
        var curRoll = theADI.roll;
        var nextRoll = curRoll + (theADI.targetRoll - curRoll) * 0.05;
        if (Math.abs(nextRoll - curRoll) < 0.001) {
            nextRoll = theADI.targetRoll;
        } else {
            need_more_animation = true;
        }
        
        theADI.drawFromAttitude(nextPitch, nextRoll);
        
        theADI.pitch = nextPitch;
        theADI.roll = nextRoll;
        if (need_more_animation) {
            MM.getFrame(theADI.animateToAttitude);
        }
    };
    
    this.setAttitude = function(pitch, roll) {
        this.targetPitch = pitch;
        this.targetRoll = roll;
        MM.getFrame(this.animateToAttitude);
    };
}


mmap.initMap = function() {
    // Microsoft Bing
    // please use your own API key!  This is jjwiseman's!
    var key = 'Anmc0b2q6140lnPvAj5xANM1rvF1A4CvVtr6H2VJvQcdnDvc8NL-I2C49owIe9xC';
    var style = 'AerialWithLabels';
    var provider = new MM.BingProvider(key, style);

    mmap.mapLayer = new MM.Layer(provider);
    mmap.markerLayer = new MM.MarkerLayer();
    var eventHandlers = [
	new MouseWheelHandler(),
	new TouchHandler(),
	new DoubleClickHandler()
    ];
    mmap.map = new MM.Map('map', mmap.mapLayer, undefined, eventHandlers);
    mmap.map.addLayer(mmap.markerLayer);

    mmap.map.setCenterZoom(new MM.Location(20.0, 0), 18);

    setInterval(mmap.updateState, 250);
    $('#layerpicker').change(mmap.updateLayer);

    mmap.mapPanner = new MapPanner(mmap.map);
    
    mmap.adi = new ADI('adi');

    var zoomSlider = document.getElementById('zoom');
    zoomSlider.onchange = function() {
        var sliderProp = (zoomSlider.value - zoomSlider.min) / (zoomSlider.max - zoomSlider.min);
        var targetZoom = sliderProp * 18.0; 
        mmap.map.setZoom(targetZoom);
    };

    mmap.altSlider = document.getElementById('altinput');
    mmap.setAlt(mmap.altSlider.value, false);
    mmap.altSlider.onchange = function () {
      mmap.setAlt(mmap.altSlider.value, false);
    };

    document.getElementById('altinput_submit').onclick = function () {
      if (mmap.lastFlyTo){
        mmap.flyTo(mmap.lastFlyTo);
      }
    }
};


mmap.updateLinkStatus = function() {
    var now = (new Date()).getTime();
    if (!mmap.lastMessageHandledTime) {
        $('#t_link').html('<span class="link error">NO</span>');
    } else if (now - mmap.lastMessageHandledTime > 5000) {
        $('#t_link').html('<span class="link error">TIMEOUT</span>');
    } else if (now - mmap.lastMessageHandledTime > 1000) {
        $('#t_link').html('<span class="link slow">SLOW</span>');
    } else {
        $('#t_link').html('<span class="link ok">OK</span>');
    }
};


mmap.arduPlaneFlightModes = {
    0: 'MANUAL',
    1: 'CIRCLE',
    2: 'STABILIZE',
    5: 'FBWA',
    6: 'FBWB',
    7: 'FBWC',
    10: 'AUTO',
    11: 'RTL',
    12: 'LOITER',
    13: 'TAKEOFF',
    14: 'LAND',
    15: 'GUIDED',
    16: 'INITIALIZING'
};

mmap.arduCopterFlightModes = {
    0: 'STABILIZE',
    1: 'ACRO',
    2: 'ALT_HOLD',
    3: 'AUTO',
    4: 'GUIDED',
    5: 'LOITER',
    6: 'RTL',
    7: 'CIRCLE',
    8: 'POSITION',
    9: 'LAND',
    10: 'OF_LOITER',
    11: 'APPROACH'
};

mmap.MAV_TYPE_QUADROTOR = 2;
mmap.MAV_TYPE_FIXED_WING = 1;
mmap.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1;


mmap.flightModeString = function(msg) {
    var mode;
    if (!msg.base_mode & mmap.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
        mode = 'Mode(' + msg.base_mode + ')';
    } else if (msg.type == mmap.MAV_TYPE_QUADROTOR &&
               msg.custom_mode in mmap.arduCopterFlightModes) {
        mode = mmap.arduCopterFlightModes[msg.custom_mode];
    } else if (msg.type == mmap.MAV_TYPE_FIXED_WING &&
               msg.custom_mode in mmap.arduPlaneFlightModes) {
        mode = mmap.arduPlaneFlightModes[msg.custom_mode];
    } else {
        mode = 'Mode(' + msg.custom_mode + ')';
    }
    return mode;
};


mmap.handleHeartbeat = function(time, index, msg) {
    $('#t_flt_mode').html(mmap.flightModeString(msg));
};


mmap.handleGpsRaw = function(time, index, msg) {
    $('#t_lat').html(msg.lat.toPrecision(11));
    $('#t_lon').html(msg.lon.toPrecision(11));
    mmap.location = {lat: msg.lat, lon: msg.lon};
};


mmap.handleGpsRawInt = function(time, index, msg) {
    if (msg.fix_type >= 3) {
        $('#t_gps').html('<span class="ok">OK</span>');
    } else if (msg.fix_type == 2) {
        $('#t_gps').html('<span class="slow">02</span>');
    } else {
        $('#t_gps').html('<span class="error">' + msg.fix_type + '</span>');
    }
    var lat = msg.lat / 1.0e7;
    var lon = msg.lon / 1.0e7;
    $('#t_lat').html(lat.toPrecision(11));
    $('#t_lon').html(lon.toPrecision(11));
    mmap.location = {lat: lat, lon: lon};
};


mmap.updateMap = function() {
    var location = new MM.Location(mmap.location.lat, mmap.location.lon);
    if (!mmap.lastMessageHandledTime) {
        mmap.map.setCenter(location);
    } else {
        mmap.mapPanner.setCenter(location);
    }
};


mmap.handleVfrHud = function(time, index, msg) {
    $('#t_alt').html(msg.alt.toPrecision(4));
    $('#t_gspd').html(msg.groundspeed.toPrecision(2));
    $('#t_aspd').html(msg.airspeed.toPrecision(2));
    $('#t_hdg').html(msg.heading);
    mmap.rotateDrone(msg.heading);
};


mmap.handleAttitude = function(time, index, msg) {
    mmap.adi.setAttitude(msg.pitch, msg.roll);
};


mmap.handleMetaWaypoint = function(time, index, msg) {
    if (!mmap.clientWaypointSeq || mmap.clientWaypointSeq < index) {
	mmap.clientWaypointSeq = index;
	mmap.newWaypoint(msg.waypoint);
    }
};

mmap.handleStatusText = function(time, index, msg) {
    if ((!mmap.statusTextSeq) || index > mmap.statusTextSeq) {
        var audioElement = new Audio('drone_chime.mp3');
        audioElement.play();
        $('#t_sta_txt').html(msg.text)
            .stop(true, true)
            .css('color', 'yellow')
            .css('background-color', 'rgb(0, 0, 0, 1.0)')
            .animate({
                color: $.Color('yellow'),
                backgroundColor: $.Color('rgb(0, 0, 0, 1.0)')
            }, {
                duration: 200,
                queue: true
            })
            .animate({
                color: $.Color('white'),
                backgroundColor: $.Color('rgb(0, 0, 0, 0.0)')
            }, {
                duration: 5000,
                queue: true
            });
        mmap.statusTextSeq = index;
    }
};


mmap.setAlt = function(newalt, updateslider) {
  if (updateslider) {
    mmap.altSlider.value = newalt;
  }
  mmap._alt = newalt;
  $('#v_altwaypt').html(newalt.toString())
}

mmap.getAlt = function() { 
  return mmap._alt;
}

mmap.messageHandlerMap = {
    'HEARTBEAT': mmap.handleHeartbeat,
    'GPS_RAW': mmap.handleGpsRaw,
    'GPS_RAW_INT': mmap.handleGpsRawInt,
    'VFR_HUD': mmap.handleVfrHud,
    'ATTITUDE': mmap.handleAttitude,
    'STATUSTEXT': mmap.handleStatusText,
    'META_WAYPOINT': mmap.handleMetaWaypoint
};


mmap.handleMessages = function(msgs) {
    for (var i = 0; i < msgs.length; i++) {
        mmap.handleMessage(msgs[i]);
    }
};


mmap.handleMessage = function(msg) {
    var handler = mmap.messageHandlerMap[msg.msg.mavpackettype];
    if (handler) {
        handler(msg.time_usec, msg.index, msg.msg);
    } else {
        console.warn(
            'No handler defined for message type ' + msg.msg.mavpackettype);
    }
};


mmap.updateState = function() {
    var msgTypes = Object.keys(mmap.messageHandlerMap);
    $.getJSON('mavlink/' + msgTypes.join('+'),
              function(msgs) {
                  mmap.handleMessages(msgs);
                  mmap.updateMap();
                  mmap.lastMessageHandledTime = new Date().getTime();
              });
    mmap.updateLinkStatus();
};


mmap.newWaypoint = function(location) {
    if (!mmap.waypointMarkerElement) {
	mmap.waypointMarkerElement = document.createElement('div');
	mmap.waypointMarkerElement.innerHTML = '<img src="mapmarker.png" width="50" height="50">';
	mmap.waypointMarkerElement.pixelOffset = {x: -25, y: -50};
	mmap.markerLayer.addMarker(mmap.waypointMarkerElement, location);
    } else {
	mmap.waypointMarkerElement.location = location;
	mmap.waypointMarkerElement.coord = mmap.map.locationCoordinate(location);
	mmap.markerLayer.repositionMarker(mmap.waypointMarkerElement);
    }
  mmap.setAlt(location.alt, true);
  mmap.lastFlyTo = location; 
};


mmap.rotateDrone = function(deg){
    var rotate = 'rotate(' + (deg) + 'deg);';
    var tr = new Array(
        'transform:' + rotate,
        '-moz-transform:' + rotate,
        '-webkit-transform:' + rotate,
        '-ms-transform:' + rotate,
        '-o-transform:' + rotate
    );
    var drone = document.getElementById('drone');
    drone.setAttribute('style', tr.join(';'));
};


mmap.updateMapLayer = function() {
    var provider;
    var layerNum = $(this).attr('value');
    var bing_key = 'Anmc0b2q6140lnPvAj5xANM1rvF1A4CvVtr6H2VJvQcdnDvc8NL-I2C49owIe9xC';
    var style;
    if (layerNum == '1') {
        style = 'AerialWithLabels';
        provider = new MM.BingProvider(bing_key, style,
                                       function(provider) {
                                           mmap.mapLayer.setProvider(provider);
                                       });
    } else if (layerNum == '2') {
        style = 'BirdseyeWithLabels';
        provider = new MM.BingProvider(bing_key, style,
                                       function(provider) {
                                           mmap.mapLayer.setProvider(provider);
                                       });
    } else if (layerNum == '3') {
        style = 'Road';
        provider = new MM.BingProvider(bing_key, style,
                                       function(provider) {
                                           mmap.mapLayer.setProvider(provider);
                                       });
    } else if (layerNum == '4') {
        provider = new MM.BlueMarbleProvider();
        mmap.mapLayer.setProvider(provider);
    }
};


mmap.lastFlyTo = null;
mmap.flyTo = function(location) {
  var loc = {lat: location.lat
            , lon: location.lon
            , alt: mmap.getAlt() };
  mmap.lastFlyTo = loc;
    $.ajax({
        type: 'POST',
        url: '/command',
        data: JSON.stringify({command: 'FLYTO',
                              location: loc })
    });
};
