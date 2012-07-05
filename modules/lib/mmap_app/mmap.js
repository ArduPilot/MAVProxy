mmap = {};


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
        data: "m -26,0 15,0 3,3 M -1,0 l 2,0 M 26,0 l -15,0 -3,3",
        stroke: "black",
        lineJoin: "round",
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
        data: "m -100,0 200,0",
        stroke: "black",
        lineJoin: "round",
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


var map;
var map_layer;
var marker_layer;
var waypointMarkerElement;
var mapPanner;
var last_state_update_time;
var adi;

var state = {};
state.lat = null;
state.lon = null;
state.pitch = 0.0;
state.roll = 0.0;
state.heading = 0.0;
state.waypoints = null;



function initMap() {

    // Microsoft Bing
    // please use your own API key!  This is jjwiseman's!
    var key = "Anmc0b2q6140lnPvAj5xANM1rvF1A4CvVtr6H2VJvQcdnDvc8NL-I2C49owIe9xC";
    var style = 'AerialWithLabels';
    var provider = new MM.BingProvider(key, style);

    map_layer = new MM.Layer(provider);
    marker_layer = new MM.MarkerLayer();
    eventHandlers = [
	new MouseWheelHandler(),
	new TouchHandler(),
	new DoubleClickHandler()
    ];
    map = new MM.Map('map', map_layer, undefined, eventHandlers);
    map.addLayer(marker_layer);

    map.setCenterZoom(new MM.Location(20.0, 0), 18);

    setInterval(updateState, 250);
    $('#layerpicker').change(updateLayer);

    mapPanner = new MapPanner(map);
    
    adi = new ADI("adi");

    var zoomSlider = document.getElementById('zoom');
    zoomSlider.onchange = function() {
        var sliderProp = (zoomSlider.value - zoomSlider.min) / (zoomSlider.max - zoomSlider.min);
        var targetZoom = sliderProp * 18.0; 
        map.setZoom(targetZoom);
    };
}


function updateLinkStatus() {
    var now = (new Date()).getTime();
    if (!last_state_update_time) {
        $("#t_link").html('<span class="link error">NO</span>');
    } else if (now - last_state_update_time > 5000) {
        $("#t_link").html('<span class="link error">TIMEOUT</span>');
    } else if (now - last_state_update_time > 1000) {
        $("#t_link").html('<span class="link slow">SLOW</span>');
    } else {
        $("#t_link").html('<span class="link ok">OK</span>');
    }
}

function handleMessages(msgs) {
    for (var i = 0; i < msgs.length; i++) {
        handleMessage(msgs[i]);
    }
}

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

function handleHeartbeat(time, index, msg) {
    $('#t_flt_mode').html(mmap.flightModeString(msg));
}

function handleGpsRaw(time, index, msg) {
    $("#t_lat").html(msg.lat.toPrecision(11));
    $("#t_lon").html(msg.lon.toPrecision(11));
}

function handleGpsRawInt(time, index, msg) {
    if (msg.fix_type >= 3) {
        $("#t_gps").html('<span class="ok">OK</span>');
    } else if (msg.fix_type == 2) {
        $("#t_gps").html('<span class="slow">02</span>');
    } else {
        $("#t_gps").html('<span class="error">' + msg.fix_type + '</span>');
    }
    $("#t_lat").html((msg.lat / 1.0e7).toPrecision(11));
    $("#t_lon").html((msg.lon / 1.0e7).toPrecision(11));
}

function handleVfrHud(time, index, msg) {
    $("#t_alt").html(msg.alt.toPrecision(4));
    $("#t_gspd").html(msg.groundspeed.toPrecision(2));
    $("#t_aspd").html(msg.airspeed.toPrecision(2));
    $("#t_hdg").html(msg.heading);
    rotate_drone(msg.heading);
}

function handleAttitude(time, index, msg) {
    adi.setAttitude(msg.pitch, msg.roll);
}

function handleStatusText(time, index, msg) {
    if ((!status_text_seq) || index > status_text_seq) {
        var audioElement = new Audio('drone_chime.mp3');
        audioElement.play();
        $("#t_sta_txt").html(msg.text)
            .stop(true, true)
            .css('color', 'yellow')
            .css('background-color', 'rgb(0, 0, 0, 1.0)')
            .animate({
                color: $.Color("yellow"),
                backgroundColor: $.Color("rgb(0, 0, 0, 1.0)")
            }, {
                duration: 200,
                queue: true
            })
            .animate({
                color: $.Color("white"),
                backgroundColor: $.Color("rgb(0, 0, 0, 0.0)")
            }, {
                duration: 5000,
                queue: true
            });
        status_text_seq = index;
    }
}


var messageHandlerMap = {
    'HEARTBEAT': handleHeartbeat,
    'GPS_RAW': handleGpsRaw,
    'GPS_RAW_INT': handleGpsRawInt,
    'VFR_HUD': handleVfrHud,
    'ATTITUDE': handleAttitude,
    'STATUSTEXT': handleStatusText
};

function handleMessage(msg) {
    handler = messageHandlerMap[msg.msg.mavpackettype];
    handler(msg.time_usec, msg.index, msg.msg);
}


function updateState() {
    msgTypes = Object.keys(messageHandlerMap);
    $.getJSON('mavlink/' + msgTypes.join('+'),
              function(msgs) {
                  handleMessages(msgs);
                  updateMap();
                  last_state_update_time = new Date().getTime();
              });
    updateLinkStatus();
                 
    // $.getJSON("mavlink+heartbeat+",
    //           function(data){
    //               state = data;
    //               updateTelemetryDisplay();
    //               updateMap();
    //               last_state_update_time = new Date().getTime();
    //           });
    // updateLinkStatus();
    // Just to demonstrate the mavlink message api:
    // $.getJSON("mavlink/heartbeat", function(hb){
    //   console.log("num heartbeats: " + hb.index.toString() +
    //               " time_usec: " + hb.time_usec.toString());
    // });
    // // case insensitive. mavlink/gps_raw_int works too.
    // $.getJSON("mavlink/GPS_RAW_INT", function(gps){
    //   console.log("altitude: " +  gps.msg.alt.toString()); 
    // });
}


function updateMap() {
    var location = new MM.Location(state.lat, state.lon);
    if (!last_state_update_time) {
        map.setCenter(location);
    } else {
        mapPanner.setCenter(location);
    }
}


var status_text_seq;
var chimeAudio = new Audio('drone_chime.mp3');
var clientWaypointSeq;

function updateTelemetryDisplay() {
    $("#t_lat").html(state.lat.toPrecision(11));
    $("#t_lon").html(state.lon.toPrecision(11));
    $("#t_alt").html(state.alt.toPrecision(4));
    $("#t_gspd").html(state.groundspeed.toPrecision(2));
    $("#t_aspd").html(state.airspeed.toPrecision(2));
    $("#t_hdg").html(state.heading);
    $("#t_flt_mode").html(state.flight_mode);
    now = new Date().getTime();
    $("#t_fps").html((1000.0 / (now - last_state_update_time)).toPrecision(3));
    if (state.gps_fix_type >= 3) {
        $("#t_gps").html('<span class="ok">OK</span>');
    } else if (state.gps_fix_type == 2) {
        $("#t_gps").html('<span class="slow">02</span>');
    } else if (state.gps_fix_type <= 1) {
        $("#t_gps").html('<span class="error">' + state.gps_fix_type + '</span>');
    }
    if (state.status_text) {
        if ((!status_text_seq) || state.status_text.seq > status_text_seq) {
            var audioElement = new Audio('drone_chime.mp3');
            audioElement.play();
            $("#t_sta_txt").html(state.status_text.text)
		.stop(true, true)
		.css('color', 'yellow')
		.css('background-color', 'rgb(0, 0, 0, 1.0)')
                .animate({
                    color: $.Color("yellow"),
                    backgroundColor: $.Color("rgb(0, 0, 0, 1.0)")
                }, {
                    duration: 200,
                    queue: true
		})
		.animate({
                    color: $.Color("white"),
                    backgroundColor: $.Color("rgb(0, 0, 0, 0.0)")
		}, {
                    duration: 5000,
                    queue: true
		});
            status_text_seq = state.status_text.seq;
	}
    }
    adi.setAttitude(state.pitch, state.roll);
    rotate_drone(state.heading);

    if (state.client_waypoint &&
	(!clientWaypointSeq || clientWaypointSeq < state.client_waypoint_seq)) {
	newWaypoint(state.client_waypoint);
    }
}

function newWaypoint(location) {
    if (!waypointMarkerElement) {
	waypointMarkerElement = document.createElement('div');
	waypointMarkerElement.innerHTML = '<img src="mapmarker.png" width="50" height="50">';
	waypointMarkerElement.pixelOffset = {x: -25, y: -50};
	marker_layer.addMarker(waypointMarkerElement, location);
    } else {
	waypointMarkerElement.location = location;
	waypointMarkerElement.coord = map.locationCoordinate(location);
	marker_layer.repositionMarker(waypointMarkerElement);
    }
}




function rotate_drone(deg){
    var rotate = "rotate(" + (deg) + "deg);";
    var tr = new Array(
        "transform:" + rotate,
        "-moz-transform:" + rotate,
        "-webkit-transform:" + rotate,
        "-ms-transform:" + rotate,
        "-o-transform:" + rotate
    );

    var drone = document.getElementById("drone");
    drone.setAttribute("style", tr.join(";"));
}


function updateLayer() {
    var provider;
    var layerNum = $(this).attr('value');
    console.log("Switching to layer " + layerNum);
    var bing_key = "Anmc0b2q6140lnPvAj5xANM1rvF1A4CvVtr6H2VJvQcdnDvc8NL-I2C49owIe9xC";
    var style;
    if (layerNum == '1') {
        style = 'AerialWithLabels';
        provider = new MM.BingProvider(bing_key, style, function(provider) { map_layer.setProvider(provider); });
    } else if (layerNum == '2') {
        style = 'BirdseyeWithLabels';
        provider = new MM.BingProvider(bing_key, style, function(provider) { map_layer.setProvider(provider); });
    } else if (layerNum == '3') {
        style = 'Road';
        provider = new MM.BingProvider(bing_key, style, function(provider) { map_layer.setProvider(provider); });
    } else if (layerNum == '4') {
        provider = new MM.BlueMarbleProvider();
        map_layer.setProvider(provider);
    }
}
