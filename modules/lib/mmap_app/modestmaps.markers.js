// namespacing!
if (!com) {
    var com = {};
}
if (!com.modestmaps) {
    com.modestmaps = {};
}

(function(MM) {

    /**
     * The MarkerLayer doesn't do any tile stuff, so it doesn't need to
     * inherit from MM.Layer. The constructor takes only an optional parent
     * element.
     *
     * Usage:
     *
     * // create the map with some constructor parameters
     * var map = new MM.Map(...);
     * // create a new MarkerLayer instance and add it to the map
     * var layer = new MM.MarkerLayer();
     * map.addLayer(layer);
     * // create a marker element
     * var marker = document.createElement("a");
     * marker.innerHTML = "Stamen";
     * // add it to the layer at the specified geographic location
     * layer.addMarker(marker, new MM.Location(37.764, -122.419));
     * // center the map on the marker's location
     * map.setCenterZoom(marker.location, 13);
     *
     */
    MM.MarkerLayer = function(parent) {
        this.parent = parent || document.createElement('div');
        this.parent.style.cssText = 'position: absolute; top: 0px; left: 0px; width: 100%; height: 100%; margin: 0; padding: 0; z-index: 0';
        this.markers = [];
        this.resetPosition();
    };

    MM.MarkerLayer.prototype = {
        // a list of our markers
        markers: null,
        // the absolute position of the parent element
        position: null,

        // the last coordinate we saw on the map
        lastCoord: null,
        draw: function() {
            // these are our previous and next map center coordinates
            var prev = this.lastCoord,
                next = this.map.pointCoordinate({x: 0, y: 0});
            // if we've recorded the map's previous center...
            if (prev) {
                // if the zoom hasn't changed, find the delta in screen
                // coordinates and pan the parent element
                if (prev.zoom == next.zoom) {
                    var p1 = this.map.coordinatePoint(prev),
                        p2 = this.map.coordinatePoint(next),
                        dx = p1.x - p2.x,
                        dy = p1.y - p2.y;
                    // console.log("panned:", [dx, dy]);
                    this.onPanned(dx, dy);
                // otherwise, reposition all the markers
                } else {
                    this.onZoomed();
                }
            // otherwise, reposition all the markers
            } else {
                this.onZoomed();
            }
            // remember the previous center
            this.lastCoord = next.copy();
        },

        // when zoomed, reset the position and reposition all markers
        onZoomed: function() {
            this.resetPosition();
            this.repositionAllMarkers();
        },

        // when panned, offset the position by the provided screen coordinate x
        // and y values
        onPanned: function(dx, dy) {
            this.position.x += dx;
            this.position.y += dy;
            this.parent.style.left = ~~(this.position.x + .5) + "px";
            this.parent.style.top = ~~(this.position.y + .5) + "px";
        },

        // remove all markers
        removeAllMarkers: function() {
            while (this.markers.length > 0) {
                this.removeMarker(this.markers[0]);
            }
        },

        /**
         * Coerce the provided value into a Location instance. The following
         * values are supported:
         *
         * 1. MM.Location instances
         * 2. Object literals with numeric "lat" and "lon" properties
         * 3. A string in the form "lat,lon"
         * 4. GeoJSON objects with "Point" geometries
         *
         * This function throws an exception on error.
         */
        coerceLocation: function(feature) {
            switch (typeof feature) {
                case "string":
                    // "lat,lon" string
                    return MM.Location.fromString(feature);

                case "object":
                    // GeoJSON
                    if (typeof feature.geometry === "object") {
                        var geom = feature.geometry;
                        switch (geom.type) {
                            // Point geometries => MM.Location
                            case "Point":
                                // coerce the lat and lon values, just in case
                                var lon = Number(geom.coordinates[0]),
                                    lat = Number(geom.coordinates[1]);
                                return new MM.Location(lat, lon);
                        }
                        throw 'Unable to get the location of GeoJSON "' + geom.type + '" geometry!';
                    } else if (feature instanceof MM.Location ||
                        (typeof feature.lat !== "undefined" && typeof feature.lon !== "undefined")) {
                        return feature;
                    } else {
                        throw 'Unknown location object; no "lat" and "lon" properties found!';
                    }
                    break;

                case "undefined":
                    throw 'Location "undefined"';
            }
        },

        /**
         * Add an HTML element as a marker, located at the position of the
         * provided GeoJSON feature, Location instance (or {lat,lon} object
         * literal), or "lat,lon" string.
         */
        addMarker: function(marker, feature) {
            if (!marker || !feature) {
                return null;
            }
            // convert the feature to a Location instance
            marker.location = this.coerceLocation(feature);
            // remember the tile coordinate so we don't have to reproject every time
            marker.coord = this.map.locationCoordinate(marker.location);
            // position: absolute
            marker.style.position = "absolute";
            // update the marker's position
            this.repositionMarker(marker);
            // append it to the DOM
            this.parent.appendChild(marker);
            // add it to the list
            this.markers.push(marker);
            return marker;
        },

        /**
         * Remove the element marker from the layer and the DOM.
         */
        removeMarker: function(marker) {
            var index = this.markers.indexOf(marker);
            if (index > -1) {
                this.markers.splice(index, 1);
            }
            if (marker.parentNode == this.parent) {
                this.parent.removeChild(marker);
            }
            return marker;
        },

        // reset the absolute position of the layer's parent element
        resetPosition: function() {
            this.position = new MM.Point(0, 0);
            this.parent.style.left = this.parent.style.top = "0px";
        },

        // reposition a single marker element
        repositionMarker: function(marker) {
            if (marker.coord) {
                var pos = this.map.coordinatePoint(marker.coord);
                // offset by the layer parent position if x or y is non-zero
                if (this.position.x || this.position.y) {
                    pos.x -= this.position.x;
                    pos.y -= this.position.y;
                }
		if (marker.pixelOffset) {
                    pos.x += marker.pixelOffset.x;
                    pos.y += marker.pixelOffset.y;
		}
                marker.style.left = ~~(pos.x + .5) + "px";
                marker.style.top = ~~(pos.y + .5) + "px";
            } else {
                // TODO: throw an error?
            }
        },

        // Reposition al markers
        repositionAllMarkers: function() {
            var len = this.markers.length;
            for (var i = 0; i < len; i++) {
                this.repositionMarker(this.markers[i]);
            }
        }
    };

    // Array.indexOf polyfill courtesy of Mozilla MDN:
    // https://developer.mozilla.org/en/JavaScript/Reference/Global_Objects/Array/indexOf
    if (!Array.prototype.indexOf) {
        Array.prototype.indexOf = function (searchElement /*, fromIndex */ ) {
            "use strict";
            if (this === void 0 || this === null) {
                throw new TypeError();
            }
            var t = Object(this);
            var len = t.length >>> 0;
            if (len === 0) {
                return -1;
            }
            var n = 0;
            if (arguments.length > 0) {
                n = Number(arguments[1]);
                if (n !== n) { // shortcut for verifying if it's NaN
                    n = 0;
                } else if (n !== 0 && n !== Infinity && n !== -Infinity) {
                    n = (n > 0 || -1) * Math.floor(Math.abs(n));
                }
            }
            if (n >= len) {
                return -1;
            }
            var k = n >= 0 ? n : Math.max(len - Math.abs(n), 0);
            for (; k < len; k++) {
                if (k in t && t[k] === searchElement) {
                    return k;
                }
            }
            return -1;
        }
    }

})(com.modestmaps);
