MM.BlueMarbleProvider = function() {
    MM.MapProvider.call(this, function(coordinate) {
        coordinate = this.sourceCoordinate(coordinate);
        if (!coordinate) return null;
        var img = coordinate.zoom.toFixed(0) +'-r'+ coordinate.row.toFixed(0) +'-c'+ coordinate.column.toFixed(0) + '.jpg';
        return 'http://s3.amazonaws.com/com.modestmaps.bluemarble/' + img;
    });
};

com.modestmaps.extend(MM.BlueMarbleProvider, MM.MapProvider);
