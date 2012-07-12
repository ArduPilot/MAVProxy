// namespacing!
if (!MM) {
  MM = { };
}

MM.BingProvider = function(key, style, onready) {

    this.key = key;
    this.style = style;
    
    // hit the imagery metadata service
    // http://msdn.microsoft.com/en-us/library/ff701716.aspx
    
    // Aerial, AerialWithLabels, Road
    var script = document.createElement('script');
    script.type = 'text/javascript';
    document.getElementsByTagName('head')[0].appendChild(script);
    script.src = 'http://dev.virtualearth.net/REST/V1/Imagery/Metadata/'+style+'/?key='+key+'&jsonp=onBingComplete';
    
    function toMicrosoft(column, row, zoom) {
        // generate zoom string by interleaving row/col bits
        // NB:- this assumes you're only asking for positive row/cols
        var quadKey = "";
        for (var i = 1; i <= zoom; i++) {
            var rowBit = (row >> zoom-i) & 1;
            var colBit = (column >> zoom-i) & 1;
            quadKey += (rowBit << 1) + colBit;
        }
        return quadKey;
    }

    var provider = this;

    window.onBingComplete = function(data) {
        var resourceSets = data.resourceSets;
        for (var i = 0; i < resourceSets.length; i++) {
            var resources = data.resourceSets[i].resources;
            for (var j = 0; j < resources.length; j++) {
                var resource = resources[j];

                var serverSalt = Math.floor(Math.random() * 4);
                provider.getTile = function(coord) {
                    var quadKey = toMicrosoft(coord.column, coord.row, coord.zoom);
                    // this is so that requests will be consistent in this session, rather than totally random
                    var server = Math.abs(serverSalt + coord.column + coord.row + coord.zoom) % 4;
                    return resource.imageUrl
                        .replace('{quadkey}',quadKey)
                        .replace('{subdomain}', resource.imageUrlSubdomains[server]);
                };
                // TODO: use resource.imageWidth
                // TODO: use resource.imageHeight
            }
        }

        // TODO: display data.brandLogoUri
        // TODO: display data.copyright
        onready(provider);
    };
};

MM.BingProvider.prototype = {
    key: null,
    style: null,
    subdomains: null,
    getTileUrl: null
};

MM.extend(MM.BingProvider, MM.MapProvider);
