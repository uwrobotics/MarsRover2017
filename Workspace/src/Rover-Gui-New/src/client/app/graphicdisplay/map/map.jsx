import React from 'react';

import L from './leaflet/leaflet.js';


// Force map to buffer past the edges
L.EdgeBuffer = {
    previousMethods: {
      getTiledPixelBounds: L.GridLayer.prototype._getTiledPixelBounds
    }
};
L.GridLayer.include({

    _getTiledPixelBounds : function(center, zoom, tileZoom) {
      var pixelBounds = L.EdgeBuffer.previousMethods.getTiledPixelBounds.call(this, center, zoom, tileZoom);

      // Default is to buffer one tiles beyond the pixel bounds (edgeBufferTiles = 1).
      var edgeBufferTiles = 1;
      if ((this.options.edgeBufferTiles !== undefined) && (this.options.edgeBufferTiles !== null)) {
        edgeBufferTiles = this.options.edgeBufferTiles;
      }

      if (edgeBufferTiles > 0) {
        var pixelEdgeBuffer = edgeBufferTiles * this.options.tileSize;
        pixelBounds = new L.Bounds(pixelBounds.min.subtract([pixelEdgeBuffer, pixelEdgeBuffer]), pixelBounds.max.add([pixelEdgeBuffer, pixelEdgeBuffer]));
      }
      return pixelBounds;
    }
});

// Allow markers to be rotated
(function() {
    // save these original methods before they are overwritten
    var proto_initIcon = L.Marker.prototype._initIcon;
    var proto_setPos = L.Marker.prototype._setPos;
    var proto_onDrag = L.Handler.MarkerDrag.prototype._onDrag;

    var oldIE = (L.DomUtil.TRANSFORM === 'msTransform');

    L.Marker.addInitHook(function () {
        var iconOptions = this.options.icon && this.options.icon.options;
        var iconAnchor = iconOptions && this.options.icon.options.iconAnchor;
        if (iconAnchor) {
            iconAnchor = (iconAnchor[0] + 'px ' + iconAnchor[1] + 'px');
        }
        this.options.rotationOrigin = this.options.rotationOrigin || iconAnchor || 'center bottom' ;
        this.options.rotationAngle = this.options.rotationAngle || 0;
    });

    L.Marker.include({
        _initIcon: function() {
            proto_initIcon.call(this);
        },

        _setPos: function (pos) {
            proto_setPos.call(this, pos);
            this._applyRotation();
        },

        _applyRotation: function () {
            if(this.options.rotationAngle) {
                this._icon.style[L.DomUtil.TRANSFORM+'Origin'] = this.options.rotationOrigin;

                if(oldIE) {
                    // for IE 9, use the 2D rotation
                    this._icon.style[L.DomUtil.TRANSFORM] = 'rotate(' + this.options.rotationAngle + 'deg)';
                } else {
                    // for modern browsers, prefer the 3D accelerated version
                    this._icon.style[L.DomUtil.TRANSFORM] += ' rotateZ(' + this.options.rotationAngle + 'deg)';
                }
            }
        },

        setRotationAngle: function(angle) {
            this.options.rotationAngle = angle;
            this.update();
            return this;
        },

        setRotationOrigin: function(origin) {
            this.options.rotationOrigin = origin;
            this.update();
            return this;
        }
    });

    L.Handler.MarkerDrag.include({
        _onDrag: function (e) {
            proto_onDrag.call(this, e);
            this._marker._applyRotation();
        }
    })
})();



class MapBox extends React.Component {
    
    constructor(props) {
        super(props);
        this.state = {
            map: null,
            robotMarker: null,
            markers: [],
            lat: 38.405905,
            long: -110.792088,
            angle: 0,
            zoom: 16,
            markerIcon: null,
        }
    };
    
    initMap() {
        
        let roboIcon = L.Icon.extend({
            options: {
                iconSize:     [70, 70],
                iconAnchor:   [36, 46],
                popupAnchor:  [0, -73]
            }
        });

        let markIcon = L.Icon.extend({
            options: {
                iconSize:     [30, 50],
                iconAnchor:   [36, 46],
                popupAnchor:  [0, -73]
            }
        });
        
        let roverIcon = new roboIcon({iconUrl: 'app/roverIcon2.png'});
        this.state.markerIcon = new markIcon({iconUrl: 'app/graphicdisplay/map/leaflet/images/marker-icon.png'});
        
        this.state.map = L.map(this.refs.map).setView([this.state.lat, this.state.long], this.state.zoom);
        var map = this.state.map
        
        L.tileLayer('app/Tiles/{z}/{x}/{y}.png', {edgeBufferTiles: 2}).addTo(map);
        L.control.scale().addTo(map);

        let robotMarker = L.marker([this.state.lat, this.state.long], {icon: roverIcon, rotationAngle: this.state.angle}).addTo(map);
        map.setMinZoom(12);
        map.setMaxZoom(18);
        this.setState({
            map,
            robotMarker
        });
    }
    
    componentDidMount() {
        this.initMap();
    }
    
    componentWillUpdate() {
        if (this.state.robotMarker) {
            if (this.refs.follow.checked) {
                this.state.map.setView([this.state.lat, this.state.long], this.state.zoom);
            }
            this.state.robotMarker.setLatLng(new L.LatLng(this.state.lat, this.state.long));
            this.state.robotMarker.setRotationAngle(this.state.angle);
        }
    }
    
    componentWillReceiveProps(nextProps) {
        this.setState({
            lat: nextProps.lat,
            long: nextProps.lon,
            angle: nextProps.dir
        })
    }
    
    addMarker() {
        const lat = this.refs.LatF.value;
        const long = this.refs.LongF.value;
        let markers = this.state.markers;
        markers.push(L.marker([lat,long], {icon:this.state.markerIcon}).addTo(this.state.map).bindPopup(lat+", "+long).openPopup());
        this.setState({ markers });
    }

    render() {
        let style = {width: '700px', height: '400px'};
        return (
            <div className='mapcontainer'>
                <div className='map' ref='map' style={style}></div>
                <div className='mapsettings'>
                    <div className='section'>
                        <p>Lat:</p>
                        <input type="text" ref="LatF" defaultValue="38.405905" />
                        <p>Long:</p>
                        <input type="text" ref="LongF" defaultValue="-110.792088" />
                        <button onClick={this.addMarker.bind(this)}>Add Marker</button>
                    </div>
                    <div className='section'>
                        <p>Follow Rover?</p>
                        <input type="checkbox" ref="follow" />
                    </div>
                </div>
            </div>
        );
    };
    
}

export default MapBox;
