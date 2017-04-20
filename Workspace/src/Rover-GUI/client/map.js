lati = 43.46;
longi = -80.52;
markers = [];
var map;

	var roboIcon = L.Icon.extend({
		options: {
			iconSize:     [100, 100],
			iconAnchor:   [50, 93],
			popupAnchor:  [0, -73]
		}
	});

	var roverIcon = new roboIcon({iconUrl: 'roverIcon.png'});

function updatePos(){
	//lati +=0.00001;
	//longi +=0.00001;
	if(document.getElementById("followBox").checked){
		map.setView([lati, longi], 16);
	}
}

function changePos(marker){
	updatePos();
	marker.setLatLng(new L.LatLng(lati, longi));
}

function init(){
	// initialize the map
	map = L.map('map').setView([lati, longi], 13);
	// load a tile layer
	//L.tileLayer('http://{s}.tile.osm.org/{z}/{x}/{y}.png?').addTo(map);
	L.tileLayer('Tiles/{z}/{x}/{y}.png?{foo}', {foo: 'bar'}).addTo(map);
	var marker = L.marker([lati, longi], {icon: roverIcon}).addTo(map);
	var intervalID = setInterval(function(){changePos(marker);}, 100);
	return map;
}

function addMarker(){
	var lat = document.getElementById("LatF").value;
	var long = document.getElementById("LongF").value;
	markers.push(L.marker([lat,long]).addTo(map).bindPopup(lat+", "+long).openPopup());
}
