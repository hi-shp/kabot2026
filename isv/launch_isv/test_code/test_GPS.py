#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import threading
from flask import Flask, render_template_string, jsonify
import yaml
import os

app = Flask(__name__)
# 발급받은 Google Maps API 키를 사용합니다.
GOOGLE_MAPS_API_KEY = "AIzaSyDoIwjXsVxvJy0GoNWK8Bf1UjDGktbO1o4"

current_pos = {"lat": 0.0, "lon": 0.0}

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Google Satellite Mission Monitor</title>
    <script src="https://maps.googleapis.com/maps/api/js?key={{ key }}"></script>
    <style>
        #map { height: 100vh; width: 100%; }
        body { margin: 0; padding: 0; background: #000; }
        .status-ui { 
            position: absolute; 
            top: 15px; 
            right: 15px; 
            z-index: 10; 
            background: rgba(255,255,255,0.9); 
            padding: 15px; 
            border-radius: 8px; 
            box-shadow: 0 2px 10px rgba(0,0,0,0.3); 
            font-family: sans-serif;
            min-width: 180px;
        }
    </style>
</head>
<body>
    <div class="status-ui">
        <b style="color: #4285F4;">🚢 KABOT 2026</b><br>
        <small id="gps-stat">GPS 대기 중...</small><hr>
        LAT: <span id="lat">0.0</span><br>
        LON: <span id="lon">0.0</span><br>
        <small style="color: #666;">(클릭 시 좌표 복사)</small>
    </div>
    <div id="map"></div>

    <script>
        let map, boatMarker;
        let isFirst = true;

        function initMap() {
            map = new google.maps.Map(document.getElementById("map"), {
                zoom: 18,
                center: { lat: 35.2316, lng: 129.0825 },
                mapTypeId: 'satellite',
                tilt: 0
            });

            // 지도 클릭 이벤트: 알림 없이 클립보드 복사만 수행
            map.addListener("click", (mapsMouseEvent) => {
                const lat = mapsMouseEvent.latLng.lat().toFixed(7);
                const lng = mapsMouseEvent.latLng.lng().toFixed(7);
                const coordText = `${lat}, ${lng}`;
                
                // 클립보드 복사 (백그라운드에서 실행)
                navigator.clipboard.writeText(coordText);
            });

            boatMarker = new google.maps.Marker({
                position: { lat: 0, lng: 0 },
                map: map,
                icon: {
                    path: google.maps.SymbolPath.CIRCLE,
                    scale: 8,
                    fillColor: "#EA4335",
                    fillOpacity: 1,
                    strokeWeight: 2,
                    strokeColor: "white"
                }
            });

            fetch('/waypoints').then(r => r.json()).then(wps => {
                wps.forEach((wp, i) => {
                    new google.maps.Marker({
                        position: { lat: wp.lat, lng: wp.lon },
                        map: map,
                        label: { text: i.toString(), color: 'white' },
                        title: "Mission Point " + i
                    });
                });
            });
        }

        setInterval(() => {
            fetch('/data').then(r => r.json()).then(data => {
                if (data.lat === 0) return;
                const pos = { lat: data.lat, lng: data.lon };
                document.getElementById('lat').innerText = data.lat.toFixed(7);
                document.getElementById('lon').innerText = data.lon.toFixed(7);
                document.getElementById('gps-stat').innerText = "정상 수신 중";

                boatMarker.setPosition(pos);
                if (isFirst) {
                    map.setCenter(pos);
                    map.setZoom(20);
                    isFirst = false;
                }
            });
        }, 1000);

        window.onload = initMap;
    </script>
</body>
</html>
"""

@app.route('/')
def index(): 
    return render_template_string(HTML_TEMPLATE, key=GOOGLE_MAPS_API_KEY)

@app.route('/data')
def data(): 
    return jsonify(current_pos)

@app.route('/waypoints')
def get_waypoints():
    try:
        # isv_params.yaml 파일을 읽어 waypoints 정보를 가져옵니다.
        with open('isv_params.yaml', 'r', encoding='utf-8') as f:
            params = yaml.safe_load(f)
        return jsonify(params['navigation']['waypoints']) 
    except Exception as e:
        print(f"YAML Load Error: {e}")
        return jsonify([])

class GPSMapNode(Node):
    def __init__(self):
        super().__init__('google_map_viewer')
        self.create_subscription(NavSatFix, "/gps/fix", self.gps_cb, 10) 

    def gps_cb(self, msg):
        if msg.latitude == msg.latitude:
            current_pos["lat"] = msg.latitude
            current_pos["lon"] = msg.longitude

def main():
    rclpy.init()
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000), daemon=True).start()
    rclpy.spin(GPSMapNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()