#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import threading
from flask import Flask, render_template_string, jsonify
import yaml
import os
import sys

app = Flask(__name__)
GOOGLE_MAPS_API_KEY = "AIzaSyDoIwjXsVxvJy0GoNWK8Bf1UjDGktbO1o4"

current_pos = {"lat": 0.0, "lon": 0.0}

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>KABOT 2026 GPS</title>
    <script src="https://maps.googleapis.com/maps/api/js?key={{ key }}"></script>
    <style>
        #map { height: 100vh; width: 100%; }
        body { margin: 0; padding: 0; background: #000; overflow: hidden; }
        .status-ui { 
            position: absolute; top: 15px; right: 15px; z-index: 10; 
            background: rgba(255,255,255,0.95); padding: 15px; border-radius: 8px; 
            box-shadow: 0 4px 15px rgba(0,0,0,0.4); font-family: sans-serif; min-width: 200px;
        }
        #coord-tooltip {
            position: absolute; background: rgba(0, 0, 0, 0.85); color: #00FF00;
            padding: 8px 12px; border-radius: 4px; font-size: 13px;
            font-family: 'Courier New', monospace; font-weight: bold;
            pointer-events: none; display: none; z-index: 1000;
            white-space: nowrap; border: 1px solid rgba(255,255,255,0.2);
        }
    </style>
</head>
<body>
    <div id="coord-tooltip"></div>
    <div class="status-ui">
        <b style="color: #1A73E8;">🚢 KABOT 2026 MONITOR</b><br>
        <small id="gps-stat" style="font-weight: bold; color: #d93025;">GPS 연결 대기 중...</small><hr>
        <b>LAT:</b> <span id="lat">0.0000000</span><br>
        <b>LON:</b> <span id="lon">0.0000000</span><br>
        <small style="color: #70757a;">(맵/선/마커 어디든 클릭 시 복사)</small>
    </div>
    <div id="map"></div>

    <script>
        let map, boatMarker, trailPath;
        let pathCoordinates = []; 
        let isFirst = true;
        let lastMousedOverLatLng = ""; 
        const tooltip = document.getElementById('coord-tooltip');

        function forceCopy(text) {
            const textArea = document.createElement("textarea");
            textArea.value = text;
            textArea.style.position = "fixed";
            textArea.style.left = "-9999px";
            document.body.appendChild(textArea);
            textArea.focus();
            textArea.select();
            document.execCommand('copy');
            document.body.removeChild(textArea);
        }

        function initMap() {
            map = new google.maps.Map(document.getElementById("map"), {
                zoom: 18,
                center: { lat: 35.2316, lng: 129.0825 },
                mapTypeId: 'roadmap',
                tilt: 0,
                streetViewControl: false
            });

            trailPath = new google.maps.Polyline({
                path: pathCoordinates,
                geodesic: true,
                strokeColor: "#0055FF",
                strokeOpacity: 1.0,
                strokeWeight: 4,
                map: map,
                zIndex: 50 // 지지도보다는 위, 마커보다는 아래
            });

            // 마우스 이동 이벤트 (지도 전체 좌표 감지)
            map.addListener("mousemove", (e) => {
                const lat = e.latLng.lat().toFixed(7);
                const lng = e.latLng.lng().toFixed(7);
                lastMousedOverLatLng = `${lat}, ${lng}`;

                tooltip.style.display = 'block';
                tooltip.style.left = (event.pageX + 15) + 'px';
                tooltip.style.top = (event.pageY + 10) + 'px';
                tooltip.innerHTML = `LAT: ${lat}<br>LON: ${lng}`;
            });

            map.addListener("mouseout", () => {
                tooltip.style.display = 'none';
            });

            // --- 클릭 이벤트 통합 관리 ---
            
            // 1. 지도 빈 곳 클릭
            map.addListener("click", () => {
                if (lastMousedOverLatLng) forceCopy(lastMousedOverLatLng);
            });

            // 2. 경로 선(Polyline) 클릭
            trailPath.addListener("click", () => {
                if (lastMousedOverLatLng) forceCopy(lastMousedOverLatLng);
            });

            boatMarker = new google.maps.Marker({
                position: { lat: 0, lng: 0 },
                map: map,
                zIndex: 100,
                icon: {
                    path: google.maps.SymbolPath.CIRCLE, scale: 7,
                    fillColor: "#EA4335", fillOpacity: 1,
                    strokeWeight: 2, strokeColor: "white"
                }
            });

            // 3. 현재 위치 마커 클릭
            boatMarker.addListener("click", () => {
                if (lastMousedOverLatLng) forceCopy(lastMousedOverLatLng);
            });

            // 4. 웨이포인트 마커 로드 및 클릭 이벤트 추가
            fetch('/waypoints').then(r => r.json()).then(wps => {
                if(!wps) return;
                wps.forEach((wp, i) => {
                    const wpMarker = new google.maps.Marker({
                        position: { lat: wp.lat, lng: wp.lon },
                        map: map,
                        zIndex: 80,
                        label: { text: i.toString(), color: 'white', fontWeight: 'bold' }
                    });
                    
                    // 웨이포인트 클릭 시 복사
                    wpMarker.addListener("click", () => {
                        if (lastMousedOverLatLng) forceCopy(lastMousedOverLatLng);
                    });
                });
            });
        }

        setInterval(() => {
            fetch('/data').then(r => r.json()).then(data => {
                if (data.lat === 0 || isNaN(data.lat)) return;
                const pos = { lat: data.lat, lng: data.lon };
                document.getElementById('lat').innerText = data.lat.toFixed(7);
                document.getElementById('lon').innerText = data.lon.toFixed(7);
                const statElem = document.getElementById('gps-stat');
                statElem.innerText = "정상 수신 중";
                statElem.style.color = "#1e8e3e";
                boatMarker.setPosition(pos);
                const lastPoint = pathCoordinates[pathCoordinates.length - 1];
                if (!lastPoint || lastPoint.lat !== pos.lat || lastPoint.lng !== pos.lng) {
                    pathCoordinates.push(pos);
                    trailPath.setPath(pathCoordinates);
                }
                if (isFirst) { map.setCenter(pos); map.setZoom(20); isFirst = false; }
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
        yaml_name = 'isv_params.yaml'
        if not os.path.exists(yaml_name):
            return jsonify([])
        with open(yaml_name, 'r', encoding='utf-8') as f:
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
        if msg.latitude == msg.latitude and msg.longitude == msg.longitude:
            current_pos["lat"] = msg.latitude
            current_pos["lon"] = msg.longitude

def main():
    rclpy.init()
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5001, debug=False, use_reloader=False), daemon=True).start()
    try:
        rclpy.spin(GPSMapNode())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()