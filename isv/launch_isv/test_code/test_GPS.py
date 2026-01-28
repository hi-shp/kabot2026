#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import threading
from flask import Flask, render_template_string, jsonify, request
import yaml
import os
import sys

app = Flask(__name__)
# 구글 맵 API 키
GOOGLE_MAPS_API_KEY = "AIzaSyDoIwjXsVxvJy0GoNWK8Bf1UjDGktbO1o4"

current_pos = {"lat": 0.0, "lon": 0.0}
last_clicked_pos = {"lat": None, "lon": None} # 마지막 클릭 좌표 저장용
ros_node = None

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>KABOT 2026 MONITOR</title>
    <script src="https://maps.googleapis.com/maps/api/js?key={{ key }}"></script>
    <style>
        #map { height: 100vh; width: 100%; }
        body { margin: 0; padding: 0; background: #000; overflow: hidden; }
        #map { cursor: default !important; }
        #map:active { cursor: pointer !important; }
        #map div, #map img, #map span { cursor: inherit !important; }
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
        <small style="color: #70757a;">(클릭 시 좌표 고정 및 지속 발행)</small>
    </div>
    <div id="map"></div>
    <script>
        let map, boatMarker, trailPath, clickMarker;
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

        function setGpsTarget(lat, lon) {
            fetch('/set_gps', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({lat: lat, lon: lon})
            });
        }

        function handleMapClick(latLng) {
            const lat = latLng.lat();
            const lon = latLng.lng();
            const coordStr = `${lat.toFixed(7)}, ${lon.toFixed(7)}`;
            
            forceCopy(coordStr); 
            setGpsTarget(lat, lon); 
            
            clickMarker.setPosition(latLng);
            clickMarker.setMap(map);
        }

        function initMap() {
            map = new google.maps.Map(document.getElementById("map"), {
                zoom: 19,
                center: { lat: 35.2316, lng: 129.0825 },
                mapTypeId: 'roadmap',
                tilt: 0,
                streetViewControl: false,
                draggableCursor: 'default',
                draggingCursor: 'pointer'
            });

            clickMarker = new google.maps.Marker({
                map: null,
                zIndex: 9999, 
                optimized: false,
                icon: {
                    path: google.maps.SymbolPath.CIRCLE,
                    scale: 7,
                    fillColor: "#0000FF",
                    fillOpacity: 1,
                    strokeWeight: 2,
                    strokeColor: "#FFFFFF"
                }
            });

            trailPath = new google.maps.Polyline({
                path: pathCoordinates,
                geodesic: true,
                strokeColor: "#0055FF",
                strokeOpacity: 1.0,
                strokeWeight: 4,
                map: map,
                zIndex: 50
            });

            map.addListener("mousemove", (e) => {
                const lat = e.latLng.lat().toFixed(7);
                const lng = e.latLng.lng().toFixed(7);
                lastMousedOverLatLng = `${lat}, ${lng}`;
                tooltip.style.display = 'block';
                tooltip.style.left = (event.pageX + 15) + 'px';
                tooltip.style.top = (event.pageY + 10) + 'px';
                tooltip.innerHTML = `LAT: ${lat}<br>LON: ${lng}`;
                if (clickMarker.getMap()) {
                    clickMarker.setPosition(e.latLng);
                }
            });

            map.addListener("mouseout", () => { tooltip.style.display = 'none'; });

            map.addListener("mousedown", (e) => { 
                handleMapClick(e.latLng);
            });

            map.addListener("mouseup", () => { 
                clickMarker.setMap(null);
            });

            boatMarker = new google.maps.Marker({
                position: { lat: 0, lng: 0 },
                map: map,
                zIndex: 500,
                icon: {
                    path: google.maps.SymbolPath.CIRCLE, scale: 7,
                    fillColor: "#EA4335", fillOpacity: 1,
                    strokeWeight: 2, strokeColor: "white"
                }
            });

            fetch('/waypoints').then(r => r.json()).then(wps => {
                if(!wps || wps.length === 0) return;
                wps.forEach((wp, i) => {
                    const wpMarker = new google.maps.Marker({
                        position: { lat: wp.lat, lng: wp.lon },
                        map: map,
                        zIndex: 10,
                        clickable: true,
                        label: {
                            text: (i + 1).toString(),
                            color: 'white', fontWeight: 'bold', fontSize: '11px'
                        },
                        icon: {
                            path: google.maps.SymbolPath.CIRCLE,
                            scale: 10,
                            fillColor: "#000000",
                            fillOpacity: 1,
                            strokeWeight: 1.5,
                            strokeColor: "#FFFFFF"
                        }
                    });

                    wpMarker.addListener("mousedown", (e) => {
                        handleMapClick(e.latLng);
                    });
                    wpMarker.addListener("mouseup", () => {
                        clickMarker.setMap(null);
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
                if (isFirst) { map.setCenter(pos); isFirst = false; }
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

@app.route('/set_gps', methods=['POST'])
def set_gps():
    global last_clicked_pos
    data = request.json
    if data:
        last_clicked_pos["lat"] = data['lat']
        last_clicked_pos["lon"] = data['lon']
        return jsonify({"status": "target_updated"})
    return jsonify({"status": "failed"}), 400

@app.route('/waypoints')
def get_waypoints():
    try:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        parent_dir = os.path.dirname(current_dir)
        yaml_path = os.path.join(parent_dir, 'isv_params.yaml')
        if not os.path.exists(yaml_path): return jsonify([])
        with open(yaml_path, 'r', encoding='utf-8') as f:
            params = yaml.safe_load(f)
        raw_wps = params.get('navigation', {}).get('waypoints', [])
        formatted_wps = [{"lat": float(wp[0]), "lon": float(wp[1])} for wp in raw_wps if len(wp) >= 2]
        return jsonify(formatted_wps)
    except Exception as e:
        return jsonify([])

class GPSMapNode(Node):
    def __init__(self):
        super().__init__('google_map_viewer')
        self.subscription = self.create_subscription(NavSatFix, "/gps/fix", self.gps_cb, 10)
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # 1초에 한 번씩 타이머 실행 (1.0Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def gps_cb(self, msg):
        if msg.latitude == msg.latitude and msg.longitude == msg.longitude:
            current_pos["lat"] = msg.latitude
            current_pos["lon"] = msg.longitude

    def timer_callback(self):
        # 마지막으로 클릭된 좌표가 있을 때만 발행
        if last_clicked_pos["lat"] is not None:
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps'
            msg.latitude = last_clicked_pos["lat"]
            msg.longitude = last_clicked_pos["lon"]
            msg.status.status = 2 # RTK Fix 상태 유지
            self.publisher_.publish(msg)
            
            # 터미널 피드백 출력
            print("\033[H", end="") 
            print("="*45)
            print(f" [GPS 지속 발행 중 (1Hz)]")
            print(f" 고정 좌표: Lat {last_clicked_pos['lat']:.8f}, Lon {last_clicked_pos['lon']:.8f} ")
            print("="*45)

def main():
    global ros_node
    rclpy.init()
    ros_node = GPSMapNode()
    
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5001, debug=False, use_reloader=False), daemon=True).start()
    
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok():
            ros_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()