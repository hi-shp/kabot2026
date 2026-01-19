#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import threading
from flask import Flask, render_template_string, jsonify

# --- Flask 앱 설정 ---
app = Flask(__name__)
current_pos = {"lat": 0.0, "lon": 0.0}
path_history = []

# HTML/Leaflet 프론트엔드 코드
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>ROS2 GPS Live Map</title>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        #map { height: 100vh; width: 100%; }
        .info-box { 
            position: absolute; top: 10px; right: 10px; z-index: 1000; 
            background: white; padding: 10px; border-radius: 5px; 
            box-shadow: 0 0 15px rgba(0,0,0,0.2); font-family: sans-serif;
        }
    </style>
</head>
<body>
    <div id="map"></div>
    <div class="info-box">
        <strong>마우스 좌표:</strong> <span id="mouse-coords">0, 0</span><br>
        <small style="color: #666;">좌클릭: "위도, 경도" 복사</small>
    </div>

    <script>
        var map = L.map('map').setView([0, 0], 2);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);

        var pathLine = L.polyline([], {color: 'red', weight: 3}).addTo(map);
        
        // 현재 위치를 나타내는 작은 점 (CircleMarker)
        var currentDot = L.circleMarker([0, 0], {
            radius: 6,
            fillColor: "#007bff",
            color: "#ffffff",
            weight: 2,
            opacity: 1,
            fillOpacity: 0.8
        }).addTo(map);

        var isFirstLock = true;

        // 마우스 움직임 감지
        map.on('mousemove', function(e) {
            document.getElementById('mouse-coords').innerHTML = e.latlng.lat.toFixed(7) + ", " + e.latlng.lng.toFixed(7);
        });

        // [좌클릭] 클립보드 복사 (숫자, 숫자) - 알림 없음
        map.on('click', function(e) {
            const text = e.latlng.lat.toFixed(7) + ", " + e.latlng.lng.toFixed(7);
            navigator.clipboard.writeText(text);
        });

        // 우클릭 이동 기능 삭제 (브라우저 기본 메뉴 사용 가능)

        // 1초마다 데이터 업데이트
        setInterval(function() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    if (data.lat === 0 && data.lon === 0) return;
                    
                    var newPos = [data.lat, data.lon];
                    currentDot.setLatLng(newPos);
                    pathLine.setLatLngs(data.path);

                    if (isFirstLock) {
                        map.setView(newPos, 18);
                        isFirstLock = false;
                    }
                });
        }, 1000);
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/data')
def data():
    return jsonify({"lat": current_pos["lat"], "lon": current_pos["lon"], "path": path_history})

# --- ROS 2 Node ---
class GPSMapServer(Node):
    def __init__(self):
        super().__init__('gps_map_server')
        self.create_subscription(NavSatFix, "/gps/fix", self.gps_cb, 10)
        self.get_logger().info("Map Server Started. Access http://localhost:5000")

    def gps_cb(self, msg: NavSatFix):
        if msg.latitude != msg.latitude:
            return
            
        current_pos["lat"] = msg.latitude
        current_pos["lon"] = msg.longitude
        
        if not path_history or (abs(path_history[-1][0] - msg.latitude) > 0.000001):
            path_history.append([msg.latitude, msg.longitude])

def run_flask():
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def main(args=None):
    rclpy.init(args=args)
    node = GPSMapServer()

    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()