import matplotlib.pyplot as plt
import pymap3d as pm
import math

# -----------------------------
# 1) GPS 리스트 (lat, lon)
# -----------------------------
goal_gps_coords = [
            (35.2316100, 129.0827780),
            (35.2316324, 129.0827477),
            (35.2316402, 129.0827229),
            (35.2316642, 129.0827196),
            (35.2316615, 129.0827810),
            (35.2316773, 129.0827648),
            (35.2316836, 129.0827230),
]

# -----------------------------
# 2) ENU origin
# -----------------------------
origin = (35.2316100, 129.0827780, 0.0)

# -----------------------------
# 3) GPS → ENU 변환
# -----------------------------
enu_points = []
for lat, lon in goal_gps_coords:
    e, n, u = pm.geodetic2enu(lat, lon, 0.0, *origin)
    enu_points.append((e, n))

# x, y 분리
es = [p[0] for p in enu_points]
ns = [p[1] for p in enu_points]

# -----------------------------
# 4) Plot
# -----------------------------
plt.figure(figsize=(8, 8))

# Path
plt.plot(es, ns, '-o', markersize=8, label="Waypoints")

# -----------------------------
# 5) 각 waypoint 번호 표시
# -----------------------------
for i, (e, n) in enumerate(enu_points):
    plt.text(e, n + 0.2, f"{i}", fontsize=12, color="red", ha='center')

# -----------------------------
# 6) 점 사이 거리 계산 & 표시
# -----------------------------
for i in range(len(enu_points) - 1):
    x1, y1 = enu_points[i]
    x2, y2 = enu_points[i + 1]

    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    # 두 점 중간 좌표
    mid_x = (x1 + x2) / 2
    mid_y = (y1 + y2) / 2

    # 거리 표시
    plt.text(mid_x, mid_y, f"{dist:.2f} m", fontsize=10, color="blue", ha='center')

# -----------------------------
# 7) Origin 표시
# -----------------------------
plt.scatter(0, 0, color="green", s=120, label="Origin (0,0)")
plt.text(0, 0.3, "Origin", fontsize=12, color="green", ha='center')

# -----------------------------
# 8) Plot 설정
# -----------------------------
plt.xlabel("East (m)")
plt.ylabel("North (m)")
plt.title("Waypoints + Distances + Origin (ENU Coordinates)")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.show()
