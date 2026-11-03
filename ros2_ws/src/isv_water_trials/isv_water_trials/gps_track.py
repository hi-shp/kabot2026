"""Write a valid GNSS track and a clickable live map URL; no control outputs."""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time


def valid_fix(message):
    return (message.status.status >= 0 and math.isfinite(message.latitude)
            and math.isfinite(message.longitude) and -90 <= message.latitude <= 90
            and -180 <= message.longitude <= 180)


def map_url(latitude, longitude):
    return f'https://www.google.com/maps?q={latitude:.8f},{longitude:.8f}'


def main(argv=None):
    from .shutdown import keep_terminal_loss
    keep_terminal_loss()
    parser = argparse.ArgumentParser(description='Record GNSS track and Google Maps URL')
    parser.add_argument('--output', required=True)
    args = parser.parse_args(argv)
    output = Path(args.output).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix
    rclpy.init()
    node = rclpy.create_node('water_trial_gps_track')
    points = []
    started = time.monotonic()
    csv_file = (output / 'gps_track.csv').open('w', newline='', encoding='utf-8')
    writer = csv.writer(csv_file)
    writer.writerow(['utc_received', 'latitude_deg', 'longitude_deg', 'altitude_m', 'fix_status'])

    def save(message):
        if not valid_fix(message):
            return
        latitude, longitude = message.latitude, message.longitude
        utc = datetime.now(timezone.utc).isoformat()
        writer.writerow([utc, latitude, longitude, message.altitude, message.status.status])
        csv_file.flush()
        points.append([longitude, latitude])
        if len(points) > 36000:
            del points[:len(points) - 36000]
        (output / 'gps_latest_url.txt').write_text(map_url(latitude, longitude) + '\n', encoding='utf-8')
        if time.monotonic() - save.last_geojson >= 5:
            save.last_geojson = time.monotonic()
            data = {'type': 'FeatureCollection', 'features': [
                {'type': 'Feature', 'properties': {'source': 'u-blox GNSS', 'utc': utc},
                 'geometry': {'type': 'LineString', 'coordinates': points}},
                {'type': 'Feature', 'properties': {'latest': True, 'utc': utc},
                 'geometry': {'type': 'Point', 'coordinates': points[-1]}}]}
            target = output / 'gps_track.geojson'
            temporary = target.with_suffix('.geojson.tmp')
            temporary.write_text(json.dumps(data, ensure_ascii=False), encoding='utf-8')
            temporary.replace(target)
            node.get_logger().info(f'GNSS fixes={len(points)} map={map_url(latitude, longitude)}')

    save.last_geojson = started - 5
    subscription = node.create_subscription(NavSatFix, '/gps/fix', save, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
