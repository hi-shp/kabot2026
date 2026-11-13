"""Synthetic local fixture only: verifies push/render/staleness, no SSH or ROS."""
from http.server import ThreadingHTTPServer
import json
import threading
import time
from types import SimpleNamespace
import unittest

from selenium import webdriver
from selenium.webdriver.support.ui import WebDriverWait

from pc_fast_feed import make_handler as feed_handler
from run_viewer import make_handler
from test_outdoor_browser import sample


class ReadOnlyBridge:
    browser = SimpleNamespace(viewed=lambda: None)

    def metadata(self):
        return dict(status='fixture', sensors_running=True)

    def request(self, method, path, body=None):
        assert method == 'GET', 'The display test must never send a vessel command'
        if path.startswith('/api/cloud'):
            return 204, {}, b''
        value = dict(values={'outdoor': sample()}, nodes=[], topics=[], limits={}, errors={}, selected_topic='', preview=None)
        return 200, {'Content-Type': 'application/json'}, json.dumps(value).encode()


class PcFastDashboard(unittest.TestCase):
    def test_push_paints_sensors_and_path_and_expires_disconnected_data(self):
        args = SimpleNamespace(port=0)
        server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(ReadOnlyBridge(), args))
        args.port = server.server_port
        origin = 'http://127.0.0.1:'+str(args.port)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        feed = SimpleNamespace(condition=threading.Condition(), sequence=0, packet={}, error=None, updated=0)
        push = ThreadingHTTPServer(('127.0.0.1', 0), feed_handler(feed, SimpleNamespace(dashboard=origin)))
        threading.Thread(target=push.serve_forever, daemon=True).start()
        stopped = threading.Event()

        def emit():
            while not stopped.wait(.04):
                now = time.time()
                with feed.condition:
                    feed.sequence += 1
                    feed.updated = time.monotonic()
                    feed.packet = dict(sequence=feed.sequence, instance_id='test', request_ms=0, mount=[0, 0, 0], dimensions=[1.1, .5],
                        values=dict(heading=dict(heading_deg=40+feed.sequence % 10, stamp_s=now, age_s=0),
                            scan=dict(polar_deg_m=[], rays=898, stamp_s=now, age_s=0),
                            gps=dict(valid_fix=True, latitude_deg=35.232, longitude_deg=129.083, stamp_s=now, age_s=0)))
                    feed.condition.notify_all()

        threading.Thread(target=emit, daemon=True).start()
        options = webdriver.ChromeOptions()
        options.add_argument('--headless=new')
        options.add_argument('--window-size=1600,1200')
        options.set_capability('goog:loggingPrefs', {'browser': 'ALL'})
        driver = webdriver.Chrome(options=options)
        try:
            # Redirect only the test browser's stream to our isolated fixture.
            source = "const RealEventSource=EventSource;window.EventSource=class extends RealEventSource{constructor(url,opts){super(url==='http://127.0.0.1:18766/events'?"+json.dumps('http://127.0.0.1:'+str(push.server_port)+'/events')+":url,opts);}};"
            driver.execute_cdp_cmd('Page.addScriptToEvaluateOnNewDocument', {'source': source})
            driver.execute_cdp_cmd('Network.enable', {})
            driver.execute_cdp_cmd('Network.setBlockedURLs', {'urls': ['*arcgisonline.com/*', '*openstreetmap.org/*']})
            driver.get(origin+'/?capture=1')
            wait = WebDriverWait(driver, 15)
            wait.until(lambda d: d.execute_script('return window.kabotPcPlannerStats?.planning_hz >= 20 && window.kabotPcPlannerStats?.paint_hz >= 20'))
            stats = driver.execute_script('return window.kabotPcPlannerStats')
            self.assertEqual(stats['transport'], 'push')
            self.assertGreaterEqual(stats['paint_hz'], 20)
            self.assertIn('PC 근거리 경로', stats['status'])
            driver.switch_to.frame('sensorFrame')
            wait.until(lambda d: d.find_element('id', 'attitude').get_attribute('data-heading-deg'))
            first = driver.find_element('id', 'attitude').get_attribute('data-heading-deg')
            wait.until(lambda d: d.find_element('id', 'attitude').get_attribute('data-heading-deg') != first)
            driver.switch_to.default_content()
            stopped.set()
            with feed.condition:
                feed.error = 'fixture disconnected'
                feed.condition.notify_all()
            wait.until(lambda d: '대기' in d.find_element('id', 'pcPlannerStatus').text)
            errors = [x['message'] for x in driver.get_log('browser') if 'Uncaught' in x['message']]
            self.assertEqual(errors, [])
            print('LOCAL_FIXTURE_STATS', json.dumps(stats, ensure_ascii=True))
        except Exception:
            print('FAILED_STATS',driver.execute_script('return window.kabotPcPlannerStats'))
            print('BROWSER_ERRORS',[x['message'] for x in driver.get_log('browser') if 'Uncaught' in x['message']])
            raise
        finally:
            stopped.set()
            driver.quit()
            server.shutdown()
            push.shutdown()
            server.server_close()
            push.server_close()


if __name__ == '__main__':
    unittest.main()
