from types import SimpleNamespace
import io
import unittest
from unittest.mock import Mock, patch

from run_viewer import RemoteBridge
from pc_fast_feed import Feed
from connection_network import WifiRoutes


class ConnectionPriorityTests(unittest.TestCase):
    def bridge(self,ssid='welcome_CU'):
        return RemoteBridge(SimpleNamespace(host='auto', port=8765, no_browser=True), None,
                            wifi=WifiRoutes(reader=lambda:ssid))

    def test_starlink_is_first_and_hotspot_is_not_tried(self):
        bridge = self.bridge()
        client = object()
        bridge.connect_host = Mock(return_value=client)
        self.assertEqual(bridge.connect_preferred(), ('jetson-starlink', client))
        bridge.connect_host.assert_called_once_with('jetson-starlink')

    def test_kk_uses_only_hotspot(self):
        bridge = self.bridge('Kk')
        client = object()
        bridge.connect_host = Mock(return_value=client)
        self.assertEqual(bridge.connect_preferred(), ('jetson-hotspot', client))
        bridge.connect_host.assert_called_once_with('jetson-hotspot')

    def test_welcome_cu_does_not_fall_back_to_hotspot(self):
        bridge = self.bridge()
        client = object()
        bridge.connect_host = Mock(side_effect=[OSError('offline'), client])
        bridge.discover_host = Mock(return_value=None)
        with self.assertRaises(ConnectionError):bridge.connect_preferred()
        self.assertEqual([call.args[0] for call in bridge.connect_host.call_args_list],
                         ['jetson-starlink'])

    def test_no_silent_fallback_when_both_are_unavailable(self):
        bridge = self.bridge()
        bridge.connect_host = Mock(side_effect=OSError('offline'))
        bridge.discover_host = Mock(return_value=None)
        with self.assertRaises(ConnectionError):
            bridge.connect_preferred()

    def test_return_to_starlink_requires_two_successful_checks(self):
        bridge = self.bridge()
        bridge.active_host = 'jetson-hotspot'
        previous, first, second = Mock(), Mock(), Mock()
        bridge.client = previous
        bridge.connect_host = Mock(side_effect=[first, second])
        with patch('run_viewer.time.monotonic', side_effect=[0, 0, 31, 31]):
            bridge.maybe_prefer_starlink()
            self.assertEqual(bridge.active_host, 'jetson-hotspot')
            first.close.assert_called_once()
            bridge.maybe_prefer_starlink()
        self.assertEqual(bridge.active_host, 'jetson-starlink')
        self.assertIs(bridge.client, second)
        previous.close.assert_called_once()

    def test_fast_feed_follows_actual_dashboard_connection(self):
        feed = Feed(SimpleNamespace(host='auto', follow_dashboard=True,
                                    dashboard='http://127.0.0.1:8765'))
        for host in ('jetson-starlink', 'jetson-hotspot'):
            payload = ('{"connected":true,"active_host":"'+host+'"}').encode()
            with patch('pc_fast_feed.urllib.request.urlopen', return_value=io.BytesIO(payload)):
                self.assertEqual(feed.dashboard_host(), host)
        with patch('pc_fast_feed.urllib.request.urlopen',
                   return_value=io.BytesIO(b'{"connected":false}')):
            with self.assertRaises(ConnectionError):
                feed.dashboard_host()


if __name__ == '__main__':
    unittest.main()
