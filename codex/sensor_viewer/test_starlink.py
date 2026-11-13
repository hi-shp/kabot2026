"""Offline fixtures only: no VPN login, SSH connection or vessel command."""
import io
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import paramiko
from connection_controls import ConnectionRequests
from connection_network import WifiRoutes
from pc_fast_feed import Feed
from run_viewer import RemoteBridge, ssh_settings
from ssh_discovery import update_verified_alias
from starlink_link import StarlinkLink, parse_status


def status(backend='Running', online=True, name='kaboat-jetson'):
    return dict(BackendState=backend, Self={'Online': True}, TailscaleIPs=['100.80.0.1'],
                Peer={'jetson': dict(HostName=name, DNSName=name+'.example.ts.net.',
                      Online=online, OS='linux', TailscaleIPs=['100.80.0.2', 'fd7a::2'])})


def bridge():
    b = RemoteBridge(NS(host='auto', port=8765, user=None, ssh_port=None, no_browser=True), None,
                     wifi=WifiRoutes(reader=lambda:'welcome_CU'))
    b.starlink = Mock()
    return b


class StarlinkTests(unittest.TestCase):
    def test_only_named_online_linux_peer_is_an_endpoint(self):
        data = status()
        data['Peer']['unrelated'] = dict(HostName='other-linux', OS='linux', Online=True, TailscaleIPs=['100.80.0.3'])
        data['Peer']['jetson']['TailscaleIPs'] += ['192.168.1.20']
        self.assertEqual([p['address'] for p in parse_status(data)['candidates']], ['100.80.0.2'])
        self.assertEqual(parse_status(status(name='other-linux'))['state'], 'peer_offline')

    def test_prerequisite_states_do_not_claim_connected(self):
        for backend, expected in [('NeedsLogin', 'needs_login'), ('Stopped', 'stopped'), ('Starting', 'network_wait')]:
            value = parse_status(status(backend))
            self.assertEqual(value['state'], expected)
            self.assertFalse(value['ready'])
        self.assertEqual(parse_status(status(online=False))['state'], 'peer_offline')

    def test_missing_installation_and_broken_service_are_reported(self):
        with patch('starlink_link.executable', return_value=None):
            self.assertEqual(StarlinkLink().status()['state'], 'not_installed')
        with patch('starlink_link.executable', return_value='tailscale'):
            self.assertEqual(StarlinkLink(runner=Mock(side_effect=OSError())).status()['state'], 'service_wait')

    def test_fresh_install_can_report_null_ip_list(self):
        value = parse_status(dict(BackendState='NeedsLogin', Self=None, Peer=None, TailscaleIPs=None))
        self.assertEqual(value['state'], 'needs_login')
        self.assertEqual(value['local_ips'], [])

    def test_poll_never_starts_vpn_or_opens_login(self):
        runner = Mock(return_value=NS(stdout=json.dumps(status('NeedsLogin'))))
        opener = Mock()
        with patch('starlink_link.executable', return_value='tailscale'), patch('starlink_link.subprocess.Popen') as launch:
            link = StarlinkLink(runner=runner, opener=opener)
            link.status();link.candidates()
            launch.assert_not_called();opener.assert_not_called()
            launch.return_value.poll.return_value = None
            link.prepare_login();link.prepare_login()
            launch.assert_called_once()

    def test_selected_failure_keeps_working_transport_and_retries(self):
        b = bridge();old = Mock();b.client = old;b.active_host = 'jetson-hotspot';b.remote_port = 1234
        b.connect_host = Mock(side_effect=OSError('offline'));b.discover_host = Mock(return_value=None)
        b.request_connection('starlink');job = b.connections.claim()
        self.assertFalse(b.connect_requested(job))
        self.assertIs(b.client, old);old.close.assert_not_called()
        self.assertEqual(b.connections.snapshot()['state'], 'waiting')
        self.assertEqual(b.remote_port, 1234)

    def test_retry_can_request_login_after_installation(self):
        b = bridge();b.connect_host = Mock(side_effect=OSError());b.discover_host = Mock(return_value=None)
        b.request_connection('starlink');job = b.connections.claim();job['attempts'] = 2
        b.connect_requested(job)
        b.starlink.prepare_login.assert_called_once()

    def test_reconnect_after_explicit_selection_can_restart_vpn(self):
        b = bridge();b.manual_host = 'jetson-starlink';b.connect_host = Mock(return_value=object())
        b.connect_preferred();b.starlink.prepare_login.assert_called_once()

    def test_initializing_client_can_be_started_after_explicit_request(self):
        runner = Mock(return_value=NS(stdout=json.dumps(status('NoState'))))
        with patch('starlink_link.executable', return_value='tailscale'), patch('starlink_link.subprocess.Popen') as launch:
            StarlinkLink(runner=runner).prepare_login()
            launch.assert_called_once()
            self.assertIn('up', launch.call_args.args[0])

    def test_repeated_clicks_coalesce_and_latest_request_wins(self):
        queue = ConnectionRequests()
        first = queue.request('starlink', client_id='tab', sequence=1)
        self.assertEqual(queue.request('starlink')['request_id'], first['request_id'])
        old = queue.claim()
        latest = queue.request('hotspot', client_id='tab', sequence=3)
        self.assertEqual(queue.request('starlink', client_id='tab', sequence=2)['request_id'], latest['request_id'])
        self.assertFalse(queue.finish(old, ip='old'))
        with self.assertRaises(ValueError):queue.request('rocket')

    def test_success_switches_transport_without_replacing_monitor(self):
        b = bridge();old = Mock();b.client = old;b.remote_port = 1234
        candidate = Mock();b.connect_host = Mock(return_value=candidate)
        b.request_connection('starlink');job = b.connections.claim()
        self.assertTrue(b.connect_requested(job))
        self.assertIs(b.client, candidate);self.assertEqual(b.remote_port, 1234)
        self.assertEqual(b.manual_host, 'jetson-starlink');old.close.assert_called_once()
        b.connect_host.reset_mock();b.maybe_prefer_starlink();b.connect_host.assert_not_called()

    def test_link_metadata_and_fast_feed_use_same_peer(self):
        b = bridge();b.starlink.status.return_value = parse_status(status())
        b.client = Mock();b.active_host = 'jetson-starlink';b.last_health = time.monotonic()
        b.client.get_transport.return_value.sock.getpeername.return_value = ('100.80.0.2', 22)
        b.client.get_transport.return_value.sock.getsockname.return_value = ('100.80.0.1', 10000)
        meta = b.metadata();self.assertTrue(meta['connected']);self.assertEqual(meta['connection_path'], 'starlink')
        feed = Feed(NS(follow_dashboard=True, dashboard='http://127.0.0.1:8765'))
        with patch('pc_fast_feed.urllib.request.urlopen', return_value=io.BytesIO(json.dumps(meta).encode())):
            self.assertEqual(feed.dashboard_host(), 'jetson-starlink')
        feed.client = b.client;feed.connected_host = 'jetson-starlink'
        with patch('pc_fast_feed.urllib.request.urlopen', return_value=io.BytesIO(json.dumps(dict(meta, target_ip='100.80.0.9')).encode())):
            feed.check_dashboard_route()
        b.client.close.assert_called_once()

    def test_new_alias_uses_pinned_identity_and_preserves_other_blocks(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory);(root/'.ssh').mkdir()
            config = root/'.ssh/config'
            original = 'Host jetson-hotspot\n HostName 192.168.1.20\n User isv\n HostKeyAlias verified-jetson\n IdentityFile ~/.ssh/id_isv\n\nHost other\n HostName 10.0.0.2\n'
            config.write_text(original, encoding='utf-8')
            key = paramiko.RSAKey.generate(1024);known = paramiko.HostKeys();known.add('verified-jetson', 'ssh-rsa', key)
            with patch('run_viewer.Path.home', return_value=root):
                settings = ssh_settings(NS(host='jetson-starlink', user=None, ssh_port=None), known)
            self.assertEqual(settings['username'], 'isv')
            self.assertIs(known.lookup(settings['hostname'])['ssh-rsa'], key)
            update_verified_alias('jetson-starlink', '100.80.0.2', path=config)
            self.assertTrue(config.read_text().startswith(original.rstrip()))
            self.assertIn('HostKeyAlias verified-jetson', config.read_text())
            self.assertIn('HostName 100.80.0.2', config.read_text())
            with patch('run_viewer.Path.home', return_value=root):
                self.assertEqual(ssh_settings(NS(host='jetson-starlink', user=None, ssh_port=None), known)['hostname'], '100.80.0.2')

    def test_first_connection_without_pinned_identity_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory, patch('run_viewer.Path.home', return_value=Path(directory)):
            with self.assertRaises(ConnectionError):
                ssh_settings(NS(host='jetson-starlink', user=None, ssh_port=None), paramiko.HostKeys())

    def test_feed_loads_pinned_key_for_new_overlay_ip(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory);(root/'.ssh').mkdir()
            (root/'.ssh/config').write_text('Host jetson-starlink\n HostName 100.80.0.2\n User isv\n HostKeyAlias verified-jetson\n', encoding='utf-8')
            key = paramiko.RSAKey.generate(1024);saved = paramiko.HostKeys();saved.add('verified-jetson', 'ssh-rsa', key)
            saved.save(str(root/'.ssh/known_hosts'))
            loaded = paramiko.HostKeys();client = Mock();client.get_host_keys.return_value = loaded
            client.load_host_keys.side_effect = loaded.load
            client.connect.side_effect = ConnectionError('fixture stops before network access')
            feed = Feed(NS(host='jetson-starlink', ssh_port=None, user=None))
            with patch('pc_fast_feed.Path.home', return_value=root), patch('pc_fast_feed.paramiko.SSHClient', return_value=client):
                with self.assertRaises(ConnectionError):feed.connect()
            self.assertEqual(loaded.lookup('100.80.0.2')['ssh-rsa'], key)
            self.assertEqual(client.connect.call_args.args[0], '100.80.0.2')

    def test_wait_mode_does_not_connect_before_click(self):
        b = bridge();b.args.wait_for_connection = True
        b.connect_host = Mock();b.connect_preferred = Mock()
        worker = threading.Thread(target=b.work);worker.start()
        b.stop.set();b.connection_wake.set();worker.join(3)
        self.assertFalse(worker.is_alive());b.connect_host.assert_not_called();b.connect_preferred.assert_not_called()


if __name__ == '__main__':
    unittest.main()
