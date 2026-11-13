"""Wi-Fi route enforcement without changing networks or connecting to Jetson."""
import json
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

from connection_network import parse_ssid, policy_for_ssid, WifiRoutes
from run_viewer import RemoteBridge, make_handler


class ConnectionNetworkTests(unittest.TestCase):
    def bridge(self,ssid='Kk'):
        self.reader=Mock(return_value=ssid)
        return RemoteBridge(NS(host='auto',port=8765,user=None,ssh_port=None,no_browser=True),None,
                            wifi=WifiRoutes(reader=self.reader,ttl=0))

    def test_ssid_parser_ignores_bssid_and_saved_profile(self):
        self.assertEqual(parse_ssid(' State : connected\n SSID : Kk\n AP BSSID : aa:bb\n Profile : welcome_CU\n'),'Kk')
        self.assertIsNone(parse_ssid(' State : disconnected\n'))
        self.assertIsNone(parse_ssid('SSID : Kk\nSSID : welcome_CU\n'))

    def test_only_the_designated_wifi_enables_each_route(self):
        for ssid,expected in [('Kk',{'hotspot':True,'starlink':False}),
                              ('welcome_CU',{'hotspot':False,'starlink':True}),
                              ('unrelated',{'hotspot':False,'starlink':False}),
                              (None,{'hotspot':False,'starlink':False})]:
            with self.subTest(ssid=ssid):self.assertEqual(policy_for_ssid(ssid)['allowed'],expected)

    def test_wrong_route_cannot_queue_or_attempt_ssh(self):
        b=self.bridge()
        with self.assertRaises(ValueError):b.request_connection('starlink')
        self.assertIsNone(b.connections.snapshot())
        with patch('paramiko.SSHClient') as client:
            with self.assertRaises(ValueError):b.connect_host('jetson-starlink')
            client.assert_not_called()
        b.starlink=Mock()
        self.assertIsNone(b.discover_host('jetson-starlink'))
        b.starlink.candidates.assert_not_called()

    def test_kk_metadata_never_reports_overlay_as_connected(self):
        b=self.bridge();b.starlink=Mock()
        b.starlink.status.return_value={'state':'ready','ready':True,'candidates':[]}
        b.client=Mock();b.active_host='jetson-starlink';b.last_health=b.started
        value=b.metadata()
        self.assertFalse(value['connected'])
        self.assertFalse(value['starlink']['ready'])
        self.assertTrue(value['starlink']['vpn_ready'])
        self.assertEqual(value['starlink']['state'],'wifi_blocked')
        self.assertEqual(value['wifi']['ssid'],'Kk')

    def test_network_change_cancels_incompatible_job_and_transport(self):
        b=self.bridge('welcome_CU')
        b.request_connection('starlink');b.manual_host='jetson-starlink'
        previous=Mock();b.client=previous;b.active_host='jetson-starlink'
        self.reader.return_value='Kk'
        with patch('run_viewer.adapter_networks',return_value=[]):b.check_links()
        previous.close.assert_called_once()
        self.assertIsNone(b.manual_host)
        self.assertFalse(b.connections.snapshot()['busy'])
        b.connect_host=Mock(return_value=object())
        self.assertEqual(b.connect_preferred()[0],'jetson-hotspot')

    def test_network_change_during_request_cannot_replace_working_client(self):
        b=self.bridge('welcome_CU');b.starlink=Mock()
        b.request_connection('starlink');job=b.connections.claim()
        previous,candidate=Mock(),Mock();b.client=previous
        def connected(*args,**kwargs):
            self.reader.return_value='Kk'
            return candidate
        b.connect_host=Mock(side_effect=connected)
        self.assertFalse(b.connect_requested(job))
        self.assertIs(b.client,previous)
        previous.close.assert_not_called();candidate.close.assert_called_once()

    def test_wrong_route_http_post_is_rejected_before_queue(self):
        b=self.bridge()
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(b,NS(port=0)))
        server.RequestHandlerClass=make_handler(b,NS(port=server.server_port))
        worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
        try:
            url='http://127.0.0.1:'+str(server.server_port)
            request=urllib.request.Request(url+'/api/connection',data=json.dumps({'route':'starlink'}).encode(),
                headers={'Origin':url,'Content-Type':'application/json'},method='POST')
            with self.assertRaises(urllib.error.HTTPError) as caught:urllib.request.urlopen(request,timeout=2)
            self.assertEqual(caught.exception.code,400)
            self.assertIsNone(b.connections.snapshot())
        finally:
            server.shutdown();server.server_close();worker.join(2)


if __name__=='__main__':unittest.main()
