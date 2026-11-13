from concurrent.futures import Future
from concurrent.futures import ThreadPoolExecutor
import json
import time
import sys
from types import SimpleNamespace as N
import unittest
from unittest.mock import Mock, patch

from jetson_server import Store
from outdoor_feed import OutdoorFeed


class Fix:
    def __init__(self):
        self.header=N(stamp=None,frame_id='');self.status=N(status=-1,service=0)


class VirtualGpsTests(unittest.TestCase):
    def setUp(self):
        self.node=Mock();self.node.create_publisher.side_effect=lambda *_:Mock()
        self.node.get_clock.return_value.now.return_value.to_msg.return_value='stamp'
        self.store=Store(N())
        with patch.dict(sys.modules,{'std_msgs':N(),'std_msgs.msg':N(String=Mock()),
                                    'sensor_msgs':N(),'sensor_msgs.msg':N(NavSatFix=Fix)}):
            self.feed=OutdoorFeed(self.node,self.store)
        self.feed.publisher.get_subscription_count.return_value=1

    def command(self, action, **data):
        future=Future();self.feed.queue.put((dict(action=action,request_id='test',**data),future))
        self.feed.work();return future.result(timeout=1)

    def test_only_explicit_start_streams_distinct_topic_with_fresh_stamp(self):
        self.feed.publish_virtual_gps();self.feed.virtual_publisher.publish.assert_not_called()
        status,result=self.command('virtual_gps_start',point=[35.,129.])
        self.assertEqual(status,200);self.assertTrue(result['simulation'])
        self.feed.publish_virtual_gps();self.feed.publish_virtual_gps()
        msg=self.feed.virtual_publisher.publish.call_args.args[0]
        self.assertEqual((msg.latitude,msg.longitude,msg.header.frame_id),(35.,129.,'virtual_gps'))
        self.assertEqual(msg.header.stamp,'stamp')
        self.assertEqual(self.store.values['virtual_gps']['published'],3)
        self.assertIn('/gps/virtual/fix',[call.args[1] for call in self.node.create_publisher.call_args_list])
        self.assertNotIn('/gps/fix',[call.args[1] for call in self.node.create_publisher.call_args_list])
        self.assertNotIn('gps',self.store.values)
        self.feed.publisher.publish.assert_not_called()

    def test_stop_removes_virtual_position_and_stops_publishing(self):
        self.command('virtual_gps_start',point=[35.,129.]);self.command('virtual_gps_stop')
        count=self.feed.virtual_publisher.publish.call_count;self.feed.publish_virtual_gps()
        self.assertEqual(self.feed.virtual_publisher.publish.call_count,count)
        self.assertFalse(self.store.values['virtual_gps']['active'])

    def test_reject_invalid_points_before_queueing(self):
        for point in [None,[],[True,2],[91,0],[0,181],[float('nan'),0],['35',129]]:
            self.assertEqual(self.feed.request(dict(action='virtual_gps_start',point=point))[0],400)
        self.assertTrue(self.feed.queue.empty())

    def test_virtual_point_follow_selects_virtual_input_and_can_reposition_while_running(self):
        self.command('virtual_gps_start',point=[35.,129.])
        for action in ['start','start_model_test']:
            self.assertEqual(self.feed.request(dict(action=action))[0],409)
        self.command('start_point_follow')
        self.assertEqual(json.loads(self.feed.kind.call_args.kwargs['data'])['position_source'],'virtual')
        self.store.value('outdoor',{'execute':True,'navigation_mode':'point_follow','point_follow_position_source':'virtual'})
        with ThreadPoolExecutor(max_workers=1) as pool:
            future=pool.submit(self.feed.request,dict(action='virtual_gps_start',point=[35.001,129.]))
            deadline=time.monotonic()+1
            while self.feed.queue.empty() and time.monotonic()<deadline:time.sleep(.001)
            self.feed.work();self.assertEqual(future.result()[0],200)
        self.assertEqual(self.feed.virtual_point,[35.001,129.])
        self.command('virtual_gps_stop')
        self.assertEqual(self.feed.virtual_publisher.publish.call_args.args[0].status.status,-1)
        self.store.value('outdoor',{'execute':True})
        self.assertEqual(self.feed.request(dict(action='virtual_gps_start',point=[35.,129.]))[0],409)


if __name__=='__main__':unittest.main()
