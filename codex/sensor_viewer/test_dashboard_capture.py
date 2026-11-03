"""PC-only media checks; generated fixtures stay in temporary directories."""
import io
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
import urllib.request
import urllib.error
from http.server import ThreadingHTTPServer
from PIL import Image, ImageDraw, ImageStat
from dashboard_capture import CaptureService, CapturePage, NativeVideo
from run_viewer import make_handler
from test_outdoor_browser import Bridge


class FakePage:
    def __init__(self,*_):
        self.scale=1;self.n=0
        self.bounds={'width':641,'height':361,'panels':{'panel':[20,20,301,181]}}
    def frame(self,fmt='png'):
        self.n+=1
        image=Image.new('RGB',(641,361),'#102030');d=ImageDraw.Draw(image)
        d.rectangle((20,20,320,200),fill=(self.n%200+30,160,70));d.text((25,25),str(self.n),fill='white')
        output=io.BytesIO();image.save(output,format='JPEG' if fmt=='jpeg' else 'PNG',quality=100)
        return output.getvalue()
    def close(self):pass


class MediaTests(unittest.TestCase):
    def wait_idle(self,service):
        deadline=time.monotonic()+50
        while service.status()['phase']!='idle' and time.monotonic()<deadline:time.sleep(.05)
        self.assertEqual(service.status()['phase'],'idle',service.status())
        self.assertFalse(service.status()['error'],service.status())

    def test_native_png_and_timed_h264_panel(self):
        import cv2
        with tempfile.TemporaryDirectory() as tmp:
            s=CaptureService('http://unused',tmp,FakePage)
            self.assertEqual(s.request({'action':'invalid'})[0],400)
            self.assertEqual(s.request({'action':'screenshot'})[0],202);self.wait_idle(s)
            folder=Path(s.status()['last_path'])
            with Image.open(folder/'전체.png') as im, Image.open(folder/'panel.png') as panel:
                self.assertEqual(im.size,(641,361))
                self.assertEqual(panel.size,(301,181))
                self.assertEqual(panel.tobytes(),im.crop((20,20,321,201)).tobytes())
            self.assertEqual(len(list(folder.glob('*.png'))),2)
            self.assertEqual(s.request({'action':'start'})[0],202)
            deadline=time.monotonic()+10
            while not s.status()['recording'] and time.monotonic()<deadline:time.sleep(.02)
            self.assertTrue(s.status()['recording'])
            self.assertEqual(s.request({'action':'start'})[0],409)
            time.sleep(2.2)
            s.request({'action':'stop'});self.wait_idle(s)
            folder=Path(s.status()['last_path'])
            frame_counts=[]
            for name,size in [('전체.mp4',(642,362)),('panel.mp4',(302,182))]:
                cap=cv2.VideoCapture(str(folder/name))
                try:
                    self.assertTrue(cap.isOpened(),name)
                    frame_counts.append(int(cap.get(cv2.CAP_PROP_FRAME_COUNT)))
                    self.assertEqual((int(cap.get(3)),int(cap.get(4))),size)
                    self.assertGreater(cap.get(cv2.CAP_PROP_FRAME_COUNT)/cap.get(cv2.CAP_PROP_FPS),1.5)
                    self.assertLess(cap.get(cv2.CAP_PROP_FRAME_COUNT)/cap.get(cv2.CAP_PROP_FPS),3.5)
                    self.assertIn(int(cap.get(cv2.CAP_PROP_FOURCC)),[cv2.VideoWriter_fourcc(*v) for v in ('avc1','h264','H264')])
                    ok,frame=cap.read();self.assertTrue(ok);self.assertGreater(frame.std(),10)
                finally:cap.release()
            self.assertEqual(frame_counts[0],frame_counts[1])
            self.assertFalse((folder/'전체.recording.mp4').exists())
            meta=json.loads((folder/'capture.json').read_text(encoding='utf-8'))
            self.assertEqual(meta['frame_timing'],'native');self.assertGreater(meta['unique_captures'],1)
            self.assertEqual(frame_counts,[meta['unique_captures']]*2)
            s.close()

    def test_irregular_capture_timestamps_survive_panel_export(self):
        import av
        from imageio_ffmpeg import get_ffmpeg_exe
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);page=FakePage();partial=folder/'recording.mp4';final=folder/'전체.mp4'
            times=[0.,.037,.2,.73,1.4]
            writer=NativeVideo(partial)
            for stamp in times:writer.write(page.frame('jpeg'),stamp)
            writer.close(1.8)
            CaptureService('',tmp).finish_video(get_ffmpeg_exe(),partial,final,page,folder)
            for path in (final,folder/'panel.mp4'):
                with av.open(str(path)) as video:
                    frames=list(video.decode(video=0))
                    self.assertEqual(len(frames),len(times),path.name)
                    for frame,stamp in zip(frames,times):self.assertAlmostEqual(float(frame.pts*frame.time_base),stamp,places=5)
                    self.assertAlmostEqual(float(video.streams.video[0].duration*video.streams.video[0].time_base),1.8,delta=.05)

    def test_real_dashboard_snapshot_and_no_control_requests(self):
        bridge=Bridge();args=SimpleNamespace(port=0)
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(bridge,args));args.port=server.server_port
        threading.Thread(target=server.serve_forever,daemon=True).start()
        page=None
        try:
            page=CapturePage(f'http://127.0.0.1:{args.port}',dict(width=1440,height=1000,scale=1,tiles=False))
            self.assertFalse(page.driver.find_element('id','captureTools').is_displayed())
            self.assertIn('42.0',page.driver.find_element('id','position').text)
            self.assertGreaterEqual(len(page.bounds['panels']),10)
            self.assertEqual(page.driver.execute_script('return getComputedStyle(document.body).backgroundColor'),'rgb(25, 27, 29)')
            self.assertEqual(page.driver.execute_script("const d=document.getElementById('sensorFrame').contentDocument;return d.defaultView.getComputedStyle(d.querySelector('.panel')).backgroundColor"),'rgb(37, 40, 43)')
            with tempfile.TemporaryDirectory() as tmp:
                s=CaptureService('',tmp);s.screenshot(page);folder=Path(s.status()['last_path'])
                with Image.open(folder/'전체.png') as im:
                    self.assertEqual(im.width,1440)
                    self.assertGreater(im.height,1000)
                    self.assertGreater(max(ImageStat.Stat(im).stddev),5)
                self.assertEqual(len(list(folder.glob('*.png'))),1+len(page.bounds['panels']))
                # Keep one local review image outside the user's presentation folder.
                review=Path(tempfile.gettempdir())/'kabot-capture-review.png'
                review.write_bytes((folder/'전체.png').read_bytes())
                timer=threading.Timer(3,s.stop_video.set);timer.start()
                s.video(page);timer.join()
                video=Path(s.status()['last_path'])
                self.assertEqual(len(list(video.glob('*.mp4'))),1+len(page.bounds['panels']))
                self.assertGreater((video/'전체.mp4').stat().st_size,10000)
                print('Native dashboard recording:',s.status()['capture_fps'],'unique fps')
            self.assertEqual(bridge.actions,[])
        finally:
            if page:page.close()
            server.shutdown();server.server_close()

    def test_capture_api_stays_local_and_checks_origin(self):
        bridge=Bridge();args=SimpleNamespace(port=0)
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(bridge,args));args.port=server.server_port
        threading.Thread(target=server.serve_forever,daemon=True).start()
        url=f'http://127.0.0.1:{args.port}'
        try:
            with tempfile.TemporaryDirectory() as tmp:
                bridge.capture=CaptureService(url,tmp,FakePage)
                body=json.dumps({'action':'screenshot'}).encode()
                request=urllib.request.Request(url+'/api/capture',data=body,headers={'Origin':'http://outside'})
                with self.assertRaises(urllib.error.HTTPError) as error:urllib.request.urlopen(request)
                self.assertEqual(error.exception.code,403)
                request=urllib.request.Request(url+'/api/capture',data=body,headers={'Origin':url})
                with urllib.request.urlopen(request) as response:self.assertEqual(response.status,202)
                self.wait_idle(bridge.capture)
                with urllib.request.urlopen(url+'/api/capture') as response:
                    self.assertEqual(json.load(response)['phase'],'idle')
                self.assertEqual(bridge.actions,[]);bridge.capture.close()
        finally:server.shutdown();server.server_close()


if __name__=='__main__':unittest.main()
