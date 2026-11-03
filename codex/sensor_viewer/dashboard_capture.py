"""Manual PC-only dashboard captures; no sensor/control commands are sent."""
import base64
from datetime import datetime, timezone, timedelta
import io
import json
from pathlib import Path
import queue
import shutil
import subprocess
import threading
import time
import uuid
from fractions import Fraction

DEFAULT_ROOT=Path(r'C:\Users\Park\바탕화면\2026 활동\카봇 전국대회')/'대시보드기록'
KST=timezone(timedelta(hours=9))

def now_name():return datetime.now(KST).strftime('%Y%m%d_%H%M%S_%f')

def hidden():return {'creationflags':getattr(subprocess,'CREATE_NO_WINDOW',0)}


class NativeVideo:
    """One encoded frame per capture, with its original monotonic timestamp."""
    time_base=Fraction(1,1_000_000)

    def __init__(self,path):
        import av
        self.av=av;self.pending=None;self.stream=None;self.last_pts=-1
        self.container=av.open(str(path),'w',options={'movflags':'+frag_keyframe+empty_moov'})

    def write(self,jpeg,seconds):
        from PIL import Image, ImageOps
        with Image.open(io.BytesIO(jpeg)) as source:
            rgb=source.convert('RGB')
            rgb=ImageOps.expand(rgb,(0,0,rgb.width%2,rgb.height%2))
        if self.stream is None:
            self.stream=self.container.add_stream('libx264',rate=60)
            self.stream.width,self.stream.height=rgb.size
            self.stream.pix_fmt='yuv420p'
            self.stream.time_base=self.time_base
            self.stream.codec_context.time_base=self.time_base
            self.stream.options={'preset':'veryfast','crf':'14','tune':'zerolatency'}
        frame=self.av.VideoFrame.from_image(rgb)
        frame.pts=max(self.last_pts+1,round(seconds/self.time_base))
        frame.time_base=self.time_base;self.last_pts=frame.pts
        for packet in self.stream.encode(frame):self.packet(packet)

    def packet(self,packet):
        if self.pending is not None:
            self.pending.duration=max(1,round((packet.pts*packet.time_base-self.pending.pts*self.pending.time_base)/self.pending.time_base))
            self.container.mux(self.pending)
        self.pending=packet

    def close(self,seconds):
        try:
            if self.stream:
                for packet in self.stream.encode():self.packet(packet)
            if self.pending is not None:
                self.pending.duration=max(1,round(seconds/self.pending.time_base)-self.pending.pts)
                self.container.mux(self.pending);self.pending=None
        finally:self.container.close()


class CapturePage:
    def __init__(self,url,view):
        from selenium import webdriver
        from selenium.webdriver.support.ui import WebDriverWait
        options=webdriver.ChromeOptions()
        options.add_argument('--headless=new');options.add_argument('--hide-scrollbars')
        options.add_argument('--disable-background-timer-throttling')
        options.add_argument('--disable-renderer-backgrounding')
        options.add_argument('--enable-unsafe-swiftshader')
        self.driver=webdriver.Chrome(options=options)
        try:
            width=max(1100,min(3840,int(view.get('width',1600))))
            height=max(700,min(2160,int(view.get('height',1000))))
            scale=max(1.,min(2.,float(view.get('scale',1))))
            self.driver.execute_cdp_cmd('Emulation.setDeviceMetricsOverride',
                dict(width=width,height=height,deviceScaleFactor=scale,mobile=False))
            self.driver.set_page_load_timeout(30);self.driver.get(url+'/?capture=1')
            WebDriverWait(self.driver,20).until(lambda d:d.execute_script(
                "return !!window.kabotCaptureView && [...document.querySelectorAll('iframe')].every(f=>f.contentDocument?.readyState==='complete')"))
            self.driver.execute_script('window.kabotCaptureView(arguments[0])',view)
            time.sleep(.7)
            self.scale=scale;self.bounds=self.layout()
            # Capture a single compositor surface, rather than repainting a long
            # off-screen page and all canvases for every video frame.
            self.driver.execute_script("window.kabotCaptureLocked=true;for(const e of document.querySelectorAll('#outdoorOverview canvas,#outdoorOverview .map-stage'))e.style.height=e.getBoundingClientRect().height+'px';for(const e of document.querySelectorAll('#outdoorOverview>.panel')){e.style.height=e.getBoundingClientRect().height+'px';e.style.overflow='hidden';}")
            self.driver.execute_cdp_cmd('Emulation.setDeviceMetricsOverride',
                dict(width=width,height=int(self.bounds['height']),deviceScaleFactor=scale,mobile=False))
            time.sleep(.2)
        except Exception:self.close();raise

    def layout(self):
        return self.driver.execute_script("""
          const panels={};
          function add(name,e,ox=0,oy=0){if(!e)return;let r=e.getBoundingClientRect();if(r.width<2||r.height<2)return;panels[name]=[r.left+ox+scrollX,r.top+oy+scrollY,r.width,r.height];}
          const root=[['gps_map','.gps-map'],['response','.response-plots'],['models','.model-controls'],['comparison','.model-comparison']];
          for(const [name,sel] of root)add(name,document.querySelector(sel));
          for(const [frameId,items] of [['sensorFrame',[['lidar','.lidar-panel'],['imu','.imu-panel'],['scan','.scan-panel'],['gps','.gps-panel'],['actuator','.actuator-panel']]],['topicFrame',[['topics','.graph-panel']]]]){
            const f=document.getElementById(frameId),r=f.getBoundingClientRect();
            for(const [name,sel] of items)add(name,f.contentDocument.querySelector(sel),r.left,r.top);
          }
          for(const id of ['inputs','speed','yaw'])add('plot_'+id,document.getElementById(id).parentElement);
          return {width:document.documentElement.clientWidth,height:document.documentElement.scrollHeight,panels};
        """)

    def frame(self,fmt='png'):
        if fmt=='png':self.driver.execute_script("window.kabotCaptureDraw?.();document.getElementById('sensorFrame').contentWindow.kabotCaptureDraw?.()")
        box=self.bounds
        if box['width']*box['height']*self.scale**2>32_000_000:raise ValueError('촬영 화면이 너무 큽니다. 브라우저 폭을 줄여 주세요.')
        args=dict(format=fmt,captureBeyondViewport=False,fromSurface=True,
                  clip=dict(x=0,y=0,width=box['width'],height=box['height'],scale=1))
        if fmt=='jpeg':args['quality']=100
        return base64.b64decode(self.driver.execute_cdp_cmd('Page.captureScreenshot',args)['data'])

    def close(self):
        if self.driver:self.driver.quit();self.driver=None


class CaptureService:
    def __init__(self,url,root=DEFAULT_ROOT,page_factory=CapturePage):
        self.url=url;self.root=Path(root);self.page_factory=page_factory
        self.lock=threading.Lock();self.queue=queue.Queue(maxsize=3);self.stop_video=threading.Event()
        self.closing=threading.Event();self.thread=None
        self.info=dict(phase='idle',recording=False,error='',last_path='',capture_fps=0.,frame_timing='native')

    def status(self):
        with self.lock:return dict(self.info)

    def update(self,**kw):
        with self.lock:self.info.update(kw)

    def request(self,data):
        if not isinstance(data,dict) or data.get('action') not in ('screenshot','start','stop'):
            return 400,{'error':'알 수 없는 촬영 요청'}
        action=data['action'];view=data.get('view',{})
        if not isinstance(view,dict):return 400,{'error':'화면 설정 오류'}
        with self.lock:
            if self.closing.is_set():return 409,{'error':'촬영 프로그램 종료 중입니다'}
            if action=='stop':
                self.stop_video.set();return 202,{'accepted':True}
            if self.info['phase']!='idle':
                if action=='screenshot' and self.info['recording']:
                    try:self.queue.put_nowait(action);return 202,{'accepted':True}
                    except queue.Full:return 429,{'error':'사진 저장 중입니다'}
                return 409,{'error':'촬영 또는 영상 저장 중입니다'}
            self.info.update(phase='preparing',operation=action,error='');self.stop_video.clear()
            self.thread=threading.Thread(target=self.work,args=(action,view),daemon=True);self.thread.start()
        return 202,{'accepted':True}

    def directory(self,kind):
        target=self.root/datetime.now(KST).strftime('%Y-%m-%d')/(kind+'_'+now_name()+'_'+uuid.uuid4().hex[:6])
        target.mkdir(parents=True,exist_ok=False);self.check_space();return target

    def check_space(self):
        if shutil.disk_usage(self.root).free<2*1024**3:raise OSError('저장 공간이 2GB 미만입니다. 기존 파일은 유지합니다.')

    def screenshot(self,page):
        from PIL import Image
        folder=self.directory('사진');frame=Image.open(io.BytesIO(page.frame('png'))).convert('RGB')
        frame.save(folder/'전체.png',compress_level=6)
        scale=frame.width/page.bounds['width']
        for name,(x,y,w,h) in page.bounds['panels'].items():
            frame.crop(tuple(round(v*scale) for v in (x,y,x+w,y+h))).save(folder/(name+'.png'),compress_level=6)
        self.metadata(folder,kind='screenshot',pixels=list(frame.size),panels=page.bounds['panels'])
        self.update(last_path=str(folder))

    def metadata(self,folder,**values):
        payload=dict(captured_at=datetime.now(KST).isoformat(),**values)
        (folder/'capture.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf-8')

    def work(self,action,view):
        page=None
        try:
            page=self.page_factory(self.url,view)
            if action=='screenshot':self.screenshot(page)
            elif not self.stop_video.is_set():self.video(page)
        except Exception as exc:self.update(error=str(exc)[:350])
        finally:
            if page:
                try:page.close()
                except Exception:pass
            while not self.queue.empty():self.queue.get_nowait()
            self.update(phase='idle',recording=False)

    def video(self,page):
        from imageio_ffmpeg import get_ffmpeg_exe
        folder=self.directory('영상');partial=folder/'전체.recording.mp4';final=folder/'전체.mp4'
        exe=get_ffmpeg_exe();first=page.frame('jpeg');count=0;started=time.monotonic()
        writer=NativeVideo(partial)
        self.update(phase='recording',recording=True,last_path=str(folder),capture_fps=0.)
        try:
            while not self.stop_video.is_set() and not self.closing.is_set():
                tick=time.monotonic();self.check_space()
                frame=first if count==0 else page.frame('jpeg')
                stamp=0. if count==0 else time.monotonic()-started
                writer.write(frame,stamp);count+=1
                elapsed=max(.001,time.monotonic()-started)
                self.update(capture_fps=round(count/elapsed,1),elapsed_s=round(elapsed,1))
                if not self.queue.empty():self.queue.get_nowait();self.screenshot(page)
                self.stop_video.wait(max(0,1/60-(time.monotonic()-tick)))
        finally:
            self.update(phase='saving',recording=False)
            duration=time.monotonic()-started
            writer.close(duration)
            if count:
                self.finish_video(exe,partial,final,page,folder)
                self.metadata(folder,kind='video',frame_timing='native',unique_captures=count,duration_s=duration,
                    capture_fps=self.status()['capture_fps'],panels=page.bounds['panels'],
                    resolution_policy='native pixels; even-dimension padding only',codec='H.264 / yuv420p',
                    timing='variable frame rate; one frame per capture at its monotonic timestamp; no inserted frames')
                self.update(last_path=str(folder))

    def finish_video(self,exe,partial,final,page,folder):
        log=(folder/'encoding.log').open('ab')
        try:
            result=subprocess.run([exe,'-hide_banner','-loglevel','error','-y','-i',str(partial),'-c','copy',
                '-movflags','+faststart',str(final)],stdout=subprocess.DEVNULL,stderr=log,timeout=180,**hidden())
            if result.returncode:raise RuntimeError('전체 영상 마무리 실패. 원본 recording.mp4를 보존했습니다.')
            args=[exe,'-hide_banner','-loglevel','error','-y','-i',str(final)]
            for name,(x,y,w,h) in page.bounds['panels'].items():
                x,y,w,h=[max(0,round(v*page.scale)) for v in (x,y,w,h)]
                args+=['-map','0:v:0','-an','-vf',f'crop={w}:{h}:{x}:{y}:exact=1,pad=ceil(iw/2)*2:ceil(ih/2)*2',
                       '-c:v','libx264','-threads','1','-preset','veryfast','-crf','16','-pix_fmt','yuv420p','-bf','0',
                       '-fps_mode','passthrough','-enc_time_base','demux','-video_track_timescale','1000000',
                       '-movflags','+faststart',str(folder/(name+'.mp4'))]
            if page.bounds['panels']:
                result=subprocess.run(args,stdout=subprocess.DEVNULL,stderr=log,**hidden())
                if result.returncode:raise RuntimeError('일부 패널 영상 저장 실패. 전체 영상과 recording.mp4를 보존했습니다.')
            partial.unlink()
        finally:log.close()

    def close(self):
        self.closing.set();self.stop_video.set()
        if self.thread:
            while self.thread.is_alive():self.thread.join(timeout=.5)
