"""Local browser fixture: no ROS, SSH or propulsion connections."""
import gzip
from copy import deepcopy
from http.server import ThreadingHTTPServer
import json
import math
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from run_viewer import make_handler


def sample():
    rows=[]
    for i in range(180):
        t=i*.3;u=.35+.2*math.sin(i/20);v=.08*math.sin(i/12);r=.15*math.cos(i/20)
        rows.append(dict(t=t,command=[.25+.15*math.sin(i/20),.2+.2*math.cos(i/18)],measured=[u,v,r],predicted=[u*.95,v*.8,r*1.03]))
    return dict(state=dict(lat=35.232,lon=129.083,heading_deg=42,position_sigma_m=.5),state_reason='',
        model=dict(id='0123456789abcdef',name='전체·비선형',candidate_name='전체·비선형',qualified=True,delay_steps=2,dt_s=.2,native_period_s=.2,
            speed_coverage={'mean_m_s':.34},recording_segments=3,training_samples=900,candidate_count=2,
            candidate_catalog=[dict(name='전체·비선형',delay_s=.4,score=.02,selected=True,incumbent=False),dict(name='균형·선형',delay_s=.2,score=.09,selected=False,incumbent=False)],
            output_coverage={'mean_percent':32.,'peak_band':{'low_percent':30,'high_percent':40,'seconds':85.,'share_percent':55.}},
            ranking=[dict(label='전체·비선형 0.4s',score=.02),dict(label='균형·선형 0.2s',score=.09)],
            metrics=dict(speed_bands=[dict(low_m_s=.2,high_m_s=.4,windows=8,score=.02)],best_speed_band=dict(low_m_s=.2,high_m_s=.4,windows=8,score=.02),output_bands=[dict(low_percent=i*10,high_percent=(i+1)*10,windows=8,score=.01*(i+1)) for i in range(7)],
                best_output_band=dict(low_percent=0,high_percent=10,windows=8,score=.01),
                horizons={str(i):dict(velocity_rmse=[.02*i,.01*i,.02],heading_rmse_deg=.8*i) for i in (1,2,3)})),
        route=dict(points=[[35.23205,129.083],[35.2321,129.0831]]),goal=0,arrivals=0,execute=False,
        learning=True,frozen=False,fit_running=False,models=[dict(id='0123456789abcdef',label='20261102 074012123456 32%',name='32%',compatible=True,finalized=True)],mode='RC',
        origin=[35.232,129.083],guidance_raw=[[35.232,129.083],[35.23204,129.083],[35.23204,129.08304]],
        map_layers={'ttl_s':20,'hits':[[i*.2,5.] for i in range(-50,50)],
                    'accumulated':[[i*.2,7.,8.] for i in range(-50,50)]},
        recording_summary=dict(segments=3,samples=1000),valid_frames=900,native_frames=1000,bag='recording',error='',ack=None,age_s=0,history=rows,
        trail=[[35.232-.0001+i*.000001,129.083+.00004*math.sin(i/20),'segment',i] for i in range(100)],
        preview=[[35.232+i*.000005,129.083+i*.000002] for i in range(8)],prediction=[],
        guidance=[[35.232,129.083],[35.23203,129.08302],[35.23205,129.083]],
        obstacle_tracks=[dict(id=1,path=[[35.23203,129.08303+i*.000005] for i in range(4)],
                             radius_m=.4,speed_m_s=.5,confirmed=True)])


class Bridge:
    def __init__(self):self.state=sample();self.browser=SimpleNamespace(viewed=lambda:None);self.actions=[]
    def metadata(self):return dict(status='fixture',sensors_running=True)
    def request(self, method, path, body=None):
        if method=='GET':
            return 200,{'Content-Type':'application/json'},json.dumps({'values':{'outdoor':self.state}}).encode()
        value=json.loads(body);action=value['action'];self.state['ack']=dict(id='fixture-request',ok=True,message='적용됨')
        self.actions.append(action)
        if action=='pause':self.state['learning']=False
        if action=='resume':self.state.update(learning=True)
        if action=='load':self.state.update(learning=False,frozen=True,learning_branch=None)
        if action=='apply_model':self.state['validation_model']=deepcopy(self.state['model'])
        if action=='clear_trail':self.state['trail']=[]
        if action=='route':
            self.state['route']={'points':value['points']}
            if getattr(self,'replace_route_ack',False):self.state['ack']={'id':'other-command','ok':True}
        if action=='regions':self.state['regions']=value['regions']
        if action=='start':self.state['execute']=True
        if action=='start_point_follow':self.state.update(execute=True,navigation_mode='point_follow',point_follow_tracking=value.get('tracking','point'),point_follow_limit_percent=value['percent'],point_follow_position_source=value.get('position_source','gps'))
        if action=='point_follow_limit':self.state['point_follow_limit_percent']=value['percent']
        if action=='start_model_test':self.state['execute']=True;self.state['model_test']=dict(running=True,navigation='saved_route',origin=[35.232,129.083],area=None,obstacles=[],metrics=dict(samples=8,rmse_m=.25,max_error_m=.7));self.test_area=None
        if action=='stop':
            self.state['execute']=False
            if self.state.get('model_test'):self.state['model_test']['running']=False
        return 202,{'Content-Type':'application/json'},json.dumps({'request_id':'fixture-request'}).encode()


class OutdoorBrowser(unittest.TestCase):
    def test_path_follow_button_cap_and_shared_stop(self):
        bridge=Bridge();args=SimpleNamespace(port=0)
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(bridge,args));args.port=server.server_port
        threading.Thread(target=server.serve_forever,daemon=True).start()
        options=webdriver.ChromeOptions();options.add_argument('--headless=new');options.add_argument('--window-size=1600,1100')
        driver=webdriver.Chrome(options=options)
        try:
            driver.get(f'http://127.0.0.1:{args.port}/');wait=WebDriverWait(driver,12)
            wait.until(lambda d:d.find_element(By.ID,'startPathFollow').is_enabled())
            self.assertTrue(driver.find_element(By.ID,'startPointFollow').is_enabled())
            driver.find_element(By.ID,'startPathFollow').click()
            wait.until(lambda d:bridge.state.get('point_follow_tracking')=='path' and bridge.state['execute'])
            wait.until(lambda d:d.find_element(By.ID,'startPathFollow').text=='경로 추종 중')
            self.assertFalse(driver.find_element(By.ID,'startPointFollow').is_enabled())
            self.assertTrue(driver.find_element(By.ID,'stopPathFollow').is_enabled())
            self.assertIn('파랑',driver.find_element(By.CSS_SELECTOR,'.map-legend').text)
            driver.execute_script("const e=document.getElementById('percent');e.value='15';e.dispatchEvent(new Event('change',{bubbles:true}));")
            wait.until(lambda d:bridge.state.get('point_follow_limit_percent')==15)
            driver.find_element(By.ID,'stopPathFollow').click();wait.until(lambda d:not bridge.state['execute'])
            wait.until(lambda d:d.find_element(By.ID,'startPathFollow').is_enabled())
            self.assertEqual(bridge.actions,['start_point_follow','point_follow_limit','stop'])
        finally:driver.quit();server.shutdown();server.server_close()

    def test_pc_map_without_gps_and_virtual_gps_toggle(self):
        class GeoBridge(Bridge):
            def __init__(self):
                super().__init__();self.state['state']=None;self.state['route']={'points':[]}
                self.virtual={'active':False,'age_s':0}
                self.no_outdoor=False
            def request(self, method, path, body=None):
                if method=='GET':
                    values={'virtual_gps':self.virtual,'heading':{'heading_deg':42.,'age_s':0},'map_reference':{'point':[35.24,129.09]}}
                    if not self.no_outdoor:values['outdoor']=self.state
                    return 200,{'Content-Type':'application/json'},json.dumps({'values':values}).encode()
                value=json.loads(body)
                if value['action'].startswith('virtual_gps_'):
                    active=value['action']=='virtual_gps_start'
                    self.virtual=dict(active=active,point=value.get('point'),age_s=0,topic='/gps/virtual/fix')
                    self.actions.append(value['action'])
                    return 200,{'Content-Type':'application/json'},json.dumps(dict(simulation=True,active=active)).encode()
                return super().request(method,path,body)
        bridge=GeoBridge();args=SimpleNamespace(port=0)
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(bridge,args));args.port=server.server_port
        threading.Thread(target=server.serve_forever,daemon=True).start()
        options=webdriver.ChromeOptions();options.add_argument('--headless=new');options.add_argument('--window-size=1600,1100')
        driver=webdriver.Chrome(options=options)
        try:
            driver.execute_cdp_cmd('Page.addScriptToEvaluateOnNewDocument',{'source':"Object.defineProperty(navigator,'geolocation',{configurable:true,value:{getCurrentPosition:ok=>ok({coords:{latitude:35.23,longitude:129.08,accuracy:40}})}});Object.defineProperty(navigator,'clipboard',{value:{writeText:async()=>{}}});"})
            driver.get(f'http://127.0.0.1:{args.port}/');wait=WebDriverWait(driver,12)
            wait.until(lambda d:d.execute_script('return window.kabotCaptureView?.().center?.[0]===35.23'))
            self.assertIn('PC',driver.find_element(By.ID,'position').text)
            driver.execute_script("const c=document.getElementById('map'),r=c.getBoundingClientRect(),p={clientX:r.left+r.width/2,clientY:r.top+r.height/2,pointerId:1,bubbles:true};c.dispatchEvent(new PointerEvent('pointerdown',p));c.dispatchEvent(new PointerEvent('pointerup',p));")
            self.assertFalse(bridge.virtual['active'])
            wait.until(lambda d:len(bridge.state['route']['points'])==1)
            self.assertAlmostEqual(bridge.state['route']['points'][0][0],35.23)
            driver.find_element(By.ID,'virtualGps').click()
            self.assertFalse(bridge.virtual['active'])
            driver.execute_script("const c=document.getElementById('map'),r=c.getBoundingClientRect(),p={clientX:r.left+r.width/2,clientY:r.top+r.height/2,pointerId:1,bubbles:true};c.dispatchEvent(new PointerEvent('pointerdown',p));c.dispatchEvent(new PointerEvent('pointerup',p));")
            wait.until(lambda d:bridge.virtual['active'])
            self.assertAlmostEqual(bridge.virtual['point'][0],35.23)
            wait.until(lambda d:d.find_element(By.ID,'stopVirtualGps').is_enabled())
            wait.until(lambda d:d.execute_script('return window.kabotCaptureView().selectedCoordinate===null'))
            self.assertIn('/gps/virtual/fix',driver.find_element(By.ID,'virtualGpsInfo').get_attribute('title'))
            self.assertIn('가상 GPS',driver.find_element(By.ID,'position').text)
            bridge.state['route']={'points':[[35.2301,129.08]]}
            wait.until(lambda d:d.find_element(By.ID,'startPointFollow').is_enabled())
            driver.find_element(By.ID,'startPointFollow').click()
            wait.until(lambda d:bridge.state.get('point_follow_position_source')=='virtual')
            previous=list(bridge.virtual['point'])
            driver.execute_script("const c=document.getElementById('map'),r=c.getBoundingClientRect(),p={clientX:r.left+r.width*.6,clientY:r.top+r.height*.4,pointerId:1,bubbles:true};c.dispatchEvent(new PointerEvent('pointerdown',p));c.dispatchEvent(new PointerEvent('pointerup',p));")
            wait.until(lambda d:bridge.virtual['point']!=previous)
            wait.until(lambda d:d.execute_script('return window.kabotCaptureView().selectedCoordinate===null'))
            driver.find_element(By.ID,'stopVirtualGps').click();wait.until(lambda d:not bridge.virtual['active'])
            driver.find_element(By.ID,'stop').click();wait.until(lambda d:not bridge.state['execute'])
            self.assertEqual([a for a in bridge.actions if a.startswith('virtual_gps_')],['virtual_gps_start','virtual_gps_start','virtual_gps_stop'])
            self.assertTrue(bridge.state['learning'])
            bridge.no_outdoor=True
            driver.execute_script("localStorage.removeItem('outdoor.map.center')")
            driver.execute_cdp_cmd('Page.addScriptToEvaluateOnNewDocument',{'source':"Object.defineProperty(navigator,'geolocation',{value:{getCurrentPosition:(ok,fail)=>fail()}});"})
            driver.refresh()
            wait.until(lambda d:d.execute_script('return window.kabotCaptureView?.().center?.[0]===35.24'))
            self.assertFalse(driver.find_element(By.ID,'start').is_enabled())
        finally:driver.quit();server.shutdown();server.server_close()

    def test_desktop_graphs_map_and_controls(self):
        bridge=Bridge();args=SimpleNamespace(port=0)
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(bridge,args));args.port=server.server_port
        threading.Thread(target=server.serve_forever,daemon=True).start()
        options=webdriver.ChromeOptions();options.add_argument('--headless=new');options.add_argument('--window-size=1440,1200');options.add_argument('--disable-background-networking')
        options.set_capability('goog:loggingPrefs',{'browser':'ALL'})
        driver=webdriver.Chrome(options=options)
        try:
            driver.get(f'http://127.0.0.1:{args.port}/');wait=WebDriverWait(driver,15)
            wait.until(lambda d:'42.0' in d.find_element(By.ID,'position').text)
            wait.until(lambda d:d.find_element(By.ID,'startPointFollow').is_enabled())
            driver.find_element(By.ID,'startPointFollow').click()
            wait.until(lambda d:bridge.state.get('navigation_mode')=='point_follow' and bridge.state['execute'])
            self.assertEqual(bridge.state['point_follow_limit_percent'],20)
            wait.until(lambda d:d.find_element(By.ID,'startPointFollow').text=='지점 추종 중')
            driver.execute_script("const e=document.getElementById('percent');e.value='30';e.dispatchEvent(new Event('change',{bubbles:true}));")
            wait.until(lambda d:bridge.state['point_follow_limit_percent']==30)
            driver.find_element(By.ID,'stop').click();wait.until(lambda d:not bridge.state['execute'])
            driver.execute_script("window.testCopied=[];Object.defineProperty(navigator,'clipboard',{configurable:true,value:{writeText:async text=>window.testCopied.push(text)}})")
            driver.execute_script("const c=document.getElementById('map'),r=c.getBoundingClientRect();const p={clientX:r.left+r.width/2,clientY:r.top+r.height/2,pointerId:1,bubbles:true};c.dispatchEvent(new PointerEvent('pointermove',p));c.dispatchEvent(new PointerEvent('pointerdown',p));c.dispatchEvent(new PointerEvent('pointerup',p));")
            wait.until(lambda d:'복사됨' in d.find_element(By.ID,'copyCoordinate').text)
            self.assertEqual(driver.execute_script('return window.testCopied[0]'),'35.2320000, 129.0830000')
            driver.execute_script("const c=document.getElementById('map'),r=c.getBoundingClientRect();const p={clientX:r.left+100,clientY:r.top+100,pointerId:1,bubbles:true};c.dispatchEvent(new PointerEvent('pointerdown',p));p.clientX+=30;c.dispatchEvent(new PointerEvent('pointermove',p));c.dispatchEvent(new PointerEvent('pointerup',p));")
            self.assertEqual(driver.execute_script('return window.testCopied.length'),1)
            driver.find_element(By.ID,'follow').click()
            self.assertEqual(driver.title,'KABOAT 2026')
            self.assertFalse(driver.find_elements(By.TAG_NAME,'header'))
            self.assertFalse(driver.find_elements(By.ID,'demoBanner'))
            self.assertFalse(driver.find_elements(By.ID,'bringupToggle'))
            driver.find_element(By.ID,'tiles').click()
            self.assertIn('42.0',driver.find_element(By.ID,'position').text)
            self.assertIn('0.34 m/s',driver.find_element(By.ID,'learningSpeed').text)
            self.assertIn('0.2–0.4 m/s',driver.find_element(By.ID,'learningSpeed').text)
            self.assertIn('3구간',driver.find_element(By.ID,'recordCount').text)
            driver.find_element(By.ID,'candidateSummary').click()
            self.assertEqual(len(driver.find_elements(By.CSS_SELECTOR,'#candidateRows tr')),2)
            self.assertIn('전체·비선형',driver.find_element(By.CSS_SELECTOR,'#candidateRows tr.selected').text)
            self.assertEqual([e.get_attribute('data-layer') for e in driver.find_elements(By.CSS_SELECTOR,'[data-layer]')],['boat','trail','waypoints'])
            def current_zoom():
                return driver.execute_script('return window.kabotCaptureView().zoom')
            z=current_zoom()
            driver.find_element(By.ID,'zoomIn').click()
            self.assertEqual(current_zoom(),z+1)
            driver.find_element(By.ID,'zoomOut').click()
            self.assertEqual(current_zoom(),z)
            self.assertEqual(driver.find_element(By.ID,'zoomOut').text,'−')
            # Old overlay preferences and new survey frames cannot restore debug layers.
            driver.execute_script("localStorage.setItem('outdoor.layers.v1',JSON.stringify({raw:true,survey:true,prediction:true}));")
            bridge.state['map_layers']['survey_id']='new-survey'
            bridge.state['map_layers']['survey']=[[0,1],[1,2]]
            time.sleep(1)
            self.assertEqual(len(driver.find_elements(By.CSS_SELECTOR,'[data-layer]')),3)
            self.assertNotIn('A*',driver.find_element(By.ID,'layers').text)
            driver.execute_script("window.kabotCaptureView({center:arguments[0],zoom:19,follow:true,layers:{boat:true,trail:true,waypoints:true}})",bridge.state['origin'])
            before=list(bridge.actions)
            for checkbox in driver.find_elements(By.CSS_SELECTOR,'[data-layer]'): checkbox.click()
            self.assertEqual(before,bridge.actions)
            self.assertTrue(driver.execute_script("return Object.values(JSON.parse(localStorage.getItem('outdoor.layers.core.v2'))).every(v=>v===false)"))
            driver.refresh();wait.until(lambda d:'42.0' in d.find_element(By.ID,'position').text)
            self.assertEqual(driver.title,'KABOAT 2026')
            self.assertFalse(driver.find_elements(By.TAG_NAME,'header'))
            self.assertFalse(driver.find_elements(By.ID,'demoBanner'))
            self.assertFalse(driver.find_elements(By.ID,'bringupToggle'))
            self.assertTrue(all(not e.is_selected() for e in driver.find_elements(By.CSS_SELECTOR,'[data-layer]')))
            driver.find_element(By.ID,'tiles').click()
            for checkbox in driver.find_elements(By.CSS_SELECTOR,'[data-layer]'): checkbox.click()
            self.assertFalse(driver.find_elements(By.ID,'models'))
            self.assertFalse(driver.find_elements(By.ID,'load'))
            bridge.state['model']['qualified']=False
            bridge.state['model']['reasons']=['분리한 검증 기록 부족']
            wait.until(lambda d:'수동 적용 가능' in d.find_element(By.ID,'applyModel').get_attribute('title'))
            self.assertTrue(driver.find_element(By.ID,'applyModel').is_enabled())
            driver.find_element(By.ID,'applyModel').click();wait.until(lambda d:'전체·비선형' in d.find_element(By.ID,'validationState').text)
            self.assertFalse(bridge.state['validation_model']['qualified'])
            self.assertTrue(bridge.state['learning'])
            self.assertIn('0.34 m/s',driver.find_element(By.ID,'validationSpeed').text)
            bridge.state['model']=deepcopy(bridge.state['model']);bridge.state['model'].update(id='new-model',candidate_name='최근·선형')
            wait.until(lambda d:d.find_element(By.ID,'modelState').text=='최근·선형')
            self.assertEqual(driver.find_element(By.ID,'validationState').text,'전체·비선형')
            self.assertTrue(driver.execute_script("return [...document.querySelectorAll('.model-spec')].every(e=>e.scrollWidth<=e.clientWidth)"))
            driver.find_element(By.ID,'pause').click();wait.until(lambda d:'학습 중지' in d.find_element(By.ID,'learningState').text)
            self.assertTrue(bridge.state['trail'])
            driver.find_element(By.ID,'resume').click();wait.until(lambda d:'학습 중' in d.find_element(By.ID,'learningState').text)
            driver.find_element(By.ID,'start').click();wait.until(lambda d:'실행 중' in d.find_element(By.ID,'trial').text)
            driver.find_element(By.ID,'stop').click();wait.until(lambda d:d.find_element(By.ID,'trial').text=='정지')
            driver.find_element(By.ID,'clearPoints').click();driver.find_element(By.ID,'addPoint').click()
            driver.execute_script("const c=document.getElementById('map'),r=c.getBoundingClientRect();for(const x of [r.width*.25,r.width*.5,r.width*.7]){c.dispatchEvent(new PointerEvent('pointerdown',{clientX:r.left+x,clientY:r.top+r.height*.5,pointerId:1,bubbles:true}));c.dispatchEvent(new PointerEvent('pointerup',{clientX:r.left+x,clientY:r.top+r.height*.5,pointerId:1,bubbles:true}));}")
            wait.until(lambda d:len(d.find_elements(By.CSS_SELECTOR,'.point'))==3)
            self.assertFalse(driver.find_element(By.ID,'start').is_enabled())
            self.assertIn('편집한 경로',driver.find_element(By.ID,'start').get_attribute('title'))
            bridge.replace_route_ack=True
            driver.find_element(By.ID,'savePoints').click();wait.until(lambda d:len(bridge.state['route']['points'])==3)
            # Saved coordinates restore start controls even if another command
            # replaced the ACK before the next dashboard poll.
            wait.until(lambda d:d.find_element(By.ID,'start').is_enabled())
            self.assertTrue(driver.find_element(By.ID,'startModelTest').is_enabled())
            self.assertIn('저장됨 · 3점',driver.find_element(By.ID,'savePoints').text)
            # A delayed success ACK must not erase a newer local edit.
            driver.find_element(By.ID,'undoPoint').click()
            bridge.state['ack']={'id':'fixture-request','ok':True}
            time.sleep(1)
            self.assertEqual(len(driver.find_elements(By.CSS_SELECTOR,'.point')),2)
            self.assertFalse(driver.find_element(By.ID,'start').is_enabled())
            driver.find_element(By.ID,'savePoints').click()
            wait.until(lambda d:len(bridge.state['route']['points'])==2 and d.find_element(By.ID,'start').is_enabled())
            for id in ['inputs','speed','yaw','ranking','errorsSpeed','errorsHeading','outputBands']:
                count=driver.execute_script("const c=document.getElementById(arguments[0]),p=c.getContext('2d').getImageData(0,0,c.width,c.height).data;let n=0;for(let i=3;i<p.length;i+=4)if(p[i])n++;return n",id)
                self.assertGreater(count,100,id)
            artifacts=Path(tempfile.gettempdir())/'kabot-outdoor-review';artifacts.mkdir(exist_ok=True)
            driver.execute_script('window.scrollTo(0,0)');driver.save_screenshot(str(artifacts/'desktop.png'))
            driver.execute_script('window.scrollTo(0,document.body.scrollHeight)');driver.save_screenshot(str(artifacts/'graphs.png'))
            driver.find_element(By.ID,'defineTestArea').click()
            driver.execute_script("const c=document.getElementById('map'),r=c.getBoundingClientRect();for(const [x,y] of [[.2,.2],[.8,.2],[.8,.8],[.2,.8]]){const p={clientX:r.left+r.width*x,clientY:r.top+r.height*y,pointerId:1,bubbles:true};c.dispatchEvent(new PointerEvent('pointerdown',p));c.dispatchEvent(new PointerEvent('pointerup',p));}")
            driver.find_element(By.ID,'addTestRegion').click()
            self.assertTrue(driver.find_element(By.ID,'startModelTest').is_enabled())
            from selenium.webdriver.support.ui import Select
            Select(driver.find_element(By.ID,'regionKind')).select_by_value('exclude')
            driver.find_element(By.ID,'defineTestArea').click()
            driver.execute_script("const c=document.getElementById('map'),r=c.getBoundingClientRect();for(const [x,y] of [[.4,.4],[.5,.4],[.5,.5],[.4,.5]]){const p={clientX:r.left+r.width*x,clientY:r.top+r.height*y,pointerId:1,bubbles:true};c.dispatchEvent(new PointerEvent('pointerdown',p));c.dispatchEvent(new PointerEvent('pointerup',p));}")
            driver.find_element(By.ID,'addTestRegion').click()
            driver.find_element(By.ID,'saveTestRegions').click()
            wait.until(lambda d:len(bridge.state.get('regions',[]))==2)
            self.assertEqual(bridge.state['regions'][1]['kind'],'exclude')
            driver.refresh();wait.until(lambda d:len(d.find_elements(By.CSS_SELECTOR,'.region-item'))==2)
            wait.until(lambda d:d.find_element(By.ID,'startModelTest').is_enabled())
            driver.find_element(By.ID,'startModelTest').click();wait.until(lambda d:'시험 중' in d.find_element(By.ID,'testInfo').text)
            self.assertEqual(driver.find_element(By.ID,'startModelTest').text,'SLAM 주행 중')
            self.assertFalse(driver.find_element(By.ID,'startModelTest').is_enabled())
            self.assertEqual(driver.execute_script("return getComputedStyle(document.getElementById('startModelTest')).opacity"),'1')
            self.assertIsNone(bridge.test_area);self.assertTrue(bridge.state['learning'])
            self.assertIn('0.25m',driver.find_element(By.ID,'testInfo').text)
            driver.find_element(By.ID,'stop').click();wait.until(lambda d:'시험 종료' in d.find_element(By.ID,'testInfo').text)
            driver.find_element(By.ID,'clearTrail').click();wait.until(lambda d:not bridge.state['trail'])
            self.assertEqual(bridge.state['valid_frames'],900)
            failures=[v for v in driver.get_log('browser') if v['level']=='SEVERE' and 'tile.openstreetmap.org' not in v['message'] and 'favicon' not in v['message']]
            self.assertFalse(failures,failures)
            print('Screenshots:',artifacts)
        finally:driver.quit();server.shutdown();server.server_close()


if __name__=='__main__':unittest.main()
