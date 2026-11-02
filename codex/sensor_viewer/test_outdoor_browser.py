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
        if action=='route':self.state['route']={'points':value['points']}
        if action=='start':self.state['execute']=True
        if action=='stop':self.state['execute']=False
        return 202,{'Content-Type':'application/json'},json.dumps({'request_id':'fixture-request'}).encode()


class OutdoorBrowser(unittest.TestCase):
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
            self.assertEqual(len(driver.find_elements(By.CSS_SELECTOR,'[data-layer]')),10)
            before=list(bridge.actions)
            for checkbox in driver.find_elements(By.CSS_SELECTOR,'[data-layer]'): checkbox.click()
            self.assertEqual(before,bridge.actions)
            self.assertTrue(driver.execute_script("return Object.values(JSON.parse(localStorage.getItem('outdoor.layers.v1'))).every(v=>v===false)"))
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
            driver.find_element(By.ID,'applyModel').click();wait.until(lambda d:'전체·비선형' in d.find_element(By.ID,'validationState').text)
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
            driver.find_element(By.ID,'savePoints').click();wait.until(lambda d:len(bridge.state['route']['points'])==3)
            for id in ['inputs','speed','yaw','ranking','errorsSpeed','errorsHeading','outputBands']:
                count=driver.execute_script("const c=document.getElementById(arguments[0]),p=c.getContext('2d').getImageData(0,0,c.width,c.height).data;let n=0;for(let i=3;i<p.length;i+=4)if(p[i])n++;return n",id)
                self.assertGreater(count,100,id)
            artifacts=Path(tempfile.gettempdir())/'kabot-outdoor-review';artifacts.mkdir(exist_ok=True)
            driver.execute_script('window.scrollTo(0,0)');driver.save_screenshot(str(artifacts/'desktop.png'))
            driver.execute_script('window.scrollTo(0,document.body.scrollHeight)');driver.save_screenshot(str(artifacts/'graphs.png'))
            driver.find_element(By.ID,'clearTrail').click();wait.until(lambda d:not bridge.state['trail'])
            self.assertEqual(bridge.state['valid_frames'],900)
            failures=[v for v in driver.get_log('browser') if v['level']=='SEVERE' and 'tile.openstreetmap.org' not in v['message'] and 'favicon' not in v['message']]
            self.assertFalse(failures,failures)
            print('Screenshots:',artifacts)
        finally:driver.quit();server.shutdown();server.server_close()


if __name__=='__main__':unittest.main()
