"""Isolated dashboard fixture: every command goes to an in-memory fake owner."""
from http.server import ThreadingHTTPServer
import json
import os
import threading
from types import SimpleNamespace
import unittest

from selenium import webdriver
from selenium.webdriver.common.action_chains import ActionChains
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait

from run_viewer import make_handler
from test_outdoor_browser import Bridge


class RouteEditor(unittest.TestCase):
    def test_ordered_route_save_reload_progress_and_running_guard(self):
        bridge=Bridge();args=SimpleNamespace(port=0)
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(bridge,args));args.port=server.server_port
        threading.Thread(target=server.serve_forever,daemon=True).start()
        options=webdriver.ChromeOptions();options.add_argument('--headless=new');options.add_argument('--window-size=1600,1200')
        options.set_capability('goog:loggingPrefs',{'browser':'ALL'})
        driver=webdriver.Chrome(options=options)
        try:
            # Prevent this fixture from connecting to the real PC's sensor feed.
            driver.execute_cdp_cmd('Page.addScriptToEvaluateOnNewDocument',{'source':"window.EventSource=class{};Object.defineProperty(navigator,'clipboard',{value:{writeText:async()=>{}}});"})
            driver.execute_cdp_cmd('Network.enable',{})
            driver.execute_cdp_cmd('Network.setBlockedURLs',{'urls':['*arcgisonline.com/*','*openstreetmap.org/*']})
            driver.get(f'http://127.0.0.1:{args.port}/?capture=1');wait=WebDriverWait(driver,12)
            wait.until(lambda d:d.find_element(By.ID,'startPathFollow').is_enabled())
            self.assertTrue(driver.find_element(By.CLASS_NAME,'route-editor').is_displayed())
            driver.find_element(By.ID,'clearPoints').click()
            canvas=driver.find_element(By.ID,'map')
            for x,y in [(-60,-30),(0,-30),(60,0)]:
                ActionChains(driver).move_to_element_with_offset(canvas,x,y).click().perform()
            wait.until(lambda d:len(d.find_elements(By.CSS_SELECTOR,'#points>.point'))==3)
            before=driver.execute_script("return JSON.parse(localStorage.getItem('outdoor.route.draft.v1')).points")
            self.assertEqual(bridge.actions,[])
            self.assertFalse(driver.find_element(By.ID,'startPathFollow').is_enabled())
            driver.find_element(By.CSS_SELECTOR,'#points>.point[data-index="2"] [data-action="up"]').click()
            expected=[before[0],before[2],before[1]]
            driver.refresh()
            wait.until(lambda d:len(d.find_elements(By.CSS_SELECTOR,'#points>.point'))==3)
            self.assertEqual(driver.execute_script("return JSON.parse(localStorage.getItem('outdoor.route.draft.v1')).points"),expected)
            wait.until(lambda d:d.find_element(By.ID,'savePoints').is_enabled())
            driver.find_element(By.ID,'savePoints').click()
            wait.until(lambda d:bridge.state['route']['points']==expected)
            wait.until(lambda d:d.find_element(By.ID,'startPathFollow').is_enabled())
            if os.environ.get('KABOT_TEST_SCREENSHOT'):
                driver.execute_script("document.querySelector('.gps-map').scrollIntoView({block:'start'})")
                driver.save_screenshot(os.environ['KABOT_TEST_SCREENSHOT'])
            driver.find_element(By.ID,'startPathFollow').click()
            wait.until(lambda d:bridge.state['execute'])
            self.assertEqual(bridge.state['route']['points'],expected)
            wait.until(lambda d:'1 / 3' in d.find_element(By.ID,'routeProgress').text)
            bridge.state['goal']=1
            wait.until(lambda d:'2 / 3' in d.find_element(By.ID,'routeProgress').text)
            bridge.state['goal']=2;bridge.state['point_follow']={'status':'목표 도착 · 위치 추적 중'}
            wait.until(lambda d:'전체 3지점 도착' in d.find_element(By.ID,'routeProgress').text)
            driver.find_element(By.ID,'clearPoints').click()
            ActionChains(driver).move_to_element_with_offset(driver.find_element(By.ID,'map'),0,0).click().perform()
            self.assertFalse(driver.find_element(By.ID,'savePoints').is_enabled())
            self.assertEqual(bridge.state['route']['points'],expected)
            self.assertEqual(bridge.actions,['route','start_point_follow'])
            errors=[x['message'] for x in driver.get_log('browser') if 'Uncaught' in x['message']]
            self.assertEqual(errors,[])
        finally:
            driver.quit();server.shutdown();server.server_close()


if __name__=='__main__':unittest.main()
