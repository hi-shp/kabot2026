"""Reuse a dedicated dashboard window on Windows; open normally elsewhere."""
import ctypes
from ctypes import wintypes
import os
from pathlib import Path
import subprocess
import threading
import time
import webbrowser


def focus_dashboard():
    if os.name != 'nt': return False
    user32=ctypes.WinDLL('user32',use_last_error=True)
    kernel32=ctypes.WinDLL('kernel32',use_last_error=True)
    user32.GetForegroundWindow.restype=wintypes.HWND
    user32.GetWindowThreadProcessId.argtypes=[wintypes.HWND,ctypes.POINTER(wintypes.DWORD)]
    user32.GetWindowThreadProcessId.restype=wintypes.DWORD
    user32.GetWindowTextW.argtypes=[wintypes.HWND,wintypes.LPWSTR,ctypes.c_int]
    user32.GetClassNameW.argtypes=[wintypes.HWND,wintypes.LPWSTR,ctypes.c_int]
    user32.IsWindowVisible.argtypes=[wintypes.HWND]
    user32.ShowWindow.argtypes=[wintypes.HWND,ctypes.c_int]
    user32.SetForegroundWindow.argtypes=[wintypes.HWND]
    callback_type=ctypes.WINFUNCTYPE(wintypes.BOOL,wintypes.HWND,wintypes.LPARAM)
    found=[]
    @callback_type
    def visit(hwnd,_):
        title=ctypes.create_unicode_buffer(512);kind=ctypes.create_unicode_buffer(128)
        user32.GetWindowTextW(hwnd,title,len(title));user32.GetClassNameW(hwnd,kind,len(kind))
        if user32.IsWindowVisible(hwnd) and 'KABOAT 2026' in title.value and kind.value.startswith('Chrome_WidgetWin'):
            found.append(hwnd)
        return True
    user32.EnumWindows.argtypes=[callback_type,wintypes.LPARAM]
    user32.EnumWindows(visit,0)
    if not found:return False
    hwnd=found[0];foreground=user32.GetForegroundWindow()
    thread=user32.GetWindowThreadProcessId(foreground,None) if foreground else 0
    current=kernel32.GetCurrentThreadId();attached=False
    try:
        if thread and thread!=current:attached=bool(user32.AttachThreadInput(current,thread,True))
        user32.ShowWindow(hwnd,3)  # SW_MAXIMIZE, including a minimized existing window.
        user32.SetForegroundWindow(hwnd)
    finally:
        if attached:user32.AttachThreadInput(current,thread,False)
    # The window exists even if Windows restricts focus; never duplicate it.
    return True


class DashboardWindow:
    def __init__(self):self.lock=threading.Lock()
    def __call__(self,url):
        with self.lock:
            if focus_dashboard():return
            if os.name=='nt':
                paths=[Path(os.environ.get('PROGRAMFILES','C:/Program Files'))/'Google/Chrome/Application/chrome.exe',
                       Path(os.environ.get('PROGRAMFILES(X86)','C:/Program Files (x86)'))/'Microsoft/Edge/Application/msedge.exe']
                browser=next((p for p in paths if p.exists()),None)
                if browser:
                    profile=Path(os.environ['LOCALAPPDATA'])/'KabotSensorViewer/browser'
                    subprocess.Popen([str(browser),'--app='+url,'--user-data-dir='+str(profile),
                                      '--no-first-run','--no-default-browser-check','--start-maximized'],
                                     stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                    until=time.monotonic()+8
                    while time.monotonic()<until:
                        if focus_dashboard():break
                        time.sleep(.1)
                    return
            webbrowser.open(url)
