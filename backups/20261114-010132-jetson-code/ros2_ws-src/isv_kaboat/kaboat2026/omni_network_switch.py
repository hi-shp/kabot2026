#!/usr/bin/env python3
"""Jetson의 Rocket용 USB 랜을 켜거나 끄는 독립 실행 도구.

    python3 omni_network_switch.py          # on/off/status/exit 대화형 입력
    python3 omni_network_switch.py on
    python3 omni_network_switch.py off
    python3 omni_network_switch.py status
    python3 omni_network_switch.py off --dry-run

on: Rocket USB 랜을 연결하고 인터넷 기본 경로로 사용합니다.
    DHCP 서버가 필요합니다. Windows 인터넷 공유를 사용한다면 공유를 먼저 켜세요.
off: Rocket USB 랜을 해제하고 준비된 Wi-Fi 경로로 돌아갑니다.
자동 연결 설정도 함께 저장하므로 off 후에는 on으로 다시 켜야 합니다.
Rocket 전원이나 안테나 간 무선 설정은 이 도구의 제어 대상이 아닙니다.
관리자 암호는 sudo가 터미널에서 요청하며 파일에 저장하지 않습니다.
"""

import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time


DEVICE = "enx80691ae85b26"
DEVICE_MAC = "80:69:1a:e8:5b:26"
PROFILE_UUID = "5864ceac-2123-4547-81cc-737579e4564a"
DHCP_TIMEOUT = 45
ACTIVATION_TIMEOUT = DHCP_TIMEOUT + 15
ON_SETTINGS = {
    "connection.autoconnect": "yes",
    "ipv4.method": "auto",
    "ipv4.addresses": "",
    "ipv4.gateway": "",
    "ipv4.dns": "",
    "ipv4.never-default": "no",
    "ipv4.route-metric": "100",
    "ipv4.ignore-auto-dns": "no",
    "ipv4.dns-priority": "50",
    "ipv4.dhcp-timeout": str(DHCP_TIMEOUT),
    "ipv4.may-fail": "no",
}


class SwitchError(Exception):
    pass


class OmniSwitch:
    def __init__(self, dry_run=False):
        self.dry_run = dry_run
        self.env = dict(os.environ, LC_ALL="C")
        self.ssh_peer = os.environ.get("SSH_CONNECTION", "").split()[:1]

    def run(self, *args, privileged=False, timeout=45):
        command = list(args)
        if privileged and os.geteuid() != 0:
            command = ["sudo", "-n"] + command
        if privileged and self.dry_run:
            print("[미리보기] " + shlex.join(command))
            return ""
        try:
            result = subprocess.run(
                command, capture_output=True, text=True, env=self.env, timeout=timeout
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise SwitchError(str(exc)) from exc
        if result.returncode:
            raise SwitchError(result.stderr.strip() or result.stdout.strip())
        return result.stdout.strip()

    def profile_value(self, key):
        return self.run(
            "nmcli", "--escape", "no", "--get-values", key,
            "connection", "show", "uuid", PROFILE_UUID
        )

    def active(self):
        value = self.run(
            "nmcli", "--get-values", "GENERAL.CON-UUID", "device", "show", DEVICE
        )
        if value not in ("", "--", PROFILE_UUID):
            raise SwitchError("USB 랜에 다른 연결 프로필이 사용 중이므로 변경하지 않습니다.")
        return value == PROFILE_UUID

    def validate_target(self):
        if not all(shutil.which(tool) for tool in ("nmcli", "ip")):
            raise SwitchError("이 도구는 nmcli와 ip가 설치된 Jetson/Linux에서 실행하세요.")
        try:
            mac = Path("/sys/class/net", DEVICE, "address").read_text().strip().lower()
        except OSError as exc:
            raise SwitchError("Rocket용 USB 랜을 찾을 수 없습니다. 연결을 확인하세요.") from exc
        if mac != DEVICE_MAC:
            raise SwitchError("USB 랜의 MAC 주소가 등록된 장치와 달라 변경하지 않습니다.")
        if self.profile_value("connection.interface-name") != DEVICE:
            raise SwitchError("연결 프로필의 장치가 달라 변경하지 않습니다.")
        self.active()

    def routes(self):
        return json.loads(self.run("ip", "-j", "-4", "route", "show", "default"))

    def internet_device(self):
        routes = json.loads(self.run("ip", "-j", "-4", "route", "get", "1.1.1.1"))
        return routes[0].get("dev", "") if routes else ""

    def wifi_fallback(self):
        devices = self.run("nmcli", "-t", "-f", "DEVICE,TYPE,STATE", "device", "status")
        wifi = set()
        for row in devices.splitlines():
            parts = row.split(":", 2)
            if len(parts) == 3 and parts[1] == "wifi" and parts[2].startswith("connected"):
                wifi.add(parts[0])
        alternatives = [
            route for route in self.routes()
            if route.get("dev") != DEVICE and "linkdown" not in route.get("flags", [])
        ]
        alternatives.sort(key=lambda route: route.get("metric", 0))
        if not alternatives or alternatives[0].get("dev") not in wifi:
            raise SwitchError("복귀할 Wi-Fi 기본 경로가 준비되지 않아 off를 중단합니다.")
        return alternatives[0]["dev"]

    def protect_ssh(self):
        if self.ssh_peer:
            route = json.loads(self.run("ip", "-j", "route", "get", self.ssh_peer[0]))
            if route and route[0].get("dev") == DEVICE:
                raise SwitchError(
                    "현재 SSH가 Rocket USB 랜을 사용 중입니다. "
                    "Wi-Fi 주소로 SSH 접속한 뒤 전환하세요."
                )

    def authorize(self):
        if not self.dry_run and os.geteuid() != 0:
            print("Jetson 관리자 인증이 필요합니다. 암호는 sudo 입력창에만 입력하세요.")
            result = subprocess.run(["sudo", "-v"], env=self.env)
            if result.returncode:
                raise SwitchError("관리자 인증에 실패했습니다. 설정을 변경하지 않았습니다.")

    def modify(self, values):
        args = ["nmcli", "connection", "modify", "uuid", PROFILE_UUID]
        for key, value in values.items():
            args.extend((key, value))
        self.run(*args, privileged=True)

    def wait_route(self, expected, seconds=15):
        end = time.monotonic() + seconds
        while True:
            if self.internet_device() == expected:
                return
            if time.monotonic() >= end:
                raise SwitchError("예상한 인터넷 경로로 전환되지 않았습니다.")
            time.sleep(0.5)

    def activate(self):
        self.run(
            "nmcli", "--wait", str(ACTIVATION_TIMEOUT), "connection", "up",
            "uuid", PROFILE_UUID, "ifname", DEVICE, privileged=True,
            timeout=ACTIVATION_TIMEOUT + 15
        )

    def diagnostics(self):
        print(self.run(
            "nmcli", "-f",
            "GENERAL.STATE,GENERAL.REASON,WIRED-PROPERTIES.CARRIER,IP4.ADDRESS,IP4.GATEWAY",
            "device", "show", DEVICE
        ))
        addresses = self.run("nmcli", "--get-values", "IP4.ADDRESS", "device", "show", DEVICE)
        if addresses in ("", "--"):
            print("USB 랜에 IPv4 주소가 없습니다. on은 DHCP로 주소와 기본 경로를 받습니다.")
            print("DHCP 서버 또는 Windows 인터넷 연결 공유(ICS), Rocket 무선 연결을 확인하세요.")
            print("고정 IP 관리 페이지에 접속하는 설정과 인터넷 기본 경로 설정은 별개입니다.")

    def restore(self, values, was_active):
        try:
            self.modify(values)
            if was_active:
                self.activate()
            elif self.active():
                self.run("nmcli", "--wait", "10", "device", "disconnect", DEVICE, privileged=True)
        except SwitchError as exc:
            print("원복 중 오류: " + str(exc), file=sys.stderr)

    def switch(self, action):
        self.validate_target()
        was_active = self.active()
        if action == "off":
            fallback = self.wifi_fallback()
            self.protect_ssh()
            previous = {"connection.autoconnect": self.profile_value("connection.autoconnect")}
            settings = {"connection.autoconnect": "no"}
        else:
            previous = {key: self.profile_value(key) for key in ON_SETTINGS}
            if was_active and previous == ON_SETTINGS and self.internet_device() == DEVICE:
                print("이미 ON 상태입니다.")
                return
            self.protect_ssh()
            settings = ON_SETTINGS
        self.authorize()
        try:
            self.modify(settings)
            if action == "on":
                if was_active:
                    self.run("nmcli", "device", "reapply", DEVICE, privileged=True)
                else:
                    self.activate()
            elif was_active:
                self.run("nmcli", "--wait", "10", "device", "disconnect", DEVICE, privileged=True)
            if not self.dry_run:
                self.wait_route(DEVICE if action == "on" else fallback)
        except (SwitchError, KeyboardInterrupt):
            if not self.dry_run:
                if action == "on":
                    try:
                        self.diagnostics()
                    except (SwitchError, KeyboardInterrupt):
                        print("추가 진단을 완료하지 못했습니다.", file=sys.stderr)
                print("전환이 완료되지 않아 이전 USB 랜 설정으로 되돌립니다.")
                self.restore(previous, was_active)
            raise
        if self.dry_run:
            print("미리보기만 수행했습니다. 설정은 변경하지 않았습니다.")
        else:
            print("ON: Omni/Rocket 경유 인터넷을 사용합니다." if action == "on"
                  else "OFF: Rocket USB 랜을 해제하고 Wi-Fi 경로로 돌아왔습니다.")

    def status(self):
        self.validate_target()
        active = self.active()
        default = self.internet_device()
        label = "ON" if active and default == DEVICE else "OFF" if not active else "연결됨(다른 기본 경로)"
        print("Omni 인터넷: " + label)
        print("기본 인터넷 장치: " + (default or "없음"))
        print("USB 랜 자동 연결: " + self.profile_value("connection.autoconnect"))
        self.diagnostics()


def main():
    parser = argparse.ArgumentParser(description="Omni/Rocket USB 랜 인터넷 on/off 전환")
    parser.add_argument("action", nargs="?", choices=("on", "off", "status"))
    parser.add_argument("--dry-run", action="store_true", help="변경 명령만 표시")
    args = parser.parse_args()
    if args.dry_run and args.action is None:
        parser.error("--dry-run에는 on 또는 off를 함께 지정하세요.")
    controller = OmniSwitch(args.dry_run)
    try:
        if args.action:
            controller.status() if args.action == "status" else controller.switch(args.action)
            return 0
        controller.status()
        print("명령: on / off / status / exit")
        while True:
            try:
                action = input("omni> ").strip().lower()
                if action in ("exit", "quit", "q"):
                    return 0
                if action == "status":
                    controller.status()
                elif action in ("on", "off"):
                    controller.switch(action)
                elif action:
                    print("on, off, status, exit 중 하나를 입력하세요.")
            except SwitchError as exc:
                print("오류: " + str(exc), file=sys.stderr)
    except (EOFError, KeyboardInterrupt):
        print("\n종료합니다. 현재 연결 상태는 유지됩니다.")
        return 0
    except SwitchError as exc:
        print("오류: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
