#!/usr/bin/env python3
"""Minimal LCD panel for the XGO Lite: browse/connect WiFi, start/stop robot-edge.

Runs standalone (no dependency on the vendor RaspberryPi-CM4-main main.py menu).
Buttons: C/D move the selection, A is the primary action, B switches between
the WIFI and ROBOT-EDGE views.
"""
import os
import signal
import subprocess
import time

import RPi.GPIO as GPIO
import xgoscreen.LCD_2inch as LCD_2inch
from PIL import Image, ImageDraw, ImageFont

WPA_CLI = ["sudo", "/usr/sbin/wpa_cli", "-i", "wlan0"]
ROBOT_EDGE_DIR = "/home/pi/RoboProtocol"
ROBOT_EDGE_BIN = f"{ROBOT_EDGE_DIR}/target/release/robot-edge"
ROBOT_EDGE_ID = "xgo_real"
ROBOT_EDGE_EXTRA_ARGS = ["--camera"]
ROBOT_EDGE_ARGS = ["--robot-id", ROBOT_EDGE_ID] + ROBOT_EDGE_EXTRA_ARGS
ROBOT_EDGE_MATCH = "target/release/robot-edge"
ROBOT_EDGE_PORT = 4433
# Optional BabyROS-integrated build (Zenoh sidecar, see BABYROS.md). If this
# file exists, button C in the ROBOT-EDGE view switches between it and
# ROBOT_EDGE_BIN. Its filename still contains ROBOT_EDGE_MATCH, so
# robot_edge_pid() finds either build while it's running.
ROBOT_EDGE_BIN_BABYROS = f"{ROBOT_EDGE_DIR}/target/release/robot-edge.babyros"
ZENOH_DEFAULT_PORT = 7447  # robot-edge's --zenoh-port default
ZENOH_PORT = 7447

FONT = ImageFont.truetype("/home/pi/model/msyh.ttc", 16)
FONT_SM = ImageFont.truetype("/home/pi/model/msyh.ttc", 13)
COLOR_BG = (15, 21, 46)
COLOR_SEL = (24, 47, 223)
COLOR_TXT = (255, 255, 255)
COLOR_DIM = (120, 130, 170)
COLOR_YELLOW = (255, 214, 0)
COLOR_OK = (60, 200, 90)
COLOR_BAD = (220, 60, 60)


class Button:
    # Empirically measured on this unit's case silkscreen (A top-left, B
    # top-right, C bottom-left, D bottom-right) -- does NOT match the
    # vendor key.py's a/b/c/d -> 24/23/17/22 assumption.
    PINS = {"a": 17, "b": 22, "c": 23, "d": 24}

    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for pin in self.PINS.values():
            GPIO.setup(pin, GPIO.IN, GPIO.PUD_UP)

    def pressed(self, key):
        pin = self.PINS[key]
        if GPIO.input(pin):
            return False
        while not GPIO.input(pin):
            time.sleep(0.02)
        return True


def run(cmd, timeout=10):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.stdout
    except Exception:
        return ""


def wifi_status():
    out = run(WPA_CLI + ["status"])
    ssid, ip_addr, state = None, None, None
    for line in out.splitlines():
        if line.startswith("ssid="):
            ssid = line.split("=", 1)[1]
        elif line.startswith("ip_address="):
            ip_addr = line.split("=", 1)[1]
        elif line.startswith("wpa_state="):
            state = line.split("=", 1)[1]
    return ssid, ip_addr, state


def saved_networks():
    out = run(WPA_CLI + ["list_networks"])
    nets = {}
    for line in out.splitlines()[1:]:
        parts = line.split("\t")
        if len(parts) >= 2:
            nets[parts[1]] = parts[0]
    return nets


def scan_networks():
    run(WPA_CLI + ["scan"])
    time.sleep(2)
    out = run(WPA_CLI + ["scan_results"])
    seen, results = set(), []
    for line in out.splitlines()[1:]:
        parts = line.split("\t")
        if len(parts) < 5:
            continue
        signal_dbm, ssid = parts[2], parts[4].strip()
        if not ssid or ssid in seen:
            continue
        seen.add(ssid)
        results.append((ssid, int(signal_dbm)))
    results.sort(key=lambda x: x[1], reverse=True)
    return results


def connect(ssid, saved):
    net_id = saved.get(ssid)
    if net_id is None:
        return False
    run(WPA_CLI + ["select_network", net_id])
    run(WPA_CLI + ["save_config"])
    return True


def robot_edge_pid():
    out = run(["pgrep", "-f", ROBOT_EDGE_MATCH])
    pids = [int(p) for p in out.split()]
    return pids[0] if pids else None


def babyros_available():
    return os.access(ROBOT_EDGE_BIN_BABYROS, os.X_OK)


def robot_edge_variant(pid):
    """"babyros" or "plain" for a running robot-edge, judged from its command line."""
    out = run(["ps", "-o", "args=", "-p", str(pid)])
    return "babyros" if "robot-edge.babyros" in out else "plain"


def zenoh_listening(port=ZENOH_PORT):
    return f":{port} " in run(["ss", "-ltn"])


def robot_edge_start(variant="plain"):
    if robot_edge_pid():
        return
    binary, args = ROBOT_EDGE_BIN, list(ROBOT_EDGE_ARGS)
    if variant == "babyros" and babyros_available():
        binary = ROBOT_EDGE_BIN_BABYROS
        if ZENOH_PORT != ZENOH_DEFAULT_PORT:
            args += ["--zenoh-port", str(ZENOH_PORT)]
    ts = time.strftime("%Y%m%d-%H%M%S")
    log_path = f"{ROBOT_EDGE_DIR}/robot-edge-{ts}-launch.log"
    with open(log_path, "wb") as log:
        subprocess.Popen(
            [binary] + args,
            cwd=ROBOT_EDGE_DIR,
            stdout=log,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )


def robot_edge_stop():
    pid = robot_edge_pid()
    if not pid:
        return
    os.kill(pid, signal.SIGINT)
    for _ in range(30):
        time.sleep(0.1)
        if robot_edge_pid() is None:
            return
    pid = robot_edge_pid()
    if pid:
        os.kill(pid, signal.SIGKILL)


def robot_edge_uptime(pid):
    out = run(["ps", "-o", "etimes=", "-p", str(pid)]).strip()
    if not out.isdigit():
        return "?"
    secs = int(out)
    h, rem = divmod(secs, 3600)
    m, s = divmod(rem, 60)
    return f"{h:02d}:{m:02d}:{s:02d}"


COMPANY_NAME = "SpinWorks ltd"


def draw_header(draw, ssid, ip_addr, state, show_company=False):
    ok = state == "COMPLETED"
    draw.rectangle((0, 0, 320, 34), fill=(10, 14, 32))
    label = f"{ssid or '(disconnected)'}"
    draw.text((8, 3), label, fill=COLOR_TXT if ok else COLOR_DIM, font=FONT)
    draw.text((8, 19), ip_addr or "no ip", fill=COLOR_OK if ok else COLOR_BAD, font=FONT_SM)
    if show_company:
        w = draw.textlength(COMPANY_NAME, font=FONT_SM)
        draw.text((320 - 8 - w, 11), COMPANY_NAME, fill=COLOR_YELLOW, font=FONT_SM)


WIFI_ROWS = 5
ROW_H = 28
LIST_TOP = 38
FOOTER_Y = 206


def draw_wifi_view(draw, networks, saved, selection):
    y = LIST_TOP
    if not networks:
        draw.text((8, y), "scanning...", fill=COLOR_DIM, font=FONT)
    for i, (ssid, sig) in enumerate(networks[:WIFI_ROWS]):
        bg = COLOR_SEL if i == selection else COLOR_BG
        draw.rectangle((0, y, 320, y + ROW_H), fill=bg)
        mark = "*" if ssid in saved else " "
        draw.text((8, y + 5), f"{mark} {ssid}", fill=COLOR_TXT, font=FONT)
        draw.text((260, y + 7), f"{sig}", fill=COLOR_DIM, font=FONT_SM)
        y += ROW_H
    draw.text((8, FOOTER_Y), "C/D move  A connect(*)  B->edge", fill=COLOR_DIM, font=FONT_SM)


def draw_edge_view(draw, pid, variant):
    running = pid is not None
    if running:
        variant = robot_edge_variant(pid)
    draw.rectangle((0, 40, 320, 72), fill=COLOR_SEL)
    status = "RUNNING" if running else "STOPPED"
    color = COLOR_OK if running else COLOR_BAD
    draw.text((16, 47), f"robot-edge: {status}", fill=color, font=FONT)

    y = 80
    zenoh_up = running and zenoh_listening()
    for line, color in (
        (f"id: {ROBOT_EDGE_ID}", COLOR_TXT),
        (f"port: {ROBOT_EDGE_PORT} (quic/udp)", COLOR_TXT),
        (f"build: {variant}", COLOR_TXT),
        (f"zenoh: :{ZENOH_PORT} " + ("UP" if zenoh_up else "off"), COLOR_OK if zenoh_up else COLOR_DIM),
        (f"args: {' '.join(ROBOT_EDGE_EXTRA_ARGS)}", COLOR_TXT),
    ):
        draw.text((16, y), line, fill=color, font=FONT_SM)
        y += 17

    if running:
        draw.text((16, y), f"pid: {pid}   up: {robot_edge_uptime(pid)}", fill=COLOR_DIM, font=FONT_SM)
    else:
        draw.text((16, y), "not running", fill=COLOR_DIM, font=FONT_SM)

    action = "A: stop" if running else "A: start"
    switch = "  C: build" if (not running and babyros_available()) else ""
    draw.text((16, FOOTER_Y), f"{action}  B: wifi{switch}", fill=COLOR_DIM, font=FONT_SM)


def main():
    display = LCD_2inch.LCD_2inch()
    display.Init()
    display.clear()
    button = Button()

    view = "wifi"
    variant = "plain"
    selection = 0
    networks = scan_networks()
    saved = saved_networks()
    last_scan = time.time()
    last_status_poll = 0
    ssid = ip_addr = state = None
    dirty = True

    while True:
        now = time.time()
        if now - last_status_poll > 2:
            ssid, ip_addr, state = wifi_status()
            last_status_poll = now
            dirty = True
        if view == "wifi" and now - last_scan > 15:
            networks = scan_networks()
            saved = saved_networks()
            last_scan = now
            dirty = True

        if button.pressed("b"):
            view = "edge" if view == "wifi" else "wifi"
            selection = 0
            dirty = True
        elif button.pressed("c"):
            if view == "wifi" and networks:
                selection = (selection - 1) % min(len(networks), WIFI_ROWS)
                dirty = True
            elif view == "edge" and not robot_edge_pid() and babyros_available():
                variant = "babyros" if variant == "plain" else "plain"
                dirty = True
        elif button.pressed("d"):
            if view == "wifi" and networks:
                selection = (selection + 1) % min(len(networks), WIFI_ROWS)
                dirty = True
        elif button.pressed("a"):
            if view == "wifi" and networks:
                connect(networks[selection][0], saved)
                last_status_poll = 0
            else:
                if robot_edge_pid():
                    robot_edge_stop()
                else:
                    robot_edge_start(variant)
            dirty = True

        if dirty:
            splash = Image.new("RGB", (320, 240), COLOR_BG)
            draw = ImageDraw.Draw(splash)
            draw_header(draw, ssid, ip_addr, state, show_company=(view == "wifi"))
            if view == "wifi":
                draw_wifi_view(draw, networks, saved, selection)
            else:
                draw_edge_view(draw, robot_edge_pid(), variant)
            display.ShowImage(splash)
            dirty = False
        else:
            time.sleep(0.05)


if __name__ == "__main__":
    main()
