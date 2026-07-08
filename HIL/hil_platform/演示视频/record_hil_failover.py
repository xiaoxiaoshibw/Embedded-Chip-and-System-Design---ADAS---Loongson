#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""录制 HIL 控制台"主备失控接管"演示视频（mock 模式，虚拟双 Nano）。

用 Playwright 打开 /live，真实点击「选场景 takeover → 加载场景 → 开始」，
捕获 12s 时主控 Nano A seq 卡死 → 备机 Nano B 接管 的全过程，录成 webm。
API 兜底：若某个 UI 动作没生效，则用后端 API 强制推进，保证失控一定被录到。
"""
import json
import os
import re
import time
import urllib.request

from playwright.sync_api import sync_playwright

B = "http://127.0.0.1:8000"
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hil_video")
os.makedirs(OUT_DIR, exist_ok=True)


def api_post(path, body=None):
    data = json.dumps(body or {}).encode()
    req = urllib.request.Request(B + path, data=data, headers={"Content-Type": "application/json"})
    try:
        return json.loads(urllib.request.urlopen(req, timeout=5).read())
    except Exception as e:
        return {"error": str(e)}


def api_status():
    try:
        return json.loads(urllib.request.urlopen(B + "/api/status", timeout=5).read())
    except Exception as e:
        return {"error": str(e)}


def main():
    # 干净起点
    api_post("/api/simulation/reset")
    time.sleep(0.5)

    with sync_playwright() as pw:
        browser = pw.chromium.launch(headless=True)
        # 整页实测 1600x1233（运行中含底部曲线图），加高视口一条视频装下全部
        ctx = browser.new_context(
            viewport={"width": 1600, "height": 1240},
            record_video_dir=OUT_DIR,
            record_video_size={"width": 1600, "height": 1240},
        )
        page = ctx.new_page()
        page.goto(B + "/live", wait_until="networkidle")
        page.wait_for_timeout(2500)  # 展示初始控制台

        # 1) 选 takeover 场景
        try:
            sel = page.locator('select:has(option[value="takeover"])').first
            sel.select_option("takeover")
            page.wait_for_timeout(1200)
        except Exception as e:
            print("select scenario UI failed:", e)

        # 2) 点「加载场景」
        try:
            page.get_by_role("button", name=re.compile("加载场景")).click()
        except Exception as e:
            print("load button UI failed:", e)
        page.wait_for_timeout(1500)
        if api_status().get("state") not in ("READY", "PAUSED"):
            print("API fallback: load takeover")
            api_post("/api/scenario/load", {"scenario": "takeover"})
            page.wait_for_timeout(1000)

        # 3) 点「开始」
        try:
            page.get_by_role("button", name=re.compile("开始")).click()
        except Exception as e:
            print("start button UI failed:", e)
        page.wait_for_timeout(1200)
        if api_status().get("state") != "RUNNING":
            print("API fallback: start")
            api_post("/api/simulation/start")

        # 4) 录制到失控发生后（12s 注入 + 数秒观察）
        t0 = time.time()
        took_over_at = None
        while time.time() - t0 < 18:
            st = api_status()
            if st.get("takeover") and took_over_at is None:
                took_over_at = st.get("scenario_time")
                print("TAKEOVER at sim_time=%.1f ctrl=%s" % (took_over_at, st.get("active_controller")))
            page.wait_for_timeout(500)
        page.wait_for_timeout(2000)  # 尾帧停留

        api_post("/api/simulation/stop")
        page.wait_for_timeout(800)

        video_path = page.video.path()
        ctx.close()   # 关闭 context 才会 flush 视频文件
        browser.close()
        print("VIDEO_WEBM:", video_path)
        print("TOOK_OVER_AT:", took_over_at)


if __name__ == "__main__":
    main()
