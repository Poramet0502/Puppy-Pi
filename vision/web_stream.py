"""
เว็บสตรีมภาพกล้องสดของ PuppyPi พร้อมผลตรวจจับสีแดงซ้อนทับแบบเรียลไทม์
รันบน host เท่านั้น ไม่แตะโค้ดเดิมของโรงงาน — ดึงภาพจาก web_video_server (port 8080) ของหุ่นยนต์
ใช้ฟังก์ชัน grab_frame() / detect_red() เดิมจาก red_detect_test.py

วิธีใช้:
    python3 web_stream.py
    เปิดเบราว์เซอร์ไปที่ http://<IP ของ Pi>:8090/   (เช่น http://192.168.137.28:8090/)
    หรือ http://localhost:8090/ ถ้าเปิดบนเครื่องเดียวกัน
"""

import threading
import time

import cv2
from flask import Flask, Response, render_template_string

from red_detect_test import grab_frame, detect_red

app = Flask(__name__)

_lock = threading.Lock()
_latest_jpeg = None
_stats = {"detections": 0, "fps": 0.0}


def capture_loop():
    global _latest_jpeg
    frame_count = 0
    t_start = time.time()
    while True:
        try:
            frame = grab_frame()
            annotated, _mask, detections = detect_red(frame)
            ok, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with _lock:
                    _latest_jpeg = buf.tobytes()
                    _stats["detections"] = len(detections)

            frame_count += 1
            elapsed = time.time() - t_start
            if elapsed >= 1.0:
                with _lock:
                    _stats["fps"] = round(frame_count / elapsed, 1)
                frame_count = 0
                t_start = time.time()
        except Exception as e:
            print("capture_loop error:", e)
            time.sleep(1.0)


def mjpeg_generator():
    while True:
        with _lock:
            jpeg = _latest_jpeg
        if jpeg is not None:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
        time.sleep(0.03)


PAGE = """
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>PuppyPi Camera - Red Detect</title>
<style>
 body { background:#111; color:#eee; font-family: sans-serif; text-align:center; margin:0; padding:1rem; }
 img { max-width: 95vw; border: 2px solid #333; border-radius: 8px; margin-top: 0.5rem; }
 h2 { margin: 0.5rem 0; }
 #stats { color:#8f8; font-size: 0.9rem; margin-top: 0.5rem; }
</style>
</head>
<body>
<h2>PuppyPi Camera — Red Detection (live)</h2>
<div id="stats">กำลังโหลด...</div>
<img src="/video_feed">
<script>
async function poll() {
  try {
    const r = await fetch('/stats');
    const j = await r.json();
    document.getElementById('stats').textContent =
      `FPS: ${j.fps} | พบวัตถุสีแดง: ${j.detections} ก้อน`;
  } catch (e) {}
  setTimeout(poll, 1000);
}
poll();
</script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(PAGE)


@app.route("/video_feed")
def video_feed():
    return Response(mjpeg_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/stats")
def stats():
    with _lock:
        return dict(_stats)


if __name__ == "__main__":
    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()
    print("เปิดเบราว์เซอร์ไปที่ http://<IP ของ Pi>:8090/  เช่น http://192.168.137.28:8090/")
    app.run(host="0.0.0.0", port=8090, threaded=True)
