"""
ทดสอบเบื้องต้น: กล้องตรวจจับวัตถุสีแดง -> สั่งเซอร์โว id=10 (แกนขึ้น-ลง/tilt) ให้หมุนตามอัตโนมัติ
พร้อมเว็บดูภาพสดพร้อม overlay ตรวจจับ (เฟรมเดียวกับที่ใช้ตัดสินใจสั่งเซอร์โวจริง)
เป็นเวอร์ชันทดสอบ IBVS แบบง่าย เฉพาะแกน tilt แกนเดียว (แกน pan รอเซอร์โวตัวใหม่ที่เข้ากันได้)
ไม่แตะโค้ดเดิมของโรงงาน — ใช้ roslibpy ผ่าน rosbridge เหมือนสคริปต์อื่นๆ

⚠️ เซอร์โว id=10 sensitive สูงมาก (พบว่าขยับแค่ 100us ทำให้หมุนไปเกือบ 1/4 รอบ)
   จึงจำกัดช่วงตำแหน่งให้แคบมาก (1470-1530us) และจำกัดการขยับต่อครั้งไม่เกิน MAX_STEP_US

⚠️ บทเรียนจากการทดสอบจริง (2026-08-13): ตั้ง KP ต่ำเกินไปตอนแรก (0.04) ทำให้ step แต่ละรอบเล็กกว่า 1us
   บ่อยครั้ง พอปัดเป็นจำนวนเต็มแล้วค่าไม่เปลี่ยนหลายรอบติดกัน เซอร์โวเลยไม่ขยับให้เห็นเลยทั้งที่ตรวจจับ
   เป้าได้ปกติ — แก้โดยเพิ่ม MIN_STEP_US บังคับขยับขั้นต่ำที่มีนัยสำคัญทุกครั้งที่ error เกิน DEADBAND_PX

logic: ถ้าเป้าอยู่ต่ำกว่ากึ่งกลางภาพ (error_y เป็นบวก) จะเพิ่มค่า pulse (ก้มลง)
       ถ้าเป้าอยู่สูงกว่ากึ่งกลางภาพ (error_y เป็นลบ) จะลดค่า pulse (เงยขึ้น)
       (ทิศทางนี้ยืนยันจากการทดสอบจริงว่า pulse มากขึ้น = ก้มลง)

วิธีใช้:
    python3 tilt_track_test.py
    เปิดเบราว์เซอร์ไปที่ http://<IP ของ Pi>:8090/   (เช่น http://192.168.137.28:8090/)
    กด Ctrl+C ที่ terminal เพื่อหยุด (เซอร์โวจะกลับตำแหน่งกึ่งกลางก่อนปิด)
"""

import sys
import threading
import time

sys.path.append("/home/pi/Desktop/Poramet_Project/vision")
from red_detect_test import grab_frame, detect_red  # noqa: E402

import cv2  # noqa: E402
import roslibpy  # noqa: E402
from flask import Flask, Response, render_template_string  # noqa: E402

ROSBRIDGE_HOST = "127.0.0.1"
ROSBRIDGE_PORT = 9090

SERVO_ID = 10
CENTER_US = 1500
MIN_US = 1470
MAX_US = 1530
KP = 0.3  # us ต่อ pixel error
DEADBAND_PX = 6  # ถ้า error น้อยกว่านี้ ถือว่าตรงพอแล้ว ไม่ต้องขยับ (กันสั่นตลอดเวลา)
MIN_STEP_US = 3  # ขยับขั้นต่ำเวลามี error เกิน deadband — กันปัญหาขยับทีละนิดจนต่ำกว่าที่เซอร์โวตอบสนอง (พบปัญหานี้จริง 2026-08-13)
MAX_STEP_US = 10  # จำกัดการขยับสูงสุดต่อ 1 รอบ กันกระโดดแรงเกิน
LOOP_INTERVAL = 0.7  # วินาที ระหว่างแต่ละรอบ

WEB_PORT = 8090

app = Flask(__name__)
_lock = threading.Lock()
_latest_jpeg = None
_stats = {"servo_pos": CENTER_US, "found": False, "error_y": 0, "fps": 0.0}


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def run_flask():
    # ปิด log คำขอ HTTP ของ Flask ที่รกๆ ทาง terminal (ไม่กระทบการทำงาน)
    app.run(host="0.0.0.0", port=WEB_PORT, threaded=True, use_reloader=False)


def tracking_loop():
    """รันใน main thread เสมอ (เหมือนสคริปต์อื่นที่ใช้ roslibpy ได้จริง) — Flask แยกไปรันใน background thread แทน"""
    global _latest_jpeg

    client = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    client.run()
    time.sleep(0.3)  # เผื่อเวลาให้เชื่อมต่อเสร็จสมบูรณ์ก่อนเช็คสถานะ
    if not client.is_connected:
        print("เชื่อมต่อ rosbridge ไม่สำเร็จ")
        return

    pub = roslibpy.Topic(
        client, "/ros_robot_controller/pwm_servo/set_state", "ros_robot_controller/SetPWMServoState"
    )

    def send_position(pos, duration=0.4):
        msg = {"state": [{"id": [SERVO_ID], "position": [int(pos)], "offset": [0]}], "duration": duration}
        pub.publish(roslibpy.Message(msg))

    current_pos = CENTER_US
    print(f"เริ่ม tilt tracking (servo id={SERVO_ID}) ช่วง {MIN_US}-{MAX_US}us — Ctrl+C เพื่อหยุด")
    send_position(current_pos, duration=0.5)
    time.sleep(0.6)

    frame_count = 0
    t_start = time.time()

    try:
        i = 0
        while True:
            try:
                frame = grab_frame()
            except Exception as e:
                print(f"[{i}] ดึงภาพไม่สำเร็จ: {e}")
                time.sleep(LOOP_INTERVAL)
                continue

            annotated, _mask, detections = detect_red(frame)

            if detections:
                d = detections[0]
                cx, cy = d["centroid"]
                img_h, img_w = frame.shape[:2]
                center_y = img_h // 2
                error_y = cy - center_y

                if abs(error_y) <= DEADBAND_PX:
                    # เป้าอยู่ใกล้กึ่งกลางพอแล้ว ไม่ต้องขยับ (กันสั่นตลอดเวลา)
                    print(f"[{i}] เป้าอยู่ในกรอบกลางแล้ว (error_y={error_y:+d}) — ไม่ขยับ, servo={current_pos:.0f}us")
                else:
                    raw_step = KP * error_y
                    # บังคับขยับขั้นต่ำ MIN_STEP_US เสมอเมื่อเกิน deadband กันปัญหาขยับน้อยกว่าที่เซอร์โวตอบสนอง
                    step = max(abs(raw_step), MIN_STEP_US)
                    step = step if raw_step >= 0 else -step
                    step = clamp(step, -MAX_STEP_US, MAX_STEP_US)
                    current_pos = clamp(current_pos + step, MIN_US, MAX_US)

                    send_position(current_pos, duration=LOOP_INTERVAL * 0.8)
                    print(f"[{i}] พบเป้า centroid_y={cy} error_y={error_y:+d} step={step:+.1f} -> servo={current_pos:.0f}us")

                with _lock:
                    _stats["found"] = True
                    _stats["error_y"] = error_y
            else:
                print(f"[{i}] ไม่พบเป้า — เซอร์โวค้างตำแหน่งเดิม {current_pos:.0f}us")
                with _lock:
                    _stats["found"] = False
                    _stats["error_y"] = 0

            cv2.putText(
                annotated,
                f"servo={current_pos:.0f}us",
                (10, annotated.shape[0] - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2,
            )
            ok, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with _lock:
                    _latest_jpeg = buf.tobytes()
                    _stats["servo_pos"] = current_pos

            frame_count += 1
            elapsed = time.time() - t_start
            if elapsed >= 1.0:
                with _lock:
                    _stats["fps"] = round(frame_count / elapsed, 1)
                frame_count = 0
                t_start = time.time()

            i += 1
            time.sleep(LOOP_INTERVAL)
    except KeyboardInterrupt:
        print("\nได้รับ Ctrl+C กำลังสั่งเซอร์โวกลับตำแหน่งกึ่งกลาง รอสักครู่...")
    finally:
        send_position(CENTER_US, duration=0.8)
        time.sleep(1.0)
        pub.unadvertise()
        client.terminate()
        print("เซอร์โวกลับตำแหน่งกึ่งกลางแล้ว ปิดการเชื่อมต่อเรียบร้อย")


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
<title>PuppyPi Tilt Tracking</title>
<style>
 body { background:#111; color:#eee; font-family: sans-serif; text-align:center; margin:0; padding:1rem; }
 img { max-width: 95vw; border: 2px solid #333; border-radius: 8px; margin-top: 0.5rem; }
 h2 { margin: 0.5rem 0; }
 #stats { color:#8f8; font-size: 0.9rem; margin-top: 0.5rem; }
</style>
</head>
<body>
<h2>PuppyPi — Tilt Tracking (id=10) live</h2>
<div id="stats">กำลังโหลด...</div>
<img src="/video_feed">
<script>
async function poll() {
  try {
    const r = await fetch('/stats');
    const j = await r.json();
    document.getElementById('stats').textContent =
      `FPS: ${j.fps} | เจอเป้า: ${j.found ? 'ใช่' : 'ไม่เจอ'} | error_y: ${j.error_y} | servo: ${j.servo_pos.toFixed(0)}us`;
  } catch (e) {}
  setTimeout(poll, 500);
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
    resp = Response(mjpeg_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")
    resp.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    resp.headers["Pragma"] = "no-cache"
    return resp


@app.route("/stats")
def stats():
    with _lock:
        return dict(_stats)


if __name__ == "__main__":
    t = threading.Thread(target=run_flask, daemon=True)
    t.start()
    print(f"เปิดเบราว์เซอร์ไปที่ http://<IP ของ Pi>:{WEB_PORT}/  เช่น http://192.168.137.28:{WEB_PORT}/")
    tracking_loop()
