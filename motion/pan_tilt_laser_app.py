#!/usr/bin/env python3
"""
Web App ควบคุม Pan-Tilt Servo (id 10, 11) + Laser (GPIO 7)
รวมทั้ง 2 เซอร์โว + เลเซอร์ ผ่าน Web Interface
URL: http://<IP>:8091/

คุณสมบัติ:
- Slider ปรับ Pan (id=10) และ Tilt (id=11)
- แสดงตำแหน่ง (pulse width) และองศา real-time
- ปุ่มควบคุมเลเซอร์ ON/OFF
- Dashboard ที่เอาไว้ใช้ง่าย
"""

import sys
import os
import json
import time
import math
import threading
import cv2
from flask import Flask, render_template_string, request, jsonify, Response
import roslibpy

sys.path.append('/home/pi/Desktop/Poramet_Project/vision')
from red_detect_test import grab_frame, detect_red  # noqa: E402

# ========== Config ==========
ROS_HOST = 'localhost'
ROS_PORT = 9090
FLASK_PORT = 8091
FLASK_HOST = '0.0.0.0'

# Servo config
PAN_ID = 10      # แกน Pan (ซ้าย-ขวา)
TILT_ID = 11     # แกน Tilt (บน-ล่าง)
MIN_POS = 500    # Pulse width minimum (us) — ใช้กับ Pan
MAX_POS = 2500   # Pulse width maximum (us) — ใช้กับ Pan

TILT_MIN_POS = 500   # ขอบเขตปลอดภัยของ Tilt (id=11) — กลับมาใช้ 500 ตามเดิม (ทดสอบ 0-1300 แล้วมีปัญหา ผู้ใช้สั่งกลับ 2026-09-15)
TILT_MAX_POS = 1300  # ตั้งจำกัดไว้ไม่ให้เกิน 1300us ตามคำขอผู้ใช้ (2026-09-08)

# --- Auto-Track config ---
# ⚠️ 2026-09-11: เปลี่ยนวิธีคำนวณจาก incremental feedback (ค่อยๆขยับตาม error ทีละนิด) เป็นคำนวณมุมเล็งสัมบูรณ์จากภาพเดียวทุกรอบ
# เหตุผล: กล้องติดอยู่กับตัวหุ่นยนต์แบบนิ่ง ไม่ได้หมุนตามป้อม Pan-Tilt — ทดสอบจริงพบว่า error ในภาพไม่ขยับเลยแม้ป้อมหมุนจนสุดทาง
# (เพราะการขยับป้อมไม่ทำให้ภาพที่กล้องเห็นเปลี่ยน) ดังนั้น incremental feedback loop ใช้ไม่ได้กับโครงสร้างนี้เลย ต้องคำนวณมุมตรงๆ แทน
AUTOTRACK_INTERVAL = 0.7  # วินาที ต่อรอบ loop
POSE_SETTLE_SEC = 2.0     # เวลารอให้ท่านิ่งหลังเปลี่ยนท่า ก่อนตรวจจับใหม่

US_PER_DEGREE = 1000 / 90  # อัตราส่วนพัลส์ต่อองศา (~11.11 us/deg) เท่ากันทั้ง Pan และ Tilt ยืนยันจาก calibration ทั้ง 2 แกน

CAM_LASER_HEIGHT_OFFSET_CM = 10.5  # เลเซอร์บนป้อมอยู่สูงกว่ากล้องบนตัวหุ่นยนต์ 10.5cm (วัดใหม่อีกครั้ง แก้จาก 11.5cm — ผู้ใช้วัดยืนยันใหม่ 2026-10-20 ช่วงทดลองเก็บผลชี้เป้า)

# ค่าชดเชย bias คงที่ (mechanical/alignment error ของป้อม+เลเซอร์จริงที่สูตรเรขาคณิตอย่างเดียวไม่ครอบคลุม)
# Calibrate จากการทดสอบจริง 2026-09-15 ที่ระยะ ~124cm: ระบบคำนวณได้ pan=1635us,tilt=566us แต่ค่าที่ยิงโดนเป้าจริงคือ pan=1510us,tilt=520us
# รอบแรก (124cm): 1510-1635=-125, 520-566=-46 — ทดสอบซ้ำแล้วยังเหลือคลาดเคลื่อน (1447/516 ควรเป็น 1420/500 ที่ ~130cm)
# ปรับสะสมรอบ 2 (2026-09-15): PAN -125-27=-152, TILT -46-16=-62
# รอบ 3 (2026-09-16): หลังแก้ FOCAL_LENGTH_PX (825->511) สูตรมุมเปลี่ยน offset เดิมไม่พอดีอีกครั้ง — เก็บข้อมูล Calibration Tool 9 จุด (100/150/200cm)
# ค่าเฉลี่ยผลต่าง (คำนวณได้ - ค่าจริง) จากทั้ง 9 จุด: Pan เกิน +50.8us เฉลี่ย, Tilt ขาด -69.4us เฉลี่ย
# ปรับ: PAN -152-51=-203, TILT -62+69=+7
# ⚠️ Pan ยังมี scatter สูง (ผลต่างแกว่ง +19 ถึง +83 ในชุดนี้) อาจมี error แบบ scale ไม่ใช่แค่ bias คงที่ — Tilt นิ่งกว่ามาก (-60 ถึง -75) น่าเชื่อถือกว่า
# Recalibrate อีกครั้ง 2026-10-21 หลังเปลี่ยนเป้าใหม่ (8cm) + วัด CAM_LASER_HEIGHT_OFFSET_CM ใหม่เป็น 10.5cm — ค่าเดิมด้านบนใช้ไม่ได้แล้ว (เลเซอร์เพี้ยนไปซ้าย-บนของเป้าทุกระยะ)
# วิธี: ผู้ใช้ลากสไลเดอร์มือจนเลเซอร์เข้ากลางเป้าที่ 100/150/200cm ได้ Pan~1330-1340us, Tilt~610-620us ทั้ง 3 ระยะ (นิ่งมาก แทบไม่ขึ้นกับระยะ)
# เทียบกับค่าที่ระบบคำนวณอัตโนมัติที่ 150cm (Pan=1418us, Tilt=564us, ยืนยันจาก /api/aimingtest/detect) ได้ผลต่าง: Pan ต้องลด 78us, Tilt ต้องเพิ่ม 56us
# Fine-tune รอบ 2 เมื่อ 2026-10-21 (หลัง recalibrate FOCAL_LENGTH_PX=616 ด้วย) จากข้อมูล /aiming-test จริง 5 trial ที่ตรวจจับได้ปกติ
# (ที่ 100/150cm): Δx เป็นบวกทุกครั้ง (เพี้ยนขวาเฉลี่ย 8.7cm) Δy เป็นบวกทุกครั้ง (เพี้ยนบนเฉลี่ย 1.9cm) — เฉลี่ยมุมเพี้ยนจริง (atan ต่อ sample แล้วเฉลี่ย): Pan ~4.25° ขวา, Tilt ~0.91° บน
# แปลงเป็น pulse (11.11us/deg): Pan ต้องเพิ่ม +47us (เลี้ยวซ้ายดึงกลับจากขวา), Tilt ต้องเพิ่ม +10us (ก้มลงดึงกลับจากบน)
# Fine-tune รอบ 3 เมื่อ 2026-10-21 จากข้อมูล /aiming-test จริง 9 trial (ทดสอบ 3 ตำแหน่งต่อระยะ: กลาง/ขยับซ้าย/ขยับขวาจากจุดกลาง, ครบ 100/150/200cm)
# Δx กลับเป็นลบทุกครั้ง (เพี้ยนซ้ายเฉลี่ย -2.94cm คือปรับรอบก่อนเกินไปนิดหน่อย) Δy ยังบวกอยู่ (เพี้ยนบนเฉลี่ย 2.0cm ใกล้เดิม การแก้รอบก่อนเบาไป)
# เฉลี่ยมุมจาก atan ต่อ sample (ไม่ใช่เฉลี่ย cm ก่อนแปลง): Pan ~1.12° ซ้าย, Tilt ~0.77° บน -> pulse: Pan ต้องลด ~12us, Tilt ต้องเพิ่มอีก ~9us
# รอบ 3 (-246/82) ผล: Pan ดีขึ้น (Δx̄=+1.69cm) แต่ Tilt overshoot หนัก (Δȳ=-6.81cm) — Tilt ไวกว่าสมมติฐาน 11.11us/deg ราว 4 เท่า
# รอบ 4 (2026-10-21): ใช้ข้อมูลทั้ง 3 รอบหาจุด bias=0 แทนการปรับทีละขั้น (มุมเฉลี่ยแบบมีเครื่องหมาย ต่อ sample)
#   Pan: -281→+4.25°, -234→-1.12°, -246→+0.67° → linear regression จุดศูนย์ ≈ -242
#   Tilt: 63→+0.94°, 73→+0.77°, 82→-2.66° → สอดแทรกระหว่าง 73/82 (2 รอบหลังข้อมูลเยอะกว่า) จุดศูนย์ ≈ 75
# ⚠️ ถ้าต้องปรับ Tilt ต่อ ให้ขยับทีละ 1-3us เท่านั้น
# รอบ 5 (2026-10-21): ผล -242/75 → Hit 66.7%, Δx̄=0 (Pan ตรงแล้ว), Δy ต่ำลงคงที่ ~1.27° ทุกระยะ (−2.2/−3.3/−4.5cm ที่ 100/150/200)
#   ผู้ใช้ขอยกขึ้นนิดเดียว → Tilt ลด 3us (ลด pulse = เงยขึ้น) คาดยกขึ้น ~0.6°
# รอบ 6 (2026-10-22): เริ่มใช้ลูป Home + เข้าเป้าจากทิศเดียว (แวะ +60µs) พร้อม Tilt 72 → Δy ต่ำลงทุกค่า (มุมเฉลี่ย −2.0° ลง)
#   สาเหตุหลักคือ backlash ถูกตรึงไว้ทิศเดียวจากวิธีเข้าเป้าใหม่ ไม่ใช่การลด 3us → ต้องจูน offset ใหม่สำหรับวิธีเข้าเป้านี้
#   ความไว Tilt ยังไม่แน่นอน (0.09–0.38 °/us) เลือกลด 7us ซึ่งให้ผลอยู่ในวงแดงได้ทั้งช่วงนั้น
PAN_OFFSET_US = -242
TILT_OFFSET_US = 65

# ทิศทางเซอร์โว Pan: ผู้ใช้ทดสอบแล้ว 2026-09-11 (ขยับจาก 1500 ไปทาง 1400us=เลี้ยวขวา, ไปทาง 1600us=เลี้ยวซ้าย)
# แปลว่า pulse เพิ่ม=เลี้ยวซ้าย ส่วน error_x บวก (เป้าอยู่ขวา) ต้องเลี้ยวขวา=ลด pulse จึงต้องคูณ -1
PAN_DIRECTION = -1

# ทิศทางเซอร์โว Tilt: ผู้ใช้ทดสอบแล้ว 2026-09-11 — ทั้งช่วง 500-1300us มีแต่ก้มลง ไม่มีเงยขึ้นเลย (500us คือจุดเงยสุดที่ทำได้)
# แปลว่า pulse เพิ่ม=ก้มลงมากขึ้น ตรงกับที่ error_y บวก (เป้าอยู่ล่าง) ต้องก้มลงมากขึ้น จึงไม่ต้องกลับทิศ (+1)
TILT_DIRECTION = 1

FOCAL_LENGTH_PX = 616    # recalibrate อีกครั้ง 2026-10-21 จากผลทดสอบ /distance-test 30 ครั้งกับเป้าใหม่ 8cm (วัดจริงยืนยันแล้วว่า 8.0cm ถูกต้อง ไม่ใช่ปัญหาขนาดเป้า)
                          # ย้อนคำนวณ f=D_จริง*P/W จาก 3 ระยะ: 100cm->637.5, 150cm->610.9, 200cm->600.0 เฉลี่ย=616 (ค่าเดิม 511 จากเป้า 3cm รอบก่อนใช้ไม่ได้แล้ว)
MARKER_WIDTH_CM = 8.0    # เส้นผ่านศูนย์กลางจุดแดงเป้าหมาย (ประวัติ 2.5→3.0→8.0 — เปลี่ยนเป็น 8.0 เมื่อ 2026-10-20 ตามแผ่นเป้า A4 ใหม่) ดู PUPPY_PI_THESIS_SYNC.md
HIT_RADIUS_CM = 4.0      # เกณฑ์ตัดสิน "เข้าเป้า" ของหน้า /aiming-test — E <= ค่านี้ (3.4.5 ในเล่ม)

# --- ไฟล์เก็บผลทดลอง 2 ชุด (แยกกันคนละหน้า 2026-09-17) ---
DISTANCE_TEST_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'distance_test_data.json')
AIMING_TEST_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'aiming_test_data.json')  # ⚠️ รูปแบบเก่า (องศา) เลิกใช้แล้ว 2026-10-20 เก็บไว้เฉยๆ ไม่ลบ ดู PUPPY_PI_THESIS_SYNC.md
AIMING_TEST_FILE_V2 = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'aiming_test_data_v2.json')  # รูปแบบใหม่ (Δx/Δy หน่วย cm อ่านจากแผ่นเป้า) ใช้งานจริงตั้งแต่ 2026-10-20

# --- Battery config ---
# แบตเตอรี่หุ่นยนต์เป็น Li-ion 2S — ประมาณช่วงแรงดันใช้งานจริงเพื่อคำนวณ % (ไม่ใช่ค่าจากโรงงาน เป็นค่าประมาณทั่วไปของ 2S Li-ion)
BATTERY_MIN_MV = 6400   # ~0% (แรงดันต่ำสุดที่ยังพอทำงานได้)
BATTERY_MAX_MV = 8400   # ~100% (ชาร์จเต็ม 2S)

# --- Video feed (ภาพกล้องพร้อม overlay ตรวจจับ) ---
VIDEO_FEED_INTERVAL = 0.15  # วินาที ต่อเฟรมที่ส่งออกหน้าเว็บ (กล้องจริงได้แค่ ~6.3fps อยู่แล้ว)

# ========== Global State ==========
ros = None
servo_pub = None
action_srv = None
battery_sub = None
laser = None

battery_state = {
    'voltage_mv': None,
    'voltage_v': None,
    'percent': None,
}

servo_state = {
    'pan': {'position': 1500, 'angle': 0},     # id=10
    'tilt': {'position': 500, 'angle': 0},     # id=11 — เริ่มที่ 500us (จุดเริ่มต้น/0° ตาม calibration)
    'laser': False
}

def load_json_records(path):
    if os.path.exists(path):
        try:
            with open(path) as f:
                return json.load(f)
        except Exception as e:
            print(f"⚠️ อ่านไฟล์ {path} ไม่สำเร็จ: {e}")
    return []

def save_json_records(path, records):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(records, f, ensure_ascii=False, indent=2)

# แต่ละจุด: real_distance_cm, computed_distance_cm, error_cm, error_pct, note, timestamp
distance_test_records = load_json_records(DISTANCE_TEST_FILE)
# แต่ละจุด: distance_cm, computed_pan_deg, computed_tilt_deg, actual_pan_deg, actual_tilt_deg, error_pan_deg, error_tilt_deg, error_total_deg, note, timestamp
aiming_test_records = load_json_records(AIMING_TEST_FILE)  # รูปแบบเก่า (องศา) — เก็บไว้เฉยๆ ไม่ใช้แล้ว
# แต่ละจุด (v2, 2026-10-20): distance_cm, status ('ok'/'off_target'/'not_detected'), dx_cm, dy_cm, E_cm, hit, note, timestamp
aiming_test_records_v2 = load_json_records(AIMING_TEST_FILE_V2)

autotrack_thread = None
autotrack_running = threading.Event()
autotrack_status = {
    'active': False,
    'found': False,
    'pose': 'ยืน',
    'pan_us': 1500,
    'tilt_us': 500,
    'error_x': 0,
    'error_y': 0,
    'distance_cm': None,
    'message': 'หยุดทำงาน',
}

# ========== Helper Functions ==========
def position_to_angle(position, min_pos=500, max_pos=2500):
    """แปลง pulse width เป็นองศา
    500us = -90°, 1500us = 0°, 2500us = +90°
    """
    if position <= min_pos:
        return -90
    elif position >= max_pos:
        return 90
    else:
        # Linear interpolation
        return (position - 1500) * 180 / (max_pos - min_pos)

def angle_to_position(angle, min_pos=500, max_pos=2500):
    """แปลงองศา เป็น pulse width"""
    angle = max(-90, min(90, angle))  # Clamp -90 to +90
    return int(1500 + angle * (max_pos - min_pos) / 180)

# ========== ROS Functions ==========
def _on_battery(msg):
    """callback รับค่าแบตเตอรี่ (mV) จาก /ros_robot_controller/battery (std_msgs/UInt16)"""
    mv = msg.get('data')
    if mv is None:
        return
    percent = round((mv - BATTERY_MIN_MV) / (BATTERY_MAX_MV - BATTERY_MIN_MV) * 100)
    percent = max(0, min(100, percent))
    battery_state.update({'voltage_mv': mv, 'voltage_v': round(mv / 1000.0, 2), 'percent': percent})

def init_ros():
    """เชื่อมต่อ ROS — สร้าง roslibpy.Ros object แค่ครั้งเดียวตลอดอายุแอป (reactor ของ Twisted รันได้แค่ครั้งเดียวต่อโปรเซส)
    ถ้าเชื่อมต่อไม่ติด (เช่น ROS/rosbridge ยังไม่พร้อมตอนแอปเพิ่งเริ่ม) ให้เรียกซ้ำได้เรื่อยๆ — จะใช้ .connect() ไม่สร้าง object ใหม่"""
    global ros, servo_pub, action_srv, battery_sub
    try:
        if ros is None:
            ros = roslibpy.Ros(host=ROS_HOST, port=ROS_PORT)
            ros.run()
        elif not ros.is_connected:
            ros.connect()

        for _ in range(30):  # รอเชื่อมต่อสูงสุด ~6 วินาทีต่อ 1 ครั้งที่เรียก
            if ros.is_connected:
                break
            time.sleep(0.2)
        if not ros.is_connected:
            raise RuntimeError("Failed to connect to ROS")

        if servo_pub is None:
            servo_pub = roslibpy.Topic(ros, '/ros_robot_controller/pwm_servo/set_state',
                                       'ros_robot_controller/SetPWMServoState')
            action_srv = roslibpy.Service(ros, '/puppy_control/runActionGroup', 'puppy_control/SetRunActionName')
            battery_sub = roslibpy.Topic(ros, '/ros_robot_controller/battery', 'std_msgs/UInt16')
            battery_sub.subscribe(_on_battery)
        print("✓ เชื่อมต่อ ROS สำเร็จ")
        return True
    except Exception as e:
        print(f"❌ เชื่อมต่อ ROS ล้มเหลว: {e}")
        return False

_ros_connect_lock = threading.Lock()

def _try_connect_async():
    """เรียก init_ros() ใน thread แยก กันกรณี ros.connect() ค้างตลอดกาล (พบจริง) ไม่ให้ลาก watchdog loop หลักค้างตามไปด้วย"""
    if _ros_connect_lock.acquire(blocking=False):
        try:
            init_ros()
        finally:
            _ros_connect_lock.release()

def ros_watchdog():
    """คอยเช็คทุก 5 วินาทีว่า ROS ยังเชื่อมต่ออยู่ไหม ถ้าหลุด/เชื่อมไม่ติดตอนแอปเพิ่งเริ่ม (ROS ยังไม่พร้อม) จะลองเชื่อมต่อใหม่ให้เอง
    ⚠️ พบจริง 2026-09-15: การเรียก init_ros() ซ้ำในโปรเซสเดิม (ใช้ ros.connect() กับ object เดิม) บางครั้งไม่ยอมเชื่อมต่อกลับจริง
    (ไม่ error แต่ก็ไม่ reconnect ให้) ทั้งที่ ROS ฝั่งหุ่นยนต์ทำงานปกติดี — ไม่ทราบสาเหตุแน่ชัด สงสัยข้อจำกัดภายในของ
    roslibpy/Twisted reactor ที่ reconnect จาก state เดิมไม่ได้เต็มรูปแบบ ทางแก้ที่ทนทานกว่า: ถ้าเชื่อมต่อไม่ติดต่อเนื่องนานเกินไป
    ให้ยอม "ฆ่าตัวเองแล้วให้ systemd (Restart=on-failure) รันโปรเซสใหม่ทั้งหมด" แทน เพราะรีสตาร์ททั้งโปรเซสยืนยันแล้วว่าใช้ได้จริงเสมอ
    ⚠️ แก้เพิ่ม 2026-10-21: เดิมเรียก init_ros() แบบ synchronous ตรงใน loop นี้ — ถ้า ros.connect() ค้าง (ตามที่พบจริงด้านบน)
    จะทำให้ loop ของ watchdog เองค้างตามไปด้วย วนไปถึงจุดนับเวลา timeout ไม่ได้เลย (เคสจริง: ค้างแบบนี้นานหลายชั่วโมงไม่ฟื้นเอง
    ไม่มี log อะไรออกมาเพิ่มเติมเลยหลังจากนั้น) เปลี่ยนมาเรียก init_ros() ใน thread แยกแทน (fire-and-forget ผ่าน _try_connect_async)
    เพื่อให้ loop หลักเดินต่อได้เสมอไม่ว่า connect จะค้างแค่ไหนก็ตาม นับเวลาด้วย wall-clock จริงไม่ใช่จำนวนครั้งที่ init_ros() คืนค่า"""
    disconnected_since = None
    MAX_SECONDS_BEFORE_RESTART = 30
    while True:
        # servo_pub is None: init_ros() หมดเวลารอแล้วโยน error ก่อนสร้าง topic แต่ roslibpy ต่อติดเองทีหลังในพื้นหลัง
        # (พบจริง 2026-10-28 หลังบูต) — is_connected เป็น True แต่ไม่มี publisher/subscriber ต้องเรียก init_ros() ให้สร้างต่อ
        if ros is None or not ros.is_connected or servo_pub is None:
            if disconnected_since is None:
                disconnected_since = time.time()
            threading.Thread(target=_try_connect_async, daemon=True).start()
            if time.time() - disconnected_since >= MAX_SECONDS_BEFORE_RESTART:
                print(f"❌ เชื่อมต่อ ROS ไม่ติดต่อเนื่องเกิน {MAX_SECONDS_BEFORE_RESTART}วิ — ยอมปิดโปรเซสให้ systemd รันใหม่ทั้งหมด")
                os._exit(1)  # ให้ systemd (Restart=on-failure) จัดการรันโปรเซสใหม่สะอาดๆ แทนการพยายาม reconnect ในโปรเซสเดิมต่อไป
        else:
            disconnected_since = None
        time.sleep(5)

def run_action_group(name):
    """สั่งเล่นท่าสำเร็จรูปของหุ่นยนต์ (ต้องมีนามสกุล .d6ac ต่อท้ายเสมอ)"""
    if action_srv is None:
        return False
    try:
        action_srv.call(roslibpy.ServiceRequest({'name': name, 'wait': True}))
        return True
    except Exception as e:
        print(f"❌ สั่งท่าทางล้มเหลว: {e}")
        return False

def init_laser():
    """เชื่อมต่อเลเซอร์"""
    global laser
    try:
        from gpiozero import LED
        laser = LED(7)  # GPIO7
        print("✓ เชื่อมต่อเลเซอร์สำเร็จ")
        return True
    except Exception as e:
        print(f"⚠️ ไม่สามารถเชื่อมต่อเลเซอร์: {e}")
        laser = None
        return False

def set_servo(servo_id, position):
    """สั่งเซอร์โว"""
    if servo_pub is None:
        return False
    try:
        position = max(MIN_POS, min(MAX_POS, position))  # Clamp
        # ใช้ format เดียวกับเฟล์เก่า
        msg = {
            "state": [{
                "id": [servo_id], 
                "position": [int(position)], 
                "offset": [0]
            }], 
            "duration": 0.6
        }
        servo_pub.publish(roslibpy.Message(msg))
        return True
    except Exception as e:
        print(f"❌ สั่งเซอร์โวล้มเหลว: {e}")
        return False


def compute_aim(cx, cy, img_w, img_h, pixel_width):
    """คำนวณตำแหน่งเซอร์โว Pan/Tilt แบบเรขาคณิตจากจุดที่ตรวจจับได้ในภาพ 1 เฟรม (ไม่คลัมป์ขอบเขต ไม่สั่งเซอร์โว ไม่ยุ่งกับท่าหุ่นยนต์)
    แยกออกมาจาก autotrack_loop() เพื่อให้ใช้ร่วมกับหน้า Calibration Tool ได้ (ดูค่าที่ระบบคำนวณได้โดยไม่ต้องรัน Auto-Track เต็มรูปแบบ)"""
    center_x, center_y = img_w / 2, img_h / 2
    error_x = cx - center_x
    error_y = cy - center_y

    distance_cm = round(FOCAL_LENGTH_PX * MARKER_WIDTH_CM / pixel_width, 1) if pixel_width > 0 else None

    pan_angle_deg = math.degrees(math.atan2(error_x, FOCAL_LENGTH_PX))
    tilt_cam_angle_deg = math.degrees(math.atan2(error_y, FOCAL_LENGTH_PX))
    parallax_deg = math.degrees(math.atan2(CAM_LASER_HEIGHT_OFFSET_CM, distance_cm)) if distance_cm else 0
    tilt_angle_deg = tilt_cam_angle_deg + parallax_deg

    pan_us = 1500 + PAN_DIRECTION * pan_angle_deg * US_PER_DEGREE + PAN_OFFSET_US
    tilt_us = 500 + TILT_DIRECTION * tilt_angle_deg * US_PER_DEGREE + TILT_OFFSET_US

    return {
        'error_x': error_x, 'error_y': error_y, 'distance_cm': distance_cm,
        'pan_angle_deg': round(pan_angle_deg, 2), 'tilt_angle_deg': round(tilt_angle_deg, 2),
        'pan_us': pan_us, 'tilt_us': tilt_us,
    }


def autotrack_loop():
    """Loop หลักของ Auto-Track — คำนวณมุมเล็งแบบเรขาคณิตสัมบูรณ์จากภาพเดียวทุกรอบ (ไม่ใช่ incremental feedback)
    เหตุผล (ยืนยันจากการทดสอบจริง 2026-09-11): กล้องติดอยู่กับตัวหุ่นยนต์แบบนิ่ง ไม่ได้หมุนตามป้อม Pan-Tilt
    การขยับป้อมจึงไม่ทำให้ error ในภาพเปลี่ยนเลย ต้องคำนวณมุมเล็งตรงๆจากตำแหน่งเป้าในภาพ+ระยะทาง แล้วสั่งเซอร์โวไปตำแหน่งนั้นเลย
    ถ้ามุม Tilt ที่ต้องการ "เงยขึ้น" เกินกว่าเซอร์โวไปถึง (500us คือจุดเงยสุด) จะสั่งเปลี่ยนท่า ยืน->นั่ง ให้ลำตัวเอนช่วยเงยแทน"""
    global autotrack_status
    current_pose = 'stand.d6ac'
    pose_label = {'stand.d6ac': 'ยืน', 'sit.d6ac': 'นั่ง'}

    while autotrack_running.is_set():
        try:
            frame = grab_frame()
        except Exception as e:
            autotrack_status.update({'found': False, 'message': f'ดึงภาพไม่สำเร็จ: {e}'})
            time.sleep(AUTOTRACK_INTERVAL)
            continue

        _annotated, _mask, detections = detect_red(frame)
        img_h, img_w = frame.shape[:2]
        center_x, center_y = img_w / 2, img_h / 2

        if not detections:
            autotrack_status.update({'found': False, 'message': 'ไม่พบเป้าหมาย', 'pose': pose_label[current_pose]})
            time.sleep(AUTOTRACK_INTERVAL)
            continue

        d = detections[0]
        cx, cy = d['centroid']
        pixel_width = d['bbox'][2]

        aim = compute_aim(cx, cy, img_w, img_h, pixel_width)
        error_x, error_y, distance_cm = aim['error_x'], aim['error_y'], aim['distance_cm']
        new_pan = max(MIN_POS, min(MAX_POS, aim['pan_us']))
        new_tilt = aim['tilt_us']

        if new_tilt < TILT_MIN_POS:
            # ต้องเงยขึ้นมากกว่าที่เซอร์โว Tilt ไปถึง (pulse ต่ำกว่าขีดจำกัด) -> ลองเปลี่ยนท่าให้ลำตัวเอนช่วยเงยแทน
            if current_pose == 'stand.d6ac':
                run_action_group('sit.d6ac')
                current_pose = 'sit.d6ac'
                autotrack_status.update({'message': 'เป้าอยู่สูงเกิน Tilt เงยไปถึง — เปลี่ยนท่าเป็นนั่ง', 'pose': 'นั่ง'})
                time.sleep(POSE_SETTLE_SEC)
                continue  # กลับไปตรวจจับใหม่รอบหน้าด้วยเรขาคณิตกล้องแบบใหม่ (มุมเปลี่ยนเพราะลำตัวเอน)
            else:
                new_tilt = TILT_MIN_POS  # นั่งแล้วก็ยังไม่พอ ทำได้แค่นี้เป็นขีดสุด
                autotrack_status['message'] = 'ถึงขีดจำกัดสูงสุดแล้ว (นั่ง + tilt สุดขอบเงย)'
        elif new_tilt > TILT_MAX_POS:
            new_tilt = TILT_MAX_POS
            autotrack_status['message'] = 'เป้าอยู่ต่ำ/ใกล้เกิน tilt สุดขอบก้ม'
        else:
            autotrack_status['message'] = f'ล็อกเป้า error=({error_x:+.0f},{error_y:+.0f})'

        set_servo(PAN_ID, int(new_pan))
        set_servo(TILT_ID, int(new_tilt))

        autotrack_status.update({
            'found': True,
            'pose': pose_label[current_pose],
            'pan_us': int(new_pan),
            'tilt_us': int(new_tilt),
            'error_x': int(error_x),
            'error_y': int(error_y),
            'distance_cm': distance_cm,
        })
        servo_state['pan']['position'] = int(new_pan)
        servo_state['tilt']['position'] = int(new_tilt)

        time.sleep(AUTOTRACK_INTERVAL)

    autotrack_status.update({'active': False, 'message': 'หยุดทำงาน'})

def laser_on():
    """เปิดเลเซอร์"""
    if laser is None:
        return False
    try:
        laser.on()
        servo_state['laser'] = True
        return True
    except Exception as e:
        print(f"❌ เปิดเลเซอร์ล้มเหลว: {e}")
        return False

def laser_off():
    """ปิดเลเซอร์"""
    if laser is None:
        return False
    try:
        laser.off()
        servo_state['laser'] = False
        return True
    except Exception as e:
        print(f"❌ ปิดเลเซอร์ล้มเหลว: {e}")
        return False

# ========== Flask App ==========
app = Flask(__name__)

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="th">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Pan-Tilt Laser Controller</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .page {
            max-width: 1100px;
            margin: 0 auto;
        }
        h1 {
            color: white;
            text-align: center;
            margin-bottom: 20px;
            font-size: 1.9em;
            text-shadow: 0 2px 8px rgba(0,0,0,0.2);
        }
        .dashboard {
            display: grid;
            grid-template-columns: 1.15fr 1fr;
            gap: 20px;
            align-items: start;
        }
        .card {
            background: white;
            border-radius: 14px;
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.18);
            padding: 20px;
        }
        .card h2 {
            color: #667eea;
            font-size: 1.1em;
            margin-bottom: 14px;
            display: flex;
            align-items: center;
            gap: 8px;
        }
        .col { display: flex; flex-direction: column; gap: 20px; }

        /* --- Camera --- */
        .camera-feed {
            width: 100%;
            display: block;
            border-radius: 10px;
            background: #111;
            aspect-ratio: 4 / 3;
            object-fit: cover;
        }
        .camera-caption {
            color: #999;
            font-size: 0.8em;
            margin-top: 8px;
            text-align: center;
        }

        /* --- Status --- */
        .status-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 7px 0;
            font-size: 0.92em;
            border-bottom: 1px solid #f0f0f0;
        }
        .status-row:last-child { border-bottom: none; }
        .status-row strong { color: #444; font-weight: 600; }
        .status-indicator {
            display: inline-block;
            width: 10px;
            height: 10px;
            border-radius: 50%;
            margin-right: 6px;
        }
        .status-indicator.ok { background: #2ed573; }
        .status-indicator.off { background: #ccc; }
        .status-indicator.warn { background: #ffa502; }
        .status-indicator.low { background: #ff4757; }

        /* --- Servo controls --- */
        .servo-readout {
            display: flex;
            justify-content: space-between;
            align-items: baseline;
            margin-bottom: 10px;
        }
        .servo-readout .big-value {
            font-size: 1.4em;
            font-weight: 700;
            color: #333;
        }
        .servo-readout .angle-value {
            color: #667eea;
            font-weight: 600;
            font-size: 1.05em;
        }
        input[type="range"] {
            width: 100%;
            height: 8px;
            border-radius: 5px;
            background: #e2e2ea;
            outline: none;
            -webkit-appearance: none;
        }
        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 24px;
            height: 24px;
            border-radius: 50%;
            background: #667eea;
            cursor: pointer;
            box-shadow: 0 2px 8px rgba(102, 126, 234, 0.5);
        }
        input[type="range"]::-moz-range-thumb {
            width: 24px;
            height: 24px;
            border-radius: 50%;
            background: #667eea;
            cursor: pointer;
            border: none;
            box-shadow: 0 2px 8px rgba(102, 126, 234, 0.5);
        }
        .range-limits {
            display: flex;
            justify-content: space-between;
            font-size: 0.75em;
            color: #aaa;
            margin-top: 4px;
        }
        .safety-note {
            color: #b8860b;
            background: #fff8e1;
            border-radius: 6px;
            padding: 6px 10px;
            font-size: 0.8em;
            margin-bottom: 12px;
        }

        /* --- Laser --- */
        .laser-controls {
            display: flex;
            gap: 10px;
        }
        button {
            flex: 1;
            padding: 12px;
            border: none;
            border-radius: 8px;
            font-size: 0.95em;
            font-weight: 600;
            cursor: pointer;
            transition: transform 0.15s ease, box-shadow 0.15s ease;
        }
        button:hover { transform: translateY(-2px); }
        .btn-laser-on { background: #ff4757; color: white; }
        .btn-laser-on:hover { box-shadow: 0 6px 16px rgba(255, 71, 87, 0.4); }
        .btn-laser-off { background: #2f3542; color: white; }
        .btn-laser-off:hover { box-shadow: 0 6px 16px rgba(47, 53, 66, 0.4); }
        .btn-pose-stand { background: #2ed573; color: white; }
        .btn-pose-stand:hover { box-shadow: 0 6px 16px rgba(46, 213, 115, 0.4); }
        .btn-pose-sit { background: #ffa502; color: white; }
        .btn-pose-sit:hover { box-shadow: 0 6px 16px rgba(255, 165, 2, 0.4); }
        .btn-pose-lie { background: #57606f; color: white; }
        .btn-pose-lie:hover { box-shadow: 0 6px 16px rgba(87, 96, 111, 0.4); }
        .laser-status {
            text-align: center;
            padding: 12px;
            margin-top: 12px;
            border-radius: 8px;
            font-weight: 600;
        }
        .laser-status.on { background: #ffebee; color: #c41c3b; }
        .laser-status.off { background: #f5f5f5; color: #777; }

        /* --- Auto-Track --- */
        .autotrack-card { margin-bottom: 20px; }
        .autotrack-controls { display: flex; gap: 10px; margin-bottom: 16px; }
        .btn-autotrack-start { background: #5352ed; color: white; }
        .btn-autotrack-start:hover { box-shadow: 0 6px 16px rgba(83, 82, 237, 0.4); }
        .btn-autotrack-stop { background: #ff6348; color: white; }
        .btn-autotrack-stop:hover { box-shadow: 0 6px 16px rgba(255, 99, 72, 0.4); }
        .autotrack-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
        }
        .autotrack-stat {
            background: #f8f9fb;
            border-radius: 8px;
            padding: 10px 12px;
        }
        .autotrack-stat span { display: block; color: #999; font-size: 0.78em; margin-bottom: 3px; }
        .autotrack-stat strong { color: #333; font-size: 0.98em; }
        @media (max-width: 650px) {
            .autotrack-grid { grid-template-columns: repeat(2, 1fr); }
        }

        @media (max-width: 850px) {
            .dashboard { grid-template-columns: 1fr; }
        }
        @media (max-width: 480px) {
            h1 { font-size: 1.4em; }
            .card { padding: 16px; }
        }
    </style>
</head>
<body>
<div class="page">
    <h1>🎯 Pan-Tilt Laser Controller</h1>
    <div style="text-align:center; margin-bottom:16px;">
        <a href="/distance-test" style="color:white; opacity:0.9;">📏 ทดสอบความแม่นยำระยะทาง</a>
        &nbsp;|&nbsp;
        <a href="/aiming-test" style="color:white; opacity:0.9;">🎯 ทดสอบความแม่นยำการชี้เป้า</a>
        &nbsp;|&nbsp;
        <a href="/walk-test" style="color:white; opacity:0.9;">🐕 ทดสอบการเดิน</a>
    </div>

    <div class="card autotrack-card">
        <h2>🎯 Auto-Track — ตรวจจับเป้าสีแดงอัตโนมัติ</h2>
        <div class="autotrack-controls">
            <button class="btn-autotrack-start" onclick="startAutotrack()">▶️ เริ่ม Auto-Track</button>
            <button class="btn-autotrack-stop" onclick="stopAutotrack()">⏹️ หยุด</button>
        </div>
        <div class="autotrack-grid">
            <div class="autotrack-stat"><span>สถานะ</span><strong id="at-message">หยุดทำงาน</strong></div>
            <div class="autotrack-stat"><span>พบเป้า</span><strong id="at-found">-</strong></div>
            <div class="autotrack-stat"><span>ท่าหุ่นยนต์</span><strong id="at-pose">-</strong></div>
            <div class="autotrack-stat"><span>Pan / Tilt</span><strong id="at-servo">- / -</strong></div>
            <div class="autotrack-stat"><span>Error (x,y)</span><strong id="at-error">-</strong></div>
            <div class="autotrack-stat"><span>ระยะทาง</span><strong id="at-distance">- (ยังไม่ calibrate)</strong></div>
        </div>
    </div>

    <div class="dashboard">
        <!-- ===== Left column: camera + status ===== -->
        <div class="col">
            <div class="card">
                <h2>📷 กล้องสด (พร้อมกรอบตรวจจับ)</h2>
                <img class="camera-feed" src="/video_feed" alt="camera feed with detection overlay">
                <div class="camera-caption">กรอบเขียว = วัตถุที่ล็อกเป้า, จุดเหลือง = จุดศูนย์กลางที่เลเซอร์จะเล็ง, กากบาทฟ้า = กึ่งกลางเฟรม</div>
            </div>

            <div class="card">
                <h2>📊 สถานะระบบ</h2>
                <div class="status-row">
                    <strong>ROS</strong>
                    <span><span class="status-indicator ok"></span>เชื่อมต่อ</span>
                </div>
                <div class="status-row">
                    <strong>แบตเตอรี่หุ่นยนต์</strong>
                    <span><span class="status-indicator off" id="battery-indicator"></span><span id="battery-text">กำลังอ่านค่า...</span></span>
                </div>
                <div class="status-row">
                    <strong>เลเซอร์ (GPIO7)</strong>
                    <span><span class="status-indicator off" id="laser-status-indicator"></span><span id="laser-status-text">ปิด</span></span>
                </div>
                <div class="status-row">
                    <strong>IP Address</strong>
                    <span>{{ ip_address }}:{{ flask_port }}</span>
                </div>
            </div>
        </div>

        <!-- ===== Right column: controls ===== -->
        <div class="col">
            <div class="card">
                <h2>📍 Pan (ซ้าย-ขวา) — id 10</h2>
                <div class="servo-readout">
                    <span class="big-value"><span id="pan-position">1500</span> µs</span>
                    <span class="angle-value" id="pan-angle">0°</span>
                </div>
                <input type="range" id="pan-slider" min="{{ min_pos }}" max="{{ max_pos }}" value="1500" step="10">
                <div class="range-limits"><span>{{ min_pos }}us (−90°)</span><span>{{ max_pos }}us (+90°)</span></div>
            </div>

            <div class="card">
                <h2>📍 Tilt (บน-ล่าง) — id 11</h2>
                <div class="safety-note">⚠️ จำกัดไว้ไม่เกิน {{ tilt_max_pos }}us เพื่อความปลอดภัยเชิงกลไก</div>
                <div class="servo-readout">
                    <span class="big-value"><span id="tilt-position">500</span> µs</span>
                    <span class="angle-value" id="tilt-angle">-90°</span>
                </div>
                <input type="range" id="tilt-slider" min="{{ tilt_min_pos }}" max="{{ tilt_max_pos }}" value="500" step="10">
                <div class="range-limits"><span>{{ tilt_min_pos }}us</span><span>{{ tilt_max_pos }}us</span></div>
            </div>

            <div class="card">
                <h2>🐕 ท่าทางหุ่นยนต์</h2>
                <div class="safety-note">💡 ถ้า Tilt ติดขีดจำกัดจนชี้เป้าไม่ถึง ลองปรับท่าหุ่นยนต์แทน</div>
                <div class="laser-controls">
                    <button class="btn-pose-stand" onclick="runAction('stand.d6ac')">🧍 ยืน</button>
                    <button class="btn-pose-sit" onclick="runAction('sit.d6ac')">🐾 นั่ง</button>
                    <button class="btn-pose-lie" onclick="runAction('lie_down.d6ac')">🐕 หมอบ</button>
                </div>
                <div class="laser-status off" id="pose-status">พร้อมสั่งท่าทาง</div>
            </div>

            <div class="card">
                <h2>🔴 เลเซอร์ (GPIO 7)</h2>
                <div class="laser-controls">
                    <button class="btn-laser-on" onclick="laserOn()">🔴 เปิดเลเซอร์</button>
                    <button class="btn-laser-off" onclick="laserOff()">⚫ ปิดเลเซอร์</button>
                </div>
                <div class="laser-status off" id="laser-status">⚫ เลเซอร์: ปิด</div>
            </div>
        </div>
    </div>
</div>

    <script>
        const MIN_POS = {{ min_pos }};
        const MAX_POS = {{ max_pos }};

        // ฟังก์ชันแปลงตำแหน่งเป็นองศา
        function positionToAngle(position) {
            if (position <= MIN_POS) return -90;
            if (position >= MAX_POS) return 90;
            return (position - 1500) * 180 / (MAX_POS - MIN_POS);
        }

        // ฟังก์ชันแปลงองศาเป็นตำแหน่ง
        function angleToPosition(angle) {
            angle = Math.max(-90, Math.min(90, angle));
            return Math.round(1500 + angle * (MAX_POS - MIN_POS) / 180);
        }

        // Update Pan
        const panSlider = document.getElementById('pan-slider');
        panSlider.addEventListener('input', function() {
            const pos = parseInt(this.value);
            const angle = positionToAngle(pos);

            document.getElementById('pan-position').textContent = pos;
            document.getElementById('pan-angle').textContent = angle.toFixed(1) + '°';

            fetch('/api/servo/set', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ id: 10, position: pos })
            });
        });

        // Update Tilt
        const tiltSlider = document.getElementById('tilt-slider');
        tiltSlider.addEventListener('input', function() {
            const pos = parseInt(this.value);
            const angle = positionToAngle(pos);

            document.getElementById('tilt-position').textContent = pos;
            document.getElementById('tilt-angle').textContent = angle.toFixed(1) + '°';

            fetch('/api/servo/set', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ id: 11, position: pos })
            });
        });

        // Auto-Track Control
        let autotrackPoller = null;
        function ensurePolling() {
            if (!autotrackPoller) autotrackPoller = setInterval(pollAutotrackStatus, 1000);
            pollAutotrackStatus();  // ดึงสถานะทันทีไม่ต้องรอรอบแรกของ interval
        }
        function startAutotrack() {
            fetch('/api/autotrack/start', { method: 'POST' })
                .then(r => r.json())
                .then(data => {
                    if (!data.success) {
                        document.getElementById('at-message').textContent = data.error || 'เริ่มไม่สำเร็จ';
                    }
                    // เริ่ม poll เสมอไม่ว่าผลจะสำเร็จหรือ error "ทำงานอยู่แล้ว" (แปลว่ามันรันอยู่จริง แค่ session นี้ไม่ได้เริ่มเอง)
                    ensurePolling();
                });
        }
        function stopAutotrack() {
            fetch('/api/autotrack/stop', { method: 'POST' })
                .then(() => {
                    if (autotrackPoller) { clearInterval(autotrackPoller); autotrackPoller = null; }
                    document.getElementById('at-message').textContent = 'หยุดทำงาน';
                });
        }
        function pollAutotrackStatus() {
            fetch('/api/autotrack/status')
                .then(r => r.json())
                .then(s => {
                    document.getElementById('at-message').textContent = s.message;
                    document.getElementById('at-found').textContent = s.found ? '✅ พบ' : '❌ ไม่พบ';
                    document.getElementById('at-pose').textContent = s.pose;
                    document.getElementById('at-servo').textContent = `${s.pan_us}us / ${s.tilt_us}us`;
                    document.getElementById('at-error').textContent = `(${s.error_x}, ${s.error_y}) px`;
                    document.getElementById('at-distance').textContent = s.distance_cm ? `${s.distance_cm} ซม.` : '- (ยังไม่ calibrate)';
                    if (!s.active && autotrackPoller) { clearInterval(autotrackPoller); autotrackPoller = null; }
                });
        }

        // Pose Control
        const poseNames = {'stand.d6ac': 'ยืน', 'sit.d6ac': 'นั่ง', 'lie_down.d6ac': 'หมอบ'};
        function runAction(name) {
            const statusEl = document.getElementById('pose-status');
            statusEl.textContent = `⏳ กำลังสั่ง ${poseNames[name]}...`;
            fetch('/api/action', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ name: name })
            })
                .then(r => r.json())
                .then(data => {
                    statusEl.textContent = data.success
                        ? `✅ สั่งท่า ${poseNames[name]} แล้ว`
                        : `❌ สั่งไม่สำเร็จ: ${data.error || ''}`;
                })
                .catch(e => { statusEl.textContent = `❌ เชื่อมต่อไม่สำเร็จ: ${e}`; });
        }

        // Laser Control
        function laserOn() {
            fetch('/api/laser/on', { method: 'POST' })
                .then(r => r.json())
                .then(data => {
                    if (data.success) {
                        document.getElementById('laser-status').className = 'laser-status on';
                        document.getElementById('laser-status').textContent = '🔴 เลเซอร์: เปิด';
                        document.getElementById('laser-status-indicator').className = 'status-indicator ok';
                        document.getElementById('laser-status-text').textContent = 'เปิด';
                    }
                });
        }

        function laserOff() {
            fetch('/api/laser/off', { method: 'POST' })
                .then(r => r.json())
                .then(data => {
                    if (data.success) {
                        document.getElementById('laser-status').className = 'laser-status off';
                        document.getElementById('laser-status').textContent = '⚫ เลเซอร์: ปิด';
                        document.getElementById('laser-status-indicator').className = 'status-indicator';
                        document.getElementById('laser-status-text').textContent = 'ปิด';
                    }
                });
        }

        // Battery Status
        function pollBatteryStatus() {
            fetch('/api/status')
                .then(r => r.json())
                .then(s => {
                    const b = s.battery || {};
                    const textEl = document.getElementById('battery-text');
                    const indEl = document.getElementById('battery-indicator');
                    if (b.voltage_v == null) {
                        textEl.textContent = 'ไม่มีข้อมูล';
                        indEl.className = 'status-indicator off';
                        return;
                    }
                    textEl.textContent = `${b.voltage_v} V (${b.percent}%)`;
                    let cls = 'ok';
                    if (b.percent < 15) cls = 'low';
                    else if (b.percent < 35) cls = 'warn';
                    indEl.className = 'status-indicator ' + cls;
                });
        }
        setInterval(pollBatteryStatus, 5000);

        // Update status on load
        window.addEventListener('load', function() {
            ensurePolling();  // sync หน้าเว็บกับสถานะ Auto-Track จริงของ backend ทันที (เผื่อรันอยู่แล้วก่อนโหลดหน้า)
            laserOff();
            pollBatteryStatus();
        });
    </script>
</body>
</html>
"""

VIDEO_FEED_GRACE_FRAMES = 3  # ถ้าเป้าหลุดไม่เกินกี่เฟรมติดกัน ให้ยังโชว์กรอบล่าสุดค้างไว้ก่อน กันภาพกระพริบตอนเป้าเล็ก/ไกล

def gen_video_feed():
    """สตรีม MJPEG ภาพกล้องพร้อม overlay ผลตรวจจับ (กรอบ+จุดล็อกเป้า) ให้เห็นว่าระบบจับวัตถุตรงไหน
    มี hysteresis กันภาพกระพริบ: ถ้าเป้าหายไปแค่ไม่กี่เฟรม (เช่น noise ชั่วครู่) จะยังโชว์กรอบเดิมค้างไว้ก่อน"""
    last_bbox = None
    last_centroid = None
    miss_streak = 0
    while True:
        try:
            frame = grab_frame()
            annotated, _mask, detections = detect_red(frame)

            if detections:
                last_bbox = detections[0]['bbox']
                last_centroid = detections[0]['centroid']
                miss_streak = 0
            elif last_bbox is not None and miss_streak < VIDEO_FEED_GRACE_FRAMES:
                miss_streak += 1
                x, y, w, h = last_bbox
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.circle(annotated, last_centroid, 6, (0, 255, 255), -1)
                cv2.putText(annotated, "TARGET (ค้างจากเฟรมก่อน)", (x, max(0, y - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            else:
                last_bbox = None

            ok, buf = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                continue
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
        except Exception as e:
            print(f"⚠️ video_feed error: {e}")
        time.sleep(VIDEO_FEED_INTERVAL)

@app.route('/video_feed')
def video_feed():
    """ภาพกล้องพร้อม overlay ตรวจจับ (ใช้แทนสตรีมดิบจาก web_video_server)"""
    return Response(gen_video_feed(), mimetype='multipart/x-mixed-replace; boundary=frame')

def pan_us_to_deg(us):
    return round((us - 1500) / US_PER_DEGREE, 2)

def pan_deg_to_us(deg):
    return 1500 + deg * US_PER_DEGREE

def tilt_us_to_deg(us):
    return round((us - 500) / US_PER_DEGREE, 2)

def tilt_deg_to_us(deg):
    return 500 + deg * US_PER_DEGREE

## ---- สถิติ (สูตรตามเล่มบทที่ 3 — ดู PUPPY_PI_THESIS_SYNC.md หัวข้อ 3) ----
def _mean(values):
    return sum(values) / len(values) if values else None

def _sd(values):
    """ส่วนเบี่ยงเบนมาตรฐานแบบตัวอย่าง (หารด้วย n-1 ตามสมการ 3.17-3.18 ในเล่ม) ต้องมีอย่างน้อย 2 ค่า"""
    if not values or len(values) < 2:
        return 0.0
    m = _mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / (len(values) - 1))

def distance_stats(records):
    """สมการ (3.15) e_D ต่อจุด, (3.17)-(3.18) ค่าเฉลี่ย/S.D., (3.19) Accuracy — แยกกลุ่มตามระยะจริง"""
    groups = {}
    for r in records:
        if r.get('real_distance_cm') is None or r.get('computed_distance_cm') is None:
            continue
        groups.setdefault(r['real_distance_cm'], []).append(r)
    result = []
    for real_d in sorted(groups.keys()):
        rows = groups[real_d]
        eD_list = [abs(r['computed_distance_cm'] - real_d) / real_d * 100 for r in rows]
        mean_eD = _mean(eD_list)
        result.append({
            'real_distance_cm': real_d,
            'n': len(rows),
            'mean_computed_cm': round(_mean([r['computed_distance_cm'] for r in rows]), 2),
            'mean_eD_pct': round(mean_eD, 2),
            'sd_eD_pct': round(_sd(eD_list), 2),
            'accuracy_pct': round(100 - mean_eD, 2),
        })
    return result

def aiming_stats(records):
    """สมการ (3.16) E, (3.20) AP, (3.21)-(3.22) RP, (3.23)-(3.24) Hit/Detection rate — รวมทุกจุด (ไม่แยกกลุ่มระยะ)"""
    n = len(records)
    if n == 0:
        return {'n': 0}
    n_detect = sum(1 for r in records if r.get('status') != 'not_detected')
    valid = [r for r in records if r.get('status') == 'ok']
    n_hit = sum(1 for r in valid if r.get('hit'))
    E_list = [r['E_cm'] for r in valid]
    dx_list = [r['dx_cm'] for r in valid]
    dy_list = [r['dy_cm'] for r in valid]
    dx_bar = _mean(dx_list) if dx_list else None
    dy_bar = _mean(dy_list) if dy_list else None
    if valid and dx_bar is not None:
        l_list = [math.sqrt((r['dx_cm'] - dx_bar) ** 2 + (r['dy_cm'] - dy_bar) ** 2) for r in valid]
        AP = round(math.sqrt(dx_bar ** 2 + dy_bar ** 2), 2)
        RP = round(_mean(l_list) + 3 * _sd(l_list), 2)
    else:
        AP = None
        RP = None
    return {
        'n': n,
        'n_detect': n_detect,
        'n_valid': len(valid),
        'n_hit': n_hit,
        'hit_rate_pct': round(n_hit / n * 100, 2),
        'detection_rate_pct': round(n_detect / n * 100, 2),
        'pct_e_le_1cm': round(sum(1 for r in valid if r['E_cm'] <= 1.0) / n * 100, 2),
        'pct_e_le_2cm': round(sum(1 for r in valid if r['E_cm'] <= 2.0) / n * 100, 2),
        'pct_e_le_4cm': round(sum(1 for r in valid if r['E_cm'] <= HIT_RADIUS_CM) / n * 100, 2),
        'mean_E_cm': round(_mean(E_list), 2) if E_list else None,
        'sd_E_cm': round(_sd(E_list), 2) if E_list else None,
        'max_E_cm': round(max(E_list), 2) if E_list else None,
        'dx_bar_cm': round(dx_bar, 2) if dx_bar is not None else None,
        'dy_bar_cm': round(dy_bar, 2) if dy_bar is not None else None,
        'AP_cm': AP,
        'RP_cm': RP,
    }

def _csv_response(rows, fieldnames, filename):
    """ส่งออกเป็น CSV พร้อม UTF-8 BOM ให้ Excel เปิดภาษาไทยได้ถูกต้อง"""
    import csv
    import io
    buf = io.StringIO()
    writer = csv.DictWriter(buf, fieldnames=fieldnames, extrasaction='ignore')
    writer.writeheader()
    for row in rows:
        writer.writerow(row)
    csv_bytes = ('﻿' + buf.getvalue()).encode('utf-8')
    return Response(csv_bytes, mimetype='text/csv',
                     headers={'Content-Disposition': f'attachment; filename="{filename}"'})

# ========== หน้าที่ 1: ทดสอบความแม่นยำระยะทาง (Distance Accuracy Test) ==========
@app.route('/api/disttest/detect', methods=['GET'])
def disttest_detect_api():
    """ตรวจจับเป้า 1 เฟรม คำนวณระยะทางอย่างเดียว (กล้องนิ่งไม่ต้องขยับเซอร์โวเลย) — บังคับยืนก่อนทุกครั้งให้ความสูงกล้องสม่ำเสมอ"""
    run_action_group('stand.d6ac')
    time.sleep(POSE_SETTLE_SEC)
    try:
        frame = grab_frame()
    except Exception as e:
        return jsonify({'found': False, 'error': f'ดึงภาพไม่สำเร็จ: {e}'})

    _annotated, _mask, detections = detect_red(frame)
    if not detections:
        return jsonify({'found': False})

    d = detections[0]
    pixel_width = d['bbox'][2]
    computed_distance_cm = round(FOCAL_LENGTH_PX * MARKER_WIDTH_CM / pixel_width, 1) if pixel_width > 0 else None
    return jsonify({'found': True, 'computed_distance_cm': computed_distance_cm, 'pixel_width': pixel_width})

@app.route('/api/disttest/save', methods=['POST'])
def disttest_save_api():
    data = request.json or {}
    real_d = data.get('real_distance_cm')
    computed_d = data.get('computed_distance_cm')
    error_cm = round(computed_d - real_d, 2) if (real_d is not None and computed_d is not None) else None
    error_pct = round(error_cm / real_d * 100, 2) if (error_cm is not None and real_d) else None
    record = {
        'real_distance_cm': real_d,
        'computed_distance_cm': computed_d,
        'error_cm': error_cm,
        'error_pct': error_pct,
        'note': data.get('note', ''),
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
    }
    distance_test_records.append(record)
    save_json_records(DISTANCE_TEST_FILE, distance_test_records)
    return jsonify({'success': True, 'record': record})

@app.route('/api/disttest/list', methods=['GET'])
def disttest_list_api():
    return jsonify(distance_test_records)

@app.route('/api/disttest/delete', methods=['POST'])
def disttest_delete_api():
    idx = (request.json or {}).get('index')
    if idx is None or not (0 <= idx < len(distance_test_records)):
        return jsonify({'success': False, 'error': 'index ไม่ถูกต้อง'})
    distance_test_records.pop(idx)
    save_json_records(DISTANCE_TEST_FILE, distance_test_records)
    return jsonify({'success': True})

@app.route('/api/disttest/stats', methods=['GET'])
def disttest_stats_api():
    """สถิติแยกตามระยะจริง (ē_D, S.D., Accuracy) — สมการ 3.15, 3.17-3.19 ในเล่ม"""
    return jsonify(distance_stats(distance_test_records))

@app.route('/api/disttest/export.csv', methods=['GET'])
def disttest_export_csv():
    fieldnames = ['real_distance_cm', 'computed_distance_cm', 'error_cm', 'error_pct', 'note', 'timestamp']
    return _csv_response(distance_test_records, fieldnames, 'distance_test_data.csv')

# ========== หน้าที่ 2: ทดสอบความแม่นยำการชี้เป้า (Aiming Accuracy Test) ==========
AIM_HOME_PAN_US = 1500
AIM_HOME_TILT_US = 800
# ก่อนเข้าตำแหน่งเป้า แวะจุดที่เลยเป้าไปทางพัลส์มากก่อน แล้วค่อยถอยเข้าเป้า — ทุก trial จึงเข้าหาเป้าจากทิศเดียวกัน
# (ตัดผล backlash ของเฟือง/ปีกเซอร์โวที่ต่างกันตามทิศที่หมุนเข้ามา ไม่ว่าเป้าจะอยู่ซ้ายหรือขวาของ Home)
AIM_APPROACH_US = 60
SERVO_MOVE_SETTLE_SEC = 0.8  # set_servo สั่ง duration 0.6s + เผื่อให้นิ่ง

def aim_go_home():
    laser_off()
    set_servo(PAN_ID, AIM_HOME_PAN_US)
    set_servo(TILT_ID, AIM_HOME_TILT_US)
    servo_state['pan']['position'] = AIM_HOME_PAN_US
    servo_state['tilt']['position'] = AIM_HOME_TILT_US

@app.route('/api/aimingtest/home', methods=['POST'])
def aimingtest_home_api():
    aim_go_home()
    return jsonify({'success': True, 'pan_us': AIM_HOME_PAN_US, 'tilt_us': AIM_HOME_TILT_US})

@app.route('/api/aimingtest/detect', methods=['GET'])
def aimingtest_detect_api():
    """1 trial: ปิดเลเซอร์+กลับ Home → ยืน รอนิ่ง → ตรวจจับ → แวะจุดเลยเป้า → เข้าเป้า → เปิดเลเซอร์"""
    aim_go_home()
    run_action_group('stand.d6ac')
    time.sleep(max(POSE_SETTLE_SEC, SERVO_MOVE_SETTLE_SEC))
    try:
        frame = grab_frame()
    except Exception as e:
        return jsonify({'found': False, 'error': f'ดึงภาพไม่สำเร็จ: {e}'})

    _annotated, _mask, detections = detect_red(frame)
    if not detections:
        return jsonify({'found': False})

    d = detections[0]
    cx, cy = d['centroid']
    img_h, img_w = frame.shape[:2]
    aim = compute_aim(cx, cy, img_w, img_h, d['bbox'][2])

    pan_us = int(max(MIN_POS, min(MAX_POS, aim['pan_us'])))
    tilt_us = int(max(TILT_MIN_POS, min(TILT_MAX_POS, aim['tilt_us'])))
    set_servo(PAN_ID, min(MAX_POS, pan_us + AIM_APPROACH_US))
    set_servo(TILT_ID, min(TILT_MAX_POS, tilt_us + AIM_APPROACH_US))
    time.sleep(SERVO_MOVE_SETTLE_SEC)
    set_servo(PAN_ID, pan_us)
    set_servo(TILT_ID, tilt_us)
    time.sleep(SERVO_MOVE_SETTLE_SEC)
    servo_state['pan']['position'] = pan_us
    servo_state['tilt']['position'] = tilt_us
    laser_on()

    return jsonify({
        'found': True,
        'pan_us': pan_us,
        'tilt_us': tilt_us,
        'distance_cm': aim['distance_cm'],
        'computed_pan_deg': pan_us_to_deg(pan_us),
        'computed_tilt_deg': tilt_us_to_deg(tilt_us),
        'actual_pan_deg': pan_us_to_deg(pan_us),
        'actual_tilt_deg': tilt_us_to_deg(tilt_us),
    })

@app.route('/api/aimingtest/nudge', methods=['POST'])
def aimingtest_nudge_api():
    """ขยับ Pan หรือ Tilt ทีละ 1 องศา (ปุ่ม +/-) — axis: 'pan'/'tilt', delta_deg: ตัวเลของศาที่จะขยับ (+1 หรือ -1)"""
    data = request.json or {}
    axis = data.get('axis')
    delta_deg = data.get('delta_deg', 0)
    if axis == 'pan':
        new_us = int(max(MIN_POS, min(MAX_POS, servo_state['pan']['position'] + delta_deg * US_PER_DEGREE)))
        set_servo(PAN_ID, new_us)
        servo_state['pan']['position'] = new_us
        return jsonify({'success': True, 'actual_pan_deg': pan_us_to_deg(new_us)})
    elif axis == 'tilt':
        new_us = int(max(TILT_MIN_POS, min(TILT_MAX_POS, servo_state['tilt']['position'] + delta_deg * US_PER_DEGREE)))
        set_servo(TILT_ID, new_us)
        servo_state['tilt']['position'] = new_us
        return jsonify({'success': True, 'actual_tilt_deg': tilt_us_to_deg(new_us)})
    return jsonify({'success': False, 'error': 'axis ไม่ถูกต้อง'})

@app.route('/api/aimingtest/save', methods=['POST'])
def aimingtest_save_api():
    """บันทึก 1 trial ตามวิธีในเล่ม (3.4.4-3.4.5): status 'ok' ต้องมี dx_cm/dy_cm (อ่านจากช่องตาราง 5mm บนแผ่นเป้า)
    status 'off_target' (เลเซอร์นอกแผ่นเป้า) หรือ 'not_detected' (ระบบตรวจไม่พบ) ไม่มี dx/dy — ห้ามข้าม/ลบทิ้งตามที่เล่มระบุ"""
    data = request.json or {}
    status = data.get('status', 'ok')
    dx = data.get('dx_cm')
    dy = data.get('dy_cm')
    record = {
        'distance_cm': data.get('distance_cm'),
        'status': status,
        'note': data.get('note', ''),
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
    }
    if status == 'ok' and dx is not None and dy is not None:
        E = round(math.sqrt(dx ** 2 + dy ** 2), 2)
        record.update({'dx_cm': dx, 'dy_cm': dy, 'E_cm': E, 'hit': E <= HIT_RADIUS_CM})
    aiming_test_records_v2.append(record)
    save_json_records(AIMING_TEST_FILE_V2, aiming_test_records_v2)
    aim_go_home()
    return jsonify({'success': True, 'record': record})

@app.route('/api/aimingtest/list', methods=['GET'])
def aimingtest_list_api():
    return jsonify(aiming_test_records_v2)

@app.route('/api/aimingtest/stats', methods=['GET'])
def aimingtest_stats_api():
    """Hit rate, Detection rate, %E<=1/2/4cm, Ē, S.D., AP, RP — สมการ 3.16, 3.20-3.24 ในเล่ม"""
    return jsonify(aiming_stats(aiming_test_records_v2))

@app.route('/api/aimingtest/export.csv', methods=['GET'])
def aimingtest_export_csv():
    fieldnames = ['distance_cm', 'status', 'dx_cm', 'dy_cm', 'E_cm', 'hit', 'note', 'timestamp']
    return _csv_response(aiming_test_records_v2, fieldnames, 'aiming_test_data.csv')

@app.route('/api/aimingtest/delete', methods=['POST'])
def aimingtest_delete_api():
    idx = (request.json or {}).get('index')
    if idx is None or not (0 <= idx < len(aiming_test_records_v2)):
        return jsonify({'success': False, 'error': 'index ไม่ถูกต้อง'})
    aiming_test_records_v2.pop(idx)
    save_json_records(AIMING_TEST_FILE_V2, aiming_test_records_v2)
    return jsonify({'success': True})

TEST_PAGE_STYLE = """
    * { margin:0; padding:0; box-sizing:border-box; }
    body { font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif; background:linear-gradient(135deg,#667eea 0%,#764ba2 100%); min-height:100vh; padding:20px; }
    .page { max-width:1100px; margin:0 auto; }
    h1 { color:white; text-align:center; margin-bottom:8px; font-size:1.7em; text-shadow:0 2px 8px rgba(0,0,0,0.2); }
    .nav-links { display:block; text-align:center; color:white; margin-bottom:16px; opacity:0.9; }
    .nav-links a { color:white; margin:0 8px; }
    .dashboard { display:grid; grid-template-columns:1.1fr 1fr; gap:20px; align-items:start; }
    .card { background:white; border-radius:14px; box-shadow:0 10px 30px rgba(0,0,0,0.18); padding:20px; margin-bottom:20px; }
    .card h2 { color:#667eea; font-size:1.05em; margin-bottom:12px; }
    .camera-feed { width:100%; display:block; border-radius:10px; background:#111; aspect-ratio:4/3; object-fit:cover; }
    .readout-stat { background:#f8f9fb; border-radius:8px; padding:10px 12px; margin-top:10px; }
    .readout-stat span { display:block; color:#999; font-size:0.78em; margin-bottom:3px; }
    .readout-stat strong { color:#333; font-size:1.05em; }
    button { padding:12px; border:none; border-radius:8px; font-size:0.95em; font-weight:600; cursor:pointer; transition:transform .15s,box-shadow .15s; }
    button:hover { transform:translateY(-2px); }
    .btn-detect { background:#5352ed; color:white; width:100%; margin-top:10px; }
    .btn-save { background:#2ed573; color:white; width:100%; margin-top:10px; }
    .btn-del { background:#ff4757; color:white; padding:6px 10px; font-size:0.8em; }
    .note-input { width:100%; padding:8px; border:1px solid #ddd; border-radius:6px; margin-top:8px; font-size:0.9em; }
    .field-grid { display:grid; grid-template-columns:repeat(2,1fr); gap:10px; margin-top:12px; }
    .field-grid label { display:flex; flex-direction:column; gap:4px; font-size:0.8em; color:#666; }
    .field-grid input { padding:8px; border:1px solid #ddd; border-radius:6px; font-size:0.95em; }
    table { width:100%; border-collapse:collapse; font-size:0.82em; }
    th, td { padding:7px 5px; text-align:center; border-bottom:1px solid #eee; }
    th { color:#999; font-weight:600; font-size:0.78em; }
    .stats-box { background:#eef1ff; border-radius:8px; padding:10px 12px; margin-top:12px; font-size:0.85em; }
    .stats-box div { padding:3px 0; }
    .nudge-grid { display:grid; grid-template-columns:1fr 1fr; gap:16px; margin-top:10px; }
    .nudge-axis { text-align:center; }
    .nudge-axis .val { font-size:1.5em; font-weight:700; color:#333; margin:8px 0; }
    .nudge-btns { display:flex; gap:8px; justify-content:center; }
    .nudge-btns button { background:#5352ed; color:white; width:52px; height:44px; font-size:1.1em; }
    @media (max-width:850px) { .dashboard { grid-template-columns:1fr; } }
"""

DISTANCE_TEST_HTML = """
<!DOCTYPE html>
<html lang="th">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Distance Accuracy Test</title>
<style>""" + TEST_PAGE_STYLE + """</style>
</head>
<body>
<div class="page">
    <h1>📏 ทดสอบความแม่นยำระยะทาง</h1>
    <div class="nav-links"><a href="/">← หน้าควบคุมหลัก</a> | <a href="/aiming-test">🎯 ทดสอบการชี้เป้า →</a></div>

    <div class="dashboard">
        <div class="col">
            <div class="card">
                <h2>📷 กล้องสด</h2>
                <img class="camera-feed" id="cam-img" alt="camera feed (ปิดอยู่ — กดเปิดกล้องเพื่อดู)" style="display:none;">
                <div id="cam-placeholder" style="background:#111; border-radius:10px; aspect-ratio:4/3; display:flex; align-items:center; justify-content:center; color:#666; font-size:0.85em;">กล้องปิดอยู่ (ประหยัดพลังงาน)</div>
                <button style="width:100%; background:#57606f; color:white; margin-top:10px;" onclick="toggleCam()" id="cam-toggle-btn">📷 เปิดกล้อง</button>
                <button class="btn-detect" onclick="detectNow()">🔍 ตรวจจับ (คำนวณระยะทาง)</button>
                <div class="readout-stat"><span>พบเป้า</span><strong id="c-found">-</strong></div>
            </div>
        </div>

        <div class="col">
            <div class="card">
                <h2>📐 บันทึกผลการวัด</h2>
                <div class="field-grid">
                    <label>ระยะจริง (cm) — วัดด้วยตลับเมตร<input type="number" step="0.1" id="in-real"></label>
                    <label>ระยะที่คำนวณได้ (cm)<input type="number" step="0.1" id="in-computed"></label>
                </div>
                <input class="note-input" id="note" placeholder="หมายเหตุ — ไม่บังคับ">
                <button class="btn-save" onclick="savePoint()">💾 บันทึกจุดนี้</button>
            </div>

            <div class="card">
                <h2>🤖 ทดสอบอัตโนมัติ (เก็บผลซ้ำเป็นชุด)</h2>
                <p style="color:#999; font-size:0.8em; margin-bottom:10px;">ตรวจจับ → รอครบรอบ → บันทึก → วนซ้ำ แล้วปิดกล้องอัตโนมัติเมื่อครบ (ใช้ระยะจริงจากช่อง "ระยะจริง" ด้านบน)</p>
                <div class="field-grid">
                    <label>จำนวนครั้ง<input type="number" step="1" id="auto-count" value="10"></label>
                    <label>ทุกๆ (วินาที)<input type="number" step="1" id="auto-interval" value="20"></label>
                    <label>หน่วงหลังบันทึกก่อนรอบถัดไป (วินาที) — ใช้เวลานี้ขยับเป้าออกแล้ววางกลับที่เดิม<input type="number" step="1" id="auto-postdelay" value="20"></label>
                </div>
                <button class="btn-detect" id="auto-btn" onclick="startAutoBatch()">▶️ เริ่มทดสอบอัตโนมัติ</button>
                <button style="width:100%; background:#ff4757; color:white; margin-top:8px;" id="auto-stop-btn" onclick="stopAutoBatch()" disabled>⏹️ หยุดการทดลอง</button>
                <div class="readout-stat" style="margin-top:10px;"><span>สถานะ</span><strong id="auto-status">ยังไม่เริ่ม</strong></div>
            </div>

            <div class="card">
                <h2>📋 ผลการทดลอง (<span id="row-count">0</span> ครั้ง) <a id="csv-link" href="/api/disttest/export.csv" style="font-size:0.7em; font-weight:400;">⬇ CSV</a></h2>
                <div id="stats-area"></div>
                <div style="overflow-x:auto; margin-top:10px;">
                <table>
                    <thead><tr><th>ระยะจริง</th><th>คำนวณได้</th><th>ผลต่าง(cm)</th><th>%คลาด</th><th>หมายเหตุ</th><th></th></tr></thead>
                    <tbody id="records-body"></tbody>
                </table>
                </div>
            </div>
        </div>
    </div>
</div>

<script>
// กล้องสด — เปิด/ปิดด้วยมือแยกจากปุ่มตรวจจับ (กลับไปใช้แบบเดิมตามที่ขอ 2026-09-17) ปิดโดยดีฟอลต์ประหยัดพลังงาน
// เปิด/ปิดคนละเรื่องกับความเป็นอิสระของแต่ละ trial: ปุ่ม "ตรวจจับ" ดึงภาพสดใหม่จากกล้องทุกครั้งเสมอไม่ว่าพรีวิวจะเปิดหรือปิดอยู่ก็ตาม
let camOn = false;
function openCam() {
    if (camOn) return;
    camOn = true;
    document.getElementById('cam-img').src = '/video_feed';
    document.getElementById('cam-img').style.display = 'block';
    document.getElementById('cam-placeholder').style.display = 'none';
    document.getElementById('cam-toggle-btn').textContent = '⏹️ ปิดกล้อง';
}
function closeCam() {
    camOn = false;
    document.getElementById('cam-img').src = '';  // เคลียร์ src เพื่อตัดการเชื่อมต่อสตรีม MJPEG จริงๆ
    document.getElementById('cam-img').style.display = 'none';
    document.getElementById('cam-placeholder').style.display = 'flex';
    document.getElementById('cam-toggle-btn').textContent = '📷 เปิดกล้อง';
}
function toggleCam() {
    if (camOn) closeCam(); else openCam();
}

function detectNow() {
    document.getElementById('c-found').textContent = '⏳ กำลังสั่งยืน+ตรวจจับ...';
    fetch('/api/disttest/detect').then(r => r.json()).then(d => {
        document.getElementById('c-found').textContent = d.found ? '✅ พบเป้า' : '❌ ไม่พบ';
        if (d.computed_distance_cm != null) document.getElementById('in-computed').value = d.computed_distance_cm;
    });
}

function sleep(ms) { return new Promise(resolve => setTimeout(resolve, ms)); }

let autoRunning = false;
let stopRequested = false;

function stopAutoBatch() {
    stopRequested = true;
    document.getElementById('auto-status').textContent = '⏹️ กำลังหยุด...';
}

async function startAutoBatch() {
    if (autoRunning) return;
    const real = parseFloat(document.getElementById('in-real').value);
    const count = parseInt(document.getElementById('auto-count').value) || 10;
    const intervalSec = parseInt(document.getElementById('auto-interval').value) || 10;
    const postDelaySec = parseInt(document.getElementById('auto-postdelay').value) || 0;
    if (isNaN(real)) { alert('กรอก "ระยะจริง (cm)" ก่อนเริ่มทดสอบอัตโนมัติ'); return; }

    autoRunning = true;
    stopRequested = false;
    document.getElementById('auto-btn').disabled = true;
    document.getElementById('auto-btn').textContent = '⏳ กำลังทดสอบ...';
    document.getElementById('auto-stop-btn').disabled = false;
    openCam();

    let stoppedEarly = false;
    for (let i = 1; i <= count; i++) {
        if (stopRequested) { stoppedEarly = true; break; }

        document.getElementById('auto-status').textContent = `ครั้งที่ ${i}/${count} — กำลังตรวจจับ...`;
        const cycleStart = Date.now();
        let computed = null;
        try {
            const d = await fetch('/api/disttest/detect').then(r => r.json());
            if (d.found) {
                computed = d.computed_distance_cm;
                document.getElementById('in-computed').value = computed;
                document.getElementById('c-found').textContent = '✅ พบเป้า';
            } else {
                document.getElementById('c-found').textContent = '❌ ไม่พบเป้า — ข้ามรอบนี้';
            }
        } catch (e) {
            document.getElementById('c-found').textContent = '❌ ตรวจจับล้มเหลว — ข้ามรอบนี้';
        }

        if (stopRequested) { stoppedEarly = true; break; }

        // รอจนครบรอบ (นับรวมเวลาตรวจจับที่ใช้ไปแล้วด้วย) ก่อนบันทึก
        const elapsed = Date.now() - cycleStart;
        const remaining = Math.max(0, intervalSec * 1000 - elapsed);
        for (let ms = remaining; ms > 0 && !stopRequested; ms -= 500) {
            document.getElementById('auto-status').textContent = `ครั้งที่ ${i}/${count} — รออีก ${(ms/1000).toFixed(0)} วิ ก่อนบันทึก...`;
            await sleep(Math.min(500, ms));
        }
        if (stopRequested) { stoppedEarly = true; break; }

        if (computed != null) {
            await fetch('/api/disttest/save', {
                method: 'POST', headers: {'Content-Type':'application/json'},
                body: JSON.stringify({real_distance_cm: real, computed_distance_cm: computed, note: `อัตโนมัติ ครั้งที่ ${i}/${count}`})
            });
            loadRecords();
        }

        // หน่วงหลังบันทึกก่อนรอบถัดไป — ให้เวลาเอาเป้าออกแล้ววางกลับที่เดิม (ข้ามได้ถ้าเป็นรอบสุดท้าย)
        if (i < count && postDelaySec > 0) {
            for (let s = postDelaySec; s > 0 && !stopRequested; s--) {
                document.getElementById('auto-status').textContent = `บันทึกครั้งที่ ${i}/${count} แล้ว — ขยับเป้าออก/วางกลับที่เดิมได้ตอนนี้ (เหลือ ${s} วิ)`;
                await sleep(1000);
            }
            if (stopRequested) { stoppedEarly = true; break; }
        }
    }

    document.getElementById('auto-status').textContent = stoppedEarly
        ? '⏹️ หยุดการทดลองแล้ว (ปิดกล้องอัตโนมัติ)'
        : `✅ เสร็จแล้ว — ปิดกล้องอัตโนมัติ`;
    document.getElementById('auto-btn').disabled = false;
    document.getElementById('auto-btn').textContent = '▶️ เริ่มทดสอบอัตโนมัติ';
    document.getElementById('auto-stop-btn').disabled = true;
    autoRunning = false;
    stopRequested = false;
    closeCam();
}

function savePoint() {
    const real = parseFloat(document.getElementById('in-real').value);
    const computed = parseFloat(document.getElementById('in-computed').value);
    if (isNaN(real) || isNaN(computed)) { alert('กรอกระยะจริงและระยะที่คำนวณได้ให้ครบก่อนบันทึก'); return; }
    fetch('/api/disttest/save', {
        method: 'POST', headers: {'Content-Type':'application/json'},
        body: JSON.stringify({real_distance_cm: real, computed_distance_cm: computed, note: document.getElementById('note').value})
    }).then(r => r.json()).then(() => { document.getElementById('note').value=''; loadRecords(); });
}

function deleteRecord(i) {
    fetch('/api/disttest/delete', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({index:i})})
        .then(() => loadRecords());
}

function loadRecords() {
    fetch('/api/disttest/list').then(r => r.json()).then(rows => {
        document.getElementById('row-count').textContent = rows.length;
        const body = document.getElementById('records-body');
        body.innerHTML = '';
        rows.forEach((row, i) => {
            const tr = document.createElement('tr');
            tr.innerHTML = `<td>${row.real_distance_cm}</td><td>${row.computed_distance_cm}</td>
                <td>${row.error_cm > 0 ? '+' : ''}${row.error_cm}</td>
                <td>${row.error_pct > 0 ? '+' : ''}${row.error_pct}%</td>
                <td>${row.note||''}</td>
                <td><button class="btn-del" onclick="deleteRecord(${i})">ลบ</button></td>`;
            body.appendChild(tr);
        });
    });
    // สถิติแยกตามระยะจริง (ē_D, S.D., Accuracy) คำนวณฝั่ง server ตามสมการ 3.15/3.17-3.19 ในเล่ม
    fetch('/api/disttest/stats').then(r => r.json()).then(groups => {
        let html = '';
        groups.forEach(g => {
            html += `<div class="stats-box"><b>ระยะจริง ${g.real_distance_cm}cm</b> (n=${g.n}): เฉลี่ยคำนวณได้ ${g.mean_computed_cm}cm, ē_D=${g.mean_eD_pct}%, S.D.(e_D)=${g.sd_eD_pct}%, Accuracy=${g.accuracy_pct}%</div>`;
        });
        document.getElementById('stats-area').innerHTML = html;
    });
}

window.addEventListener('load', loadRecords);
</script>
</body>
</html>
"""

AIMING_TEST_HTML = """
<!DOCTYPE html>
<html lang="th">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Aiming Test</title>
<style>""" + TEST_PAGE_STYLE + """</style>
</head>
<body>
<div class="page">
    <h1>🎯 ทดสอบความแม่นยำการชี้เป้า</h1>
    <div class="nav-links"><a href="/">← หน้าควบคุมหลัก</a> | <a href="/distance-test">📏 ทดสอบระยะทาง →</a></div>

    <div class="dashboard">
        <div class="col">
            <div class="card">
                <h2>📷 กล้องสด</h2>
                <img class="camera-feed" id="cam-img" alt="camera feed (ปิดอยู่ — กดเปิดกล้องเพื่อดู)" style="display:none;">
                <div id="cam-placeholder" style="background:#111; border-radius:10px; aspect-ratio:4/3; display:flex; align-items:center; justify-content:center; color:#666; font-size:0.85em;">กล้องปิดอยู่ (ประหยัดพลังงาน)</div>
                <button style="width:100%; background:#57606f; color:white; margin-top:10px;" onclick="toggleCam()" id="cam-toggle-btn">📷 เปิดกล้อง</button>
                <label style="display:block; margin-top:10px; font-size:0.8em; color:#666;">ระยะจริง (cm) — วัดด้วยตลับเมตร
                    <input type="number" step="0.1" id="in-real" class="note-input" style="margin-top:4px;">
                </label>
                <button class="btn-detect" onclick="detectNow()">🔍 ตรวจจับ + ชี้เป้า + เปิดเลเซอร์</button>
                <button style="width:100%; background:#2f3542; color:white; margin-top:8px;" onclick="goHome()">🏠 กลับตำแหน่ง Home (ปิดเลเซอร์)</button>
                <p style="color:#999; font-size:0.75em; margin-top:6px;">ทุก trial: กลับ Home (Pan 1500 / Tilt 800) → ยืน รอ 2 วิ → ตรวจจับ → เข้าเป้าจากทิศเดียวกัน → เปิดเลเซอร์ / กดบันทึกแล้วจะปิดเลเซอร์และกลับ Home เอง</p>
                <div class="readout-stat"><span>พบเป้า</span><strong id="c-found">-</strong></div>
                <div class="readout-stat"><span>ระยะที่คำนวณได้</span><strong id="c-distance">-</strong></div>
            </div>
            <div class="card">
                <h2>🔴 เลเซอร์ (GPIO 7)</h2>
                <div style="display:flex; gap:10px;">
                    <button style="flex:1; background:#ff4757; color:white;" onclick="laserOn()">🔴 เปิดเลเซอร์</button>
                    <button style="flex:1; background:#2f3542; color:white;" onclick="laserOff()">⚫ ปิดเลเซอร์</button>
                </div>
                <div id="laser-status" style="text-align:center; padding:10px; margin-top:10px; border-radius:8px; font-weight:600; background:#f5f5f5; color:#777;">⚫ เลเซอร์: ปิด</div>
            </div>
        </div>

        <div class="col">
            <div class="card">
                <h2>📐 บันทึกผล (อ่านจากแผ่นเป้า)</h2>
                <p style="color:#999; font-size:0.8em; margin-bottom:10px;">ถ่ายรูปแผ่นเป้าจากด้านหน้า อ่าน Δx (ขวา +), Δy (บน +) จากช่องตาราง 5mm (ความละเอียด ±0.25cm) แล้วกรอกด้านล่าง</p>
                <div class="field-grid">
                    <label>Δx (cm, ขวา +)<input type="number" step="0.25" id="in-dx"></label>
                    <label>Δy (cm, บน +)<input type="number" step="0.25" id="in-dy"></label>
                </div>
                <input class="note-input" id="note" placeholder="หมายเหตุ — ไม่บังคับ">
                <button class="btn-save" onclick="saveOk()">💾 บันทึกผล</button>
                <button style="width:100%; background:#ffa502; color:white; margin-top:8px;" onclick="saveOffTarget()">📍 บันทึก: เลเซอร์นอกแผ่นเป้า</button>
                <button style="width:100%; background:#747d8c; color:white; margin-top:8px;" onclick="saveNotDetected()">❌ บันทึก: ระบบตรวจไม่พบ</button>
                <p style="color:#999; font-size:0.75em; margin-top:8px;">⚠️ ตามเล่มบทที่ 3 ห้ามข้าม/ลบทิ้ง trial ที่นอกแผ่นเป้าหรือตรวจไม่พบ ต้องบันทึกไว้ตามจริง</p>
            </div>

            <div class="card">
                <h2>📋 ผลการทดลอง (<span id="row-count">0</span> trial) <a id="csv-link" href="/api/aimingtest/export.csv" style="font-size:0.7em; font-weight:400;">⬇ CSV</a></h2>
                <div id="stats-area"></div>
                <div style="overflow-x:auto; margin-top:10px;">
                <table>
                    <thead><tr><th>ระยะ</th><th>สถานะ</th><th>Δx</th><th>Δy</th><th>E(cm)</th><th>เข้าเป้า</th><th>หมายเหตุ</th><th></th></tr></thead>
                    <tbody id="records-body"></tbody>
                </table>
                </div>
            </div>
        </div>
    </div>
</div>

<script>
// เปิด/ปิดกล้องสด — ปิดโดยดีฟอลต์ประหยัดพลังงาน (ไม่ดึงภาพจากกล้องต่อเนื่องถ้าไม่ได้เปิดดู)
let camOn = false;
function toggleCam() {
    camOn = !camOn;
    const img = document.getElementById('cam-img');
    const placeholder = document.getElementById('cam-placeholder');
    const btn = document.getElementById('cam-toggle-btn');
    if (camOn) {
        img.src = '/video_feed';
        img.style.display = 'block';
        placeholder.style.display = 'none';
        btn.textContent = '⏹️ ปิดกล้อง';
    } else {
        img.src = '';
        img.style.display = 'none';
        placeholder.style.display = 'flex';
        btn.textContent = '📷 เปิดกล้อง';
    }
}

function showLaserOff() {
    const el = document.getElementById('laser-status');
    el.textContent = '⚫ เลเซอร์: ปิด'; el.style.background = '#f5f5f5'; el.style.color = '#777';
}

function detectNow() {
    showLaserOff();
    document.getElementById('c-found').textContent = '⏳ กลับ Home → ยืน รอนิ่ง → ตรวจจับ → ชี้เป้า...';
    fetch('/api/aimingtest/detect').then(r => r.json()).then(d => {
        document.getElementById('c-found').textContent = d.found ? `✅ พบ (Pan ${d.pan_us} / Tilt ${d.tilt_us} µs, เปิดเลเซอร์แล้ว)` : '❌ ไม่พบ (อยู่ที่ Home)';
        if (!d.found) return;
        document.getElementById('c-distance').textContent = d.distance_cm + ' ซม.';
        const el = document.getElementById('laser-status');
        el.textContent = '🔴 เลเซอร์: เปิด'; el.style.background = '#ffebee'; el.style.color = '#c41c3b';
    });
}

function goHome() {
    fetch('/api/aimingtest/home', {method:'POST'}).then(r => r.json()).then(d => {
        if (d.success) showLaserOff();
    });
    document.getElementById('c-found').textContent = '🏠 อยู่ที่ Home';
}

function currentReal() {
    const v = parseFloat(document.getElementById('in-real').value);
    if (isNaN(v)) { alert('กรอก "ระยะจริง (cm)" ก่อน'); return null; }
    return v;
}

function doSave(payload) {
    fetch('/api/aimingtest/save', {
        method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify(payload)
    }).then(r => r.json()).then(() => {
        document.getElementById('note').value = '';
        document.getElementById('in-dx').value = '';
        document.getElementById('in-dy').value = '';
        showLaserOff();
        document.getElementById('c-found').textContent = '🏠 บันทึกแล้ว กลับ Home — พร้อม trial ถัดไป';
        loadRecords();
    });
}

function saveOk() {
    const real = currentReal();
    if (real == null) return;
    const dx = parseFloat(document.getElementById('in-dx').value);
    const dy = parseFloat(document.getElementById('in-dy').value);
    if (isNaN(dx) || isNaN(dy)) { alert('กรอก Δx และ Δy ให้ครบก่อนบันทึก'); return; }
    doSave({distance_cm: real, status: 'ok', dx_cm: dx, dy_cm: dy, note: document.getElementById('note').value});
}

function saveOffTarget() {
    const real = currentReal();
    if (real == null) return;
    doSave({distance_cm: real, status: 'off_target', note: document.getElementById('note').value});
}

function saveNotDetected() {
    const real = currentReal();
    if (real == null) return;
    doSave({distance_cm: real, status: 'not_detected', note: document.getElementById('note').value});
}

function deleteRecord(i) {
    fetch('/api/aimingtest/delete', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({index:i})})
        .then(() => loadRecords());
}

const STATUS_LABEL = {ok: '✅ ปกติ', off_target: '📍 นอกแผ่นเป้า', not_detected: '❌ ตรวจไม่พบ'};

function loadRecords() {
    fetch('/api/aimingtest/list').then(r => r.json()).then(rows => {
        document.getElementById('row-count').textContent = rows.length;
        const body = document.getElementById('records-body');
        body.innerHTML = '';
        rows.forEach((row, i) => {
            const tr = document.createElement('tr');
            tr.innerHTML = `<td>${row.distance_cm ?? '-'}</td>
                <td>${STATUS_LABEL[row.status] || row.status}</td>
                <td>${row.dx_cm ?? '-'}</td>
                <td>${row.dy_cm ?? '-'}</td>
                <td>${row.E_cm ?? '-'}</td>
                <td>${row.status === 'ok' ? (row.hit ? '✅' : '❌') : '-'}</td>
                <td>${row.note||''}</td>
                <td><button class="btn-del" onclick="deleteRecord(${i})">ลบ</button></td>`;
            body.appendChild(tr);
        });
    });
    fetch('/api/aimingtest/stats').then(r => r.json()).then(s => {
        if (!s.n) { document.getElementById('stats-area').innerHTML = ''; return; }
        document.getElementById('stats-area').innerHTML = `<div class="stats-box">
            <div><b>n</b> = ${s.n} (ตรวจพบ ${s.n_detect}, มีค่า Δx/Δy ${s.n_valid})</div>
            <div><b>Hit rate</b> = ${s.hit_rate_pct}% &nbsp; <b>Detection rate</b> = ${s.detection_rate_pct}%</div>
            <div>%E≤1cm = ${s.pct_e_le_1cm}%, %E≤2cm = ${s.pct_e_le_2cm}%, %E≤4cm = ${s.pct_e_le_4cm}%</div>
            <div>Ē = ${s.mean_E_cm ?? '-'} cm, S.D. = ${s.sd_E_cm ?? '-'} cm, E สูงสุด = ${s.max_E_cm ?? '-'} cm</div>
            <div>Δx̄ = ${s.dx_bar_cm ?? '-'} cm, Δȳ = ${s.dy_bar_cm ?? '-'} cm</div>
            <div><b>AP</b> = ${s.AP_cm ?? '-'} cm &nbsp; <b>RP</b> = ${s.RP_cm ?? '-'} cm</div>
        </div>`;
    });
}

function laserOn() {
    fetch('/api/laser/on', {method:'POST'}).then(r => r.json()).then(d => {
        if (d.success) {
            const el = document.getElementById('laser-status');
            el.textContent = '🔴 เลเซอร์: เปิด'; el.style.background = '#ffebee'; el.style.color = '#c41c3b';
        }
    });
}
function laserOff() {
    fetch('/api/laser/off', {method:'POST'}).then(r => r.json()).then(d => {
        if (d.success) {
            const el = document.getElementById('laser-status');
            el.textContent = '⚫ เลเซอร์: ปิด'; el.style.background = '#f5f5f5'; el.style.color = '#777';
        }
    });
}

window.addEventListener('load', loadRecords);
</script>
</body>
</html>
"""

@app.route('/distance-test')
def distance_test_page():
    """หน้าทดสอบความแม่นยำระยะทาง — เก็บ (ระยะจริง, ระยะที่คำนวณได้) หลายครั้งต่อระยะ พร้อมสถิติสรุปอัตโนมัติ"""
    return render_template_string(DISTANCE_TEST_HTML)

@app.route('/aiming-test')
def aiming_test_page():
    """หน้าทดสอบความแม่นยำการชี้เป้า (3.4.4-3.4.5 ในเล่ม) — อ่าน Δx/Δy จากแผ่นเป้าจริง แทนการปรับมุมทีละองศา
    (วิธีเดิมแบบองศายังเก็บไว้ที่ /api/aimingtest/nudge แต่ไม่มีปุ่มในหน้าแล้ว) พร้อมสถิติ Hit rate/AP/RP ตามเล่ม"""
    return render_template_string(AIMING_TEST_HTML)

@app.route('/')
def index():
    """หน้าหลัก"""
    # ใช้ request.host (IP/hostname ที่เบราว์เซอร์ใช้เชื่อมต่อเข้ามาจริง) แทนการเปิด socket แกล้งต่อ 8.8.8.8
    # เพื่อหา IP ตัวเอง — วิธีเดิมพัง (OSError: Network is unreachable) ตอนไม่มีเส้นทางออกอินเทอร์เน็ต
    # (เช่น ถอดสาย LAN เหลือแต่ WiFi AP ของหุ่นยนต์เองที่ไม่มีทางออกเน็ต) พบจริง 2026-10-21
    ip_address = request.host.split(':')[0]

    return render_template_string(
        HTML_TEMPLATE,
        min_pos=MIN_POS,
        max_pos=MAX_POS,
        tilt_min_pos=TILT_MIN_POS,
        tilt_max_pos=TILT_MAX_POS,
        ip_address=ip_address,
        flask_port=FLASK_PORT
    )

@app.route('/api/servo/set', methods=['POST'])
def set_servo_api():
    """API สำหรับสั่งเซอร์โว — id=10 (Pan) และ id=11 (Tilt) มีขอบเขตปลอดภัยคนละช่วงกัน"""
    data = request.json
    servo_id = data.get('id')
    position = data.get('position')

    if servo_id not in (10, 11):
        return jsonify({'success': False, 'error': f'id={servo_id} ไม่รองรับ'})

    lo, hi = (MIN_POS, MAX_POS) if servo_id == PAN_ID else (TILT_MIN_POS, TILT_MAX_POS)
    position = max(lo, min(hi, int(position)))  # บังคับ clamp ในขอบเขตที่ปลอดภัยของแกนนั้นๆ เสมอ

    success = set_servo(servo_id, position)
    if success:
        angle = position_to_angle(position)
        key = 'pan' if servo_id == 10 else 'tilt'
        servo_state[key]['position'] = position
        servo_state[key]['angle'] = angle
        return jsonify({'success': True, 'position': position, 'angle': angle})

    return jsonify({'success': False})

@app.route('/api/action', methods=['POST'])
def action_api():
    """API สั่งเล่นท่าสำเร็จรูปของหุ่นยนต์ (เช่น ยืน/นั่ง/หมอบ) — ใช้ตอนเซอร์โว pan-tilt ติดขีดจำกัด
    ต้องปรับท่าทางหุ่นยนต์แทนตามที่ระบุไว้ในบทที่ 3 ของรายงาน"""
    data = request.json
    name = data.get('name')
    if name not in ('stand.d6ac', 'sit.d6ac', 'lie_down.d6ac'):
        return jsonify({'success': False, 'error': f'ท่า {name} ไม่รองรับ'})
    success = run_action_group(name)
    return jsonify({'success': success})

@app.route('/api/autotrack/start', methods=['POST'])
def autotrack_start_api():
    """เริ่ม Auto-Track — ต้องสั่งหุ่นยนต์ให้ยืนก่อนเริ่มเสมอ (จุดเริ่มต้นของ state machine)"""
    global autotrack_thread
    if autotrack_running.is_set():
        return jsonify({'success': False, 'error': 'Auto-Track ทำงานอยู่แล้ว'})
    run_action_group('stand.d6ac')
    time.sleep(POSE_SETTLE_SEC)
    autotrack_running.set()
    autotrack_status.update({'active': True, 'message': 'เริ่มทำงาน'})
    autotrack_thread = threading.Thread(target=autotrack_loop, daemon=True)
    autotrack_thread.start()
    return jsonify({'success': True})

@app.route('/api/autotrack/stop', methods=['POST'])
def autotrack_stop_api():
    """หยุด Auto-Track"""
    autotrack_running.clear()
    return jsonify({'success': True})

@app.route('/api/autotrack/status', methods=['GET'])
def autotrack_status_api():
    """สถานะปัจจุบันของ Auto-Track (โพลจากหน้าเว็บทุก 1 วินาที)"""
    return jsonify(autotrack_status)

@app.route('/api/laser/on', methods=['POST'])
def laser_on_api():
    """API เปิดเลเซอร์"""
    success = laser_on()
    return jsonify({'success': success})

@app.route('/api/laser/off', methods=['POST'])
def laser_off_api():
    """API ปิดเลเซอร์"""
    success = laser_off()
    return jsonify({'success': success})

@app.route('/api/status', methods=['GET'])
def status_api():
    """API สถานะระบบ (รวมแบตเตอรี่)"""
    return jsonify({**servo_state, 'battery': battery_state})

# ========== Main ==========
import walk_test  # noqa: E402  หน้า /walk-test — ถอดออกได้โดยลบ 2 บรรทัดนี้
walk_test.register(app, globals())

if __name__ == '__main__':
    print("\n" + "="*60)
    print("🚀 Pan-Tilt Laser Web Controller")
    print("="*60)
    
    # เชื่อมต่อ ROS + ตั้ง watchdog คอย reconnect อัตโนมัติถ้าหลุด/ยังไม่พร้อมตอนเริ่มแอป
    if not init_ros():
        print("⚠️ ข้อแนะนำ: ตรวจสอบว่า ROS Master รันอยู่ (จะลองเชื่อมต่อใหม่อัตโนมัติเรื่อยๆ ในพื้นหลัง)")
    threading.Thread(target=ros_watchdog, daemon=True).start()

    # เชื่อมต่อเลเซอร์
    init_laser()
    
    print(f"\n📍 Web App: http://<Your-Pi-IP>:{FLASK_PORT}")
    print("="*60 + "\n")
    
    try:
        # threaded=True สำคัญมาก: /video_feed เป็นสตรีมค้างไม่มีวันจบ ถ้าไม่ threaded เซิร์ฟเวอร์จะไปติดอยู่กับ
        # connection นั้นจนไม่ว่างตอบ request อื่น (สั่งเซอร์โว/ตรวจจับ) — ตรงกับอาการ "กล้องค้าง" ที่เจอระหว่างเก็บ calibration (2026-09-17)
        app.run(host=FLASK_HOST, port=FLASK_PORT, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n⏹️  ปิด Web App")
    finally:
        if laser:
            laser.close()
        if ros:
            ros.terminate()
