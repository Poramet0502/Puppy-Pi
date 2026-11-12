"""
สคริปต์ทดสอบกล้อง: ดึงภาพจาก web_video_server ของ PuppyPi แล้วตรวจจับวัตถุสีแดงด้วย HSV
รันบน host (ไม่แตะโค้ดเดิมของโรงงานเลย) — ต้องมี ROS ของหุ่นยนต์ทำงานอยู่ (web_video_server พอร์ต 8080)

วิธีใช้:
    python3 red_detect_test.py            # จับภาพ 1 ครั้ง บันทึกผล
    python3 red_detect_test.py --loop 10  # จับภาพซ้ำ 10 ครั้ง ห่างกันครั้งละ 1 วินาที
"""

import argparse
import sys
import time

import cv2
import numpy as np
import requests

SNAPSHOT_URL = "http://127.0.0.1:8080/snapshot?topic=/usb_cam/image_raw"
OUTPUT_DIR = "/home/pi/Desktop/Poramet_Project/vision/output"

MIN_CONTOUR_AREA = 20   # px^2 — ลดจาก 40 เมื่อ 2026-09-11: ที่ระยะไกล (>1.5m) เป้ากว้างแค่ ~8px (พื้นที่วงกลม ~50px^2)
                        # ค่าเดิม 40 อยู่ใกล้ขอบเขตของวัตถุจริงเกินไป พอรวมกับ noise ของภาพแล้วทำให้ตกเกณฑ์สลับไปมา (จับได้/หายเป็นระยะ)
MIN_CIRCULARITY = 0.55  # 1.0 = วงกลมสมบูรณ์แบบ ใช้กรองรูปทรงเบี้ยว (มือคน, ของเหลี่ยม) ทิ้ง

# ขนาดจริงของจุดแดงที่จะติดไว้กึ่งกลางเป้า (W ในสูตร D = f*W/P) — แก้ไขแล้ว 2026-09-09: เส้นผ่านศูนย์กลาง 3cm (เดิมเข้าใจผิดว่า 2.5cm)
MARKER_WIDTH_CM = 8.0  # เปลี่ยนจาก 3.0cm → 8.0cm (2026-10-20) ตามแผ่นเป้า A4 ใหม่ (จุดแดง Ø8cm + ช่องตาราง 5mm อ่าน Δx/Δy) — ดู PUPPY_PI_THESIS_SYNC.md
# ความยาวโฟกัสกล้อง (พิกเซล) — recalibrate แล้ว 2026-09-11 ด้วยระยะไกลขึ้น (P ใหญ่กว่า วัดแม่นกว่า):
#   150cm จริง -> วัดได้ 17px นิ่ง 8/8 เฟรม -> f = 150*17/3 = 850
#   200cm จริง -> วัดได้ 12px นิ่ง 7/8 เฟรม -> f = 200*12/3 = 800
#   เฉลี่ย 2 จุด = 825 ตรวจย้อนกลับแล้วคลาดเคลื่อน <3% ทั้งคู่ — ทิ้งค่าเดิม 429 (ได้จากวัดที่ 100cm ตอนจุดเล็กแค่ 9-15px ผันผวนสูง ไม่แม่น)
# Recalibrate อีกครั้ง 2026-09-16 จากข้อมูล Calibration Tool 9 จุด (100/150/200cm) พบว่าค่าเดิม 825 ประเมินระยะทางเกินจริง 1.5-2 เท่าสม่ำเสมอ
# แปลว่าจุดเป้าที่ใช้ทดสอบตอนนี้เล็กกว่าตอน calibrate ค่า 825 (ที่ 150cm/200cm รอบก่อน) — ย้อนคำนวณความกว้างพิกเซลจริงจาก 9 จุด (P=15,16,15 ที่ 100cm; P=8,9,10 ที่ 150cm; P=7,6,6 ที่ 200cm)
# fit ได้ f เฉลี่ย ~461 (ทุกจุด) แต่ค่าที่ 100cm น่าเชื่อถือสุด (P มากสุด=noise น้อยสุด) เฉลี่ยกลุ่มนั้นได้ f=511 — ใช้ค่านี้
# Recalibrate อีกครั้ง 2026-10-21 หลังเปลี่ยนเป้าใหม่เป็น 8cm (วัดจริงยืนยันแล้วว่า 8.0cm ถูกต้อง ไม่ใช่ปัญหาขนาดเป้า) จากผลทดสอบ /distance-test 30 ครั้ง
# ย้อนคำนวณ f=D_จริง*P/W จาก 3 ระยะ: 100cm->637.5, 150cm->610.9, 200cm->600.0 เฉลี่ย=616 (ค่าเดิม 511 จากเป้า 3cm รอบก่อนใช้ไม่ได้แล้ว)
FOCAL_LENGTH_PX = 616

# ช่วงสีแดงใน HSV (OpenCV: H 0-180) แดงอยู่คร่อมจุด 0 องศา จึงต้องใช้ 2 ช่วง
# บีบช่วงให้แคบกว่ารอบทดสอบก่อนหน้า (S ขั้นต่ำสูง) เพื่อตัดสีผิวคน/สีส้มซึ่งมัก S ต่ำกว่าสติกเกอร์สีแดงสดออก
# ลด V ขั้นต่ำจาก 90 -> 40 (พบ 2026-09-09: จุดเป้าหมายจริงวัดได้ V=54 ในแสงห้องปกติ ตกเกณฑ์เดิม)
LOWER_RED_1 = np.array([0, 150, 40])
UPPER_RED_1 = np.array([8, 255, 255])
LOWER_RED_2 = np.array([172, 150, 40])
UPPER_RED_2 = np.array([180, 255, 255])


def grab_frame():
    resp = requests.get(SNAPSHOT_URL, timeout=5)
    resp.raise_for_status()
    img_array = np.frombuffer(resp.content, dtype=np.uint8)
    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("ถอดรหัสภาพจากกล้องไม่สำเร็จ")
    return frame


def _circularity(contour):
    """1.0 = วงกลมสมบูรณ์แบบ, ยิ่งเบี้ยว (มือคน, ของเหลี่ยม) ยิ่งต่ำ"""
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return 0.0
    return float(4 * np.pi * area / (perimeter ** 2))


def detect_red(frame):
    """ล็อกเป้าเดียว: เลือกก้อนสีแดงที่ใหญ่ที่สุดในบรรดาก้อนที่กลมพอ (ตัด noise รูปทรงเบี้ยวเช่นมือคนทิ้ง)"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1)
    mask2 = cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
    mask = cv2.bitwise_or(mask1, mask2)

    # kernel 5x5 เดิมกัด (erode) เข้าไป 2px รอบตัว — เป้าที่ระยะไกลกว้างแค่ ~8px โดนกัดจนหายไปเกือบหมด
    # ทำให้ผลลัพธ์สั่นไม่นิ่ง (บางเฟรมรอดบางเฟรมไม่รอด) ลดเหลือ 3x3 (กัดแค่ 1px) ให้เป้าเล็กยังรอดสม่ำเสมอ
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_CONTOUR_AREA:
            continue
        circ = _circularity(c)
        if circ < MIN_CIRCULARITY:
            continue
        candidates.append((c, area, circ))

    result = frame.copy()
    detections = []

    if candidates:
        c, area, circ = max(candidates, key=lambda t: t[1])
        x, y, w, h = cv2.boundingRect(c)
        cx, cy = x + w // 2, y + h // 2

        distance_cm = None
        if FOCAL_LENGTH_PX is not None and w > 0:
            distance_cm = (FOCAL_LENGTH_PX * MARKER_WIDTH_CM) / w

        detections.append(
            {
                "bbox": (x, y, w, h),
                "centroid": (cx, cy),
                "area": area,
                "circularity": round(circ, 2),
                "distance_cm": distance_cm,
            }
        )

        cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(result, (cx, cy), 6, (0, 255, 255), -1)
        label = f"TARGET w={w}px circ={circ:.2f}"
        if distance_cm is not None:
            label += f" D={distance_cm:.0f}cm"
        cv2.putText(result, label, (x, max(0, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

    img_h, img_w = frame.shape[:2]
    cv2.drawMarker(result, (img_w // 2, img_h // 2), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)

    return result, mask, detections


def run_once(index=0):
    frame = grab_frame()
    result, mask, detections = detect_red(frame)

    cv2.imwrite(f"{OUTPUT_DIR}/red_detect_result.jpg", result)
    cv2.imwrite(f"{OUTPUT_DIR}/red_detect_mask.jpg", mask)

    if detections:
        d = detections[0]
        dist = f" distance={d['distance_cm']:.0f}cm" if d["distance_cm"] is not None else ""
        print(
            f"[{index}] ล็อกเป้าได้: centroid={d['centroid']} bbox={d['bbox']} "
            f"area={int(d['area'])}px^2 circularity={d['circularity']}{dist}"
        )
    else:
        print(f"[{index}] ไม่พบเป้าสีแดงในภาพ (กล้องทำงานปกติ ถ่ายภาพได้ {frame.shape[1]}x{frame.shape[0]})")

    return detections


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--loop", type=int, default=1, help="จำนวนครั้งที่จับภาพซ้ำ")
    parser.add_argument("--interval", type=float, default=1.0, help="วินาทีระหว่างแต่ละครั้ง")
    args = parser.parse_args()

    try:
        for i in range(args.loop):
            run_once(i)
            if i < args.loop - 1:
                time.sleep(args.interval)
    except requests.exceptions.RequestException as e:
        print(f"เชื่อมต่อกล้องไม่ได้: {e}")
        print("ตรวจสอบว่า ROS ของหุ่นยนต์ทำงานอยู่หรือไม่ (docker ps ควรเห็น container 'puppypi' สถานะ Up)")
        sys.exit(1)

    print(f"\nผลลัพธ์บันทึกไว้ที่: {OUTPUT_DIR}/red_detect_result.jpg (ภาพพร้อมกรอบตรวจจับ)")
    print(f"                    {OUTPUT_DIR}/red_detect_mask.jpg (ภาพ mask ขาวดำของสีแดง)")


if __name__ == "__main__":
    main()
