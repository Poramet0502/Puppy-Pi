# ความสอดคล้องระหว่างเล่มปริญญานิพนธ์กับโค้ด

เลขสมการและเลขหัวข้ออ้างอิงตามบทที่ 3 ของเล่ม ไฟล์ทั้งหมดอยู่ในโฟลเดอร์ `motion/` และ `vision/`

## การตรวจจับเป้า (หัวข้อ 3.2.1)

| ในเล่ม | ค่า | ในโค้ด |
|---|---|---|
| ช่วงสีแดง HSV | H 0–8 และ 172–180, S 150–255, V 40–255 | `vision/red_detect_test.py`: `LOWER_RED_1/2`, `UPPER_RED_1/2` |
| ลดสัญญาณรบกวน | kernel 3×3, Opening แล้ว Closing | `detect_red()` |
| พื้นที่ขั้นต่ำ | 20 px² | `MIN_CONTOUR_AREA` |
| ความกลม (3.1) | 4π·Area/Perimeter² ≥ 0.55 | `_circularity()`, `MIN_CIRCULARITY` |
| เลือกเป้า | ก้อนที่ใหญ่ที่สุดในบรรดาก้อนที่ผ่านเกณฑ์ | `max(candidates, key=area)` |
| P และจุดศูนย์กลางเป้า | ความกว้างกรอบสี่เหลี่ยม และกึ่งกลางกรอบ | `bbox[2]`, `centroid` |
| ภาพจากกล้อง | 640×480 พิกเซล จาก ROS `/usb_cam/image_raw` ผ่าน web_video_server พอร์ต 8080 | `grab_frame()` |

## ระยะทาง มุม และพัลส์ (หัวข้อ 3.2.2–3.2.3)

| สมการ | ในโค้ด (`motion/pan_tilt_laser_app.py`) |
|---|---|
| (3.2) D = f·W/P, f = 616, W = 8 | `compute_aim()`, `disttest_detect_api()`; `FOCAL_LENGTH_PX`, `MARKER_WIDTH_CM` |
| (3.4)–(3.5) eₓ, e_y | `compute_aim()`: `error_x`, `error_y` |
| (3.6)–(3.8) X, Y, Z | ไม่ได้คำนวณ X, Y เป็นค่าแยก ใช้ผ่านมุม (3.9)–(3.10) ซึ่งเท่ากับ arctan(X/Z) และ arctan(Y/Z); Z = D |
| (3.9)–(3.10) θ = arctan(e/f) | `compute_aim()`: `pan_angle_deg`, `tilt_cam_angle_deg` |
| (3.11)–(3.12) พารัลแลกซ์ h = 10.5 | `compute_aim()`: `parallax_deg`; `CAM_LASER_HEIGHT_OFFSET_CM` |
| (3.13)–(3.14) มุม → พัลส์, Δpan −242, Δtilt +65 | `compute_aim()`; `US_PER_DEGREE`, `PAN_DIRECTION`, `PAN_OFFSET_US`, `TILT_OFFSET_US` |
| 3.2.3.4 ช่วงเซอร์โว | `MIN_POS`/`MAX_POS` (Pan 500–2500), `TILT_MIN_POS`/`TILT_MAX_POS` (Tilt 500–1300), ท่านั่งใน `autotrack_loop()` |

## ขั้นตอนการทำงานและการทดลอง (หัวข้อ 3.3–3.4)

| ในเล่ม | ในโค้ด |
|---|---|
| 3.3 ระบบเล็งอัตโนมัติ รอบละ 0.7 วินาที | `autotrack_loop()`, `AUTOTRACK_INTERVAL` |
| สั่งยืนแล้วรอ 2 วินาทีก่อนตรวจจับ | `run_action_group('stand.d6ac')`, `POSE_SETTLE_SEC` |
| 3.4.3 การทดลองที่ 1 (`/distance-test`) | `disttest_*_api()`, `DISTANCE_TEST_HTML` |
| 3.4.4 การทดลองที่ 2 (`/aiming-test`) Home 1500/800 µs, เลยเป้า 60 µs | `aimingtest_detect_api()`, `aim_go_home()`, `AIM_HOME_*`, `AIM_APPROACH_US` |
| 3.4.5 เกณฑ์เข้าเป้า E ≤ 4 ซม. | `HIT_RADIUS_CM`, `aimingtest_save_api()` |
| 3.4.6 การทดลองที่ 3 (`/walk-test`) 8 ซม./วินาที × 6.25 วินาที | `motion/walk_test.py`: `walk_thread()`, topic `/puppy_control/velocity/autogait` |
| ร้อยละแบตเตอรี่ | `_on_battery()`: เส้นตรงจาก 6.4 V (0%) ถึง 8.4 V (100%) |

## สถิติ (หัวข้อ 3.5)

| สมการ | ในโค้ด |
|---|---|
| (3.15) e_D, (3.19) Accuracy | `distance_stats()` |
| (3.16) E, (3.20) AP, (3.21)–(3.22) RP, (3.23) Hit rate, (3.24) Detection rate | `aiming_stats()`, `aimingtest_save_api()` |
| (3.17)–(3.18) ค่าเฉลี่ย และ S.D. หาร n − 1 | `_mean()`, `_sd()` |
| (3.25) Welch's t-test | ไม่ได้อยู่ในเว็บแอป คำนวณจากไฟล์ `data/exp3_walk_test_60.csv` |

หน้า `/aiming-test` คำนวณสถิติรวมทุกระยะเข้าด้วยกัน ส่วนค่าที่รายงานแยกตามระยะในบทที่ 4 คำนวณจากไฟล์ CSV ในโฟลเดอร์ `data/` ด้วยสมการเดียวกัน
