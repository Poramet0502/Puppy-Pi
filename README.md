# PuppyPi — ระบบตรวจจับและชี้เป้าด้วยเลเซอร์บนหุ่นยนต์สุนัขสี่ขา

โค้ดประกอบงานวิจัย "การพัฒนาระบบตรวจจับและวิเคราะห์พิกัดเป้าหมายเชิงพื้นที่สำหรับหุ่นยนต์สุนัข" บนหุ่นยนต์ Hiwonder PuppyPi (Raspberry Pi 4B) ติดตั้งฐานเล็งเป้า 2 แกน (Pan-Tilt) พร้อมเลเซอร์ ตรวจจับเป้าสีแดงด้วย HSV + Contour คำนวณระยะด้วยกล้องตัวเดียว (D = f·W/P) แล้วคำนวณมุมชี้เป้าสัมบูรณ์จากภาพเดียว (กล้องยึดนิ่งกับลำตัว แบบ Eye-to-Hand)

โค้ดทำงานเป็นไคลเอนต์ของ ROS Noetic ของผู้ผลิต ผ่าน rosbridge (port 9090) และดึงภาพจาก web_video_server (port 8080) **ไม่ได้แก้โค้ดของผู้ผลิต**

## ระบบที่ใช้จริง

| ไฟล์ | หน้าที่ |
|---|---|
| `motion/pan_tilt_laser_app.py` | เว็บแอปหลัก (Flask, port 8091): ควบคุม Pan/Tilt/เลเซอร์, Auto-Track, หน้า `/distance-test` (ทดลองวัดระยะ), `/aiming-test` (ทดลองชี้เป้า), สูตรระยะ/มุม/พัลส์ (`compute_aim()`), สถิติ, export CSV |
| `motion/walk_test.py` | หน้า `/walk-test` ทดลองผลของฐานเล็งเป้าต่อการเดิน |
| `vision/red_detect_test.py` | ดึงภาพกล้อง (`grab_frame()`) และตรวจจับเป้าสีแดง (`detect_red()`) |
| `deploy/pan_tilt_laser_app.service` | systemd unit ให้แอปรันอัตโนมัติตอนเปิดเครื่อง |

ค่าที่ใช้เก็บผลทดลอง: f = 616 px, W = 8.0 cm, ความสูงเลเซอร์เหนือกล้อง h = 10.5 cm, Δpan = −242 µs, Δtilt = +65 µs, Home = Pan 1500 / Tilt 800 µs, เข้าเป้าจากทิศเดียวผ่านจุดแวะ +60 µs, เกณฑ์เข้าเป้า E ≤ 4.0 cm

## เครื่องมือช่วงพัฒนา (ไม่ได้ใช้ในระบบจริง)

| ไฟล์ | หน้าที่ |
|---|---|
| `motion/pwm_servo_control.py` | สั่งเซอร์โว id 9–11 จาก command line |
| `motion/laser_control.py` | เปิด/ปิด/กะพริบเลเซอร์ GPIO 7 |
| `motion/run_action.py` | สั่งท่าสำเร็จรูป (action group) |
| `motion/walk_and_pee.py` | สคริปต์สั่งเดินรุ่นแรก |
| `vision/web_stream.py` | เว็บดูภาพกล้องพร้อมกรอบตรวจจับ (port 8090) |
| `tracking/tilt_track_test.py` | ทดสอบ IBVS แกนเดียว — ยกเลิกแล้ว เพราะกล้องยึดนิ่งกับลำตัว |

## สภาพแวดล้อม

Raspberry Pi 4B · Debian 12 · Python 3.11.2 · Flask 2.2.2 · roslibpy 2.1.0 · OpenCV 4.9.0 · NumPy 1.24.2 · requests 2.28.1 · gpiozero 2.0.1 · ROS Noetic ของผู้ผลิตใน Docker

## วิธีรัน

```bash
cd motion
python3 pan_tilt_laser_app.py
# เปิด http://<IP ของ Pi>:8091/
```

ติดตั้งเป็น service: คัดลอก `deploy/pan_tilt_laser_app.service` ไป `/etc/systemd/system/` แล้ว `sudo systemctl enable --now pan_tilt_laser_app` (path ในไฟล์อ้างอิงตำแหน่งบนเครื่องผู้วิจัย ปรับตามเครื่องที่ใช้)
