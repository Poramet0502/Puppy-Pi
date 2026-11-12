# PuppyPi — ระบบตรวจจับและชี้เป้าด้วยเลเซอร์บนหุ่นยนต์สุนัขสี่ขา

โค้ดและข้อมูลประกอบปริญญานิพนธ์ **"การพัฒนาระบบตรวจจับและวิเคราะห์พิกัดเป้าหมายเชิงพื้นที่สำหรับหุ่นยนต์สุนัข"** (Development of a Target Detection and Spatial Coordinate Analysis System for a Robot Dog) สาขาวิศวกรรมหุ่นยนต์ คณะวิศวกรรมศาสตร์และเทคโนโลยีอุตสาหกรรม มหาวิทยาลัยราชภัฏสวนสุนันทา

ระบบทำงานบนหุ่นยนต์ Hiwonder PuppyPi (Raspberry Pi 4B) ติดตั้งฐานเล็งเป้า 2 แกน (Pan-Tilt) พร้อมเลเซอร์ ตรวจจับเป้าสีแดงด้วย HSV + Contour คำนวณระยะด้วยกล้องตัวเดียว (D = f·W/P) แล้วคำนวณมุมชี้เป้าสัมบูรณ์จากภาพเดียว เพราะกล้องยึดนิ่งกับลำตัวแบบ Eye-to-Hand

โค้ดทำงานเป็นไคลเอนต์ของ ROS Noetic ของผู้ผลิต ผ่าน rosbridge (พอร์ต 9090) และดึงภาพจาก web_video_server (พอร์ต 8080) **ไม่ได้แก้โค้ดของผู้ผลิต**

## โครงสร้าง

```
Puppy-Pi/
├── motion/
│   ├── pan_tilt_laser_app.py   ← เว็บแอปหลัก
│   └── walk_test.py            ← หน้า /walk-test
├── vision/
│   └── red_detect_test.py      ← ดึงภาพและตรวจจับเป้า
├── deploy/                     ← systemd service
├── data/                       ← ข้อมูลผลการทดลองทั้ง 3 การทดลอง (CSV)
└── docs/THESIS_CODE_MAP.md     ← ตารางเทียบสมการในเล่มกับฟังก์ชันในโค้ด
```

ไฟล์โค้ดอยู่ในตำแหน่งเดียวกับบนหุ่นยนต์ทุกไฟล์ (`/home/pi/Desktop/Poramet_Project`) เพราะแอปหลักอ้าง path นี้ตรง ๆ ตอน import `vision/red_detect_test.py`

## ไฟล์โค้ด

repo นี้มีเฉพาะโค้ดที่ใช้งานจริงในการทดลอง

| ไฟล์ | หน้าที่ |
|---|---|
| `motion/pan_tilt_laser_app.py` | เว็บแอปหลัก (Flask, พอร์ต 8091): ควบคุม Pan/Tilt/เลเซอร์, Auto-Track, หน้า `/distance-test` (การทดลองที่ 1), `/aiming-test` (การทดลองที่ 2), สูตรระยะ/มุม/พัลส์ (`compute_aim()`), สถิติ, export CSV |
| `motion/walk_test.py` | หน้า `/walk-test` (การทดลองที่ 3) ผลของฐานเล็งเป้าต่อการเดิน |
| `vision/red_detect_test.py` | ดึงภาพกล้อง (`grab_frame()`) และตรวจจับเป้าสีแดง (`detect_red()`) |
| `deploy/pan_tilt_laser_app.service` | systemd unit ให้แอปรันอัตโนมัติตอนเปิดเครื่อง |

ค่าที่ใช้เก็บผลทดลอง: f = 616 px, W = 8.0 cm, ความสูงเลเซอร์เหนือกล้อง h = 10.5 cm, Δpan = −242 µs, Δtilt = +65 µs, Home = Pan 1500 / Tilt 800 µs, เข้าเป้าจากทิศเดียวผ่านจุดแวะ +60 µs, เกณฑ์เข้าเป้า E ≤ 4.0 cm

## ข้อมูลผลการทดลอง

| ไฟล์ | การทดลอง |
|---|---|
| `data/exp1_distance_test_300.csv` | การทดลองที่ 1 ความแม่นยำของระยะทาง 300 ครั้ง |
| `data/exp2_aiming_test_300.csv` | การทดลองที่ 2 ความแม่นยำของการชี้เป้า 300 ครั้ง |
| `data/exp3_walk_test_60.csv` | การทดลองที่ 3 การเดิน 60 ครั้ง |

ความหมายของแต่ละคอลัมน์และวันเวลาที่เก็บผล อยู่ใน [`data/README.md`](data/README.md)

## สภาพแวดล้อม

Raspberry Pi 4B · Debian 12 · Python 3.11.2 · Flask 2.2.2 · roslibpy 2.1.0 · OpenCV 4.9.0 · NumPy 1.24.2 · requests 2.28.1 · gpiozero 2.0.1 · ROS Noetic ของผู้ผลิตใน Docker (รายการอยู่ใน `requirements.txt`)

## วิธีรัน

```bash
cd motion
python3 pan_tilt_laser_app.py
# เปิด http://<IP ของ Pi>:8091/
```

ติดตั้งเป็น service: คัดลอก `deploy/pan_tilt_laser_app.service` ไป `/etc/systemd/system/` แล้วรัน `sudo systemctl enable --now pan_tilt_laser_app` (path ในไฟล์อ้างอิงตำแหน่งบนเครื่องผู้วิจัย ปรับตามเครื่องที่ใช้)

⚠️ ระหว่างใช้งานห้ามหันเลเซอร์ไปทางคนหรือมองเข้าลำแสงโดยตรง
