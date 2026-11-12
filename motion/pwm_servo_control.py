"""
คุมเซอร์โว PWM ผ่าน rosbridge จาก host — ไม่แตะโค้ดเดิมของโรงงาน
topic นี้ (`/ros_robot_controller/pwm_servo/set_state`) คือ topic เดียวกับที่ขับขาหุ่นยนต์จริงทั้ง 8 ข้อ
(ผ่าน HiwonderPuppy -> setServoPulse -> PWMServoControl.setPulse -> board.pwm_servo_set_position)
ยืนยันแล้วว่าใช้งานได้จริงกับเซอร์โวใหม่ที่ id=10 (พิสูจน์ 2026-08-12 เห็นเซอร์โวหมุนจริง)

⚠️ แก้ไขความเข้าใจผิดจากรอบก่อนหน้า: เคยเข้าใจผิดว่าบอร์ดมีช่อง PWM แค่ 1-4 และช่อง 9-11 เป็น "bus servo"
   ที่ต้องสั่งผ่าน topic bus_servo/set_state — ความจริงคือขาทั้งหมด (id 1-8) และช่องขยาย (id 9-11)
   ใช้ topic pwm_servo/set_state เดียวกันหมด (ดูหลักฐานที่ยืนยันใน PROJECT_STATUS.md)
   topic bus_servo/set_state ต่างหากที่ไม่เคยถูกใช้งานจริงและไม่น่าเชื่อถือ — ห้ามใช้

หน่วย position คือ pulse width หน่วย microsecond (us) ช่วงที่ใช้ได้ 500-2500us
กึ่งกลาง (มักประมาณ 90 องศา) ส่วนใหญ่อยู่ที่ 1500us แต่ควรทดสอบเพื่อหาขอบเขตจริงของเซอร์โวที่ใช้
(อย่าสั่งค่าที่สุดขั้วเกินไปทันที เสี่ยงเซอร์โวชนปลายทางกลไกและไหม้ได้)

⚠️ id 1-8 คือขาหุ่นยนต์จริง ห้ามสั่งเล่นเด็ดขาด (จะไปรบกวนการทรงตัว/เดิน) ใช้ได้เฉพาะ id 9, 10, 11
   ที่เป็นช่องขยายว่างเท่านั้น

วิธีใช้:
    python3 pwm_servo_control.py --id 10 --position 1500              # สั่งครั้งเดียว
    python3 pwm_servo_control.py --id 10 --position 1500 --duration 1 # ค่อยๆขยับใน 1 วินาที
    python3 pwm_servo_control.py --sweep --id 10                      # กวาดซ้าย-กลาง-ขวา ทดสอบขอบเขต (ค่อยๆขยับทีละนิด)
"""

import argparse
import time

import roslibpy

ROSBRIDGE_HOST = "127.0.0.1"
ROSBRIDGE_PORT = 9090

CENTER_US = 1500  # ค่ากึ่งกลางเริ่มต้น (ทั่วไปของเซอร์โว PWM มาตรฐาน) ปรับตามจริงหลัง calibrate


def connect():
    client = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    client.run()
    if not client.is_connected:
        raise RuntimeError("เชื่อมต่อ rosbridge ไม่สำเร็จ ตรวจสอบว่า ROS ของหุ่นยนต์ทำงานอยู่หรือไม่")
    return client


def set_servo_position(client, servo_id, position, duration=0.5):
    pub = roslibpy.Topic(
        client, "/ros_robot_controller/pwm_servo/set_state", "ros_robot_controller/SetPWMServoState"
    )
    msg = {
        "state": [{"id": [servo_id], "position": [position], "offset": [0]}],
        "duration": duration,
    }
    pub.publish(roslibpy.Message(msg))
    time.sleep(duration + 0.1)
    pub.unadvertise()


def sweep_test(client, servo_id):
    """ทดสอบขอบเขตแบบค่อยเป็นค่อยไป: กลาง -> ลด 300us -> กลาง -> เพิ่ม 300us -> กลาง
    เริ่มด้วยระยะสั้นๆ ก่อนเพื่อความปลอดภัย ไม่กวาดสุดขอบทันที"""
    steps = [CENTER_US, CENTER_US - 300, CENTER_US, CENTER_US + 300, CENTER_US]
    for pos in steps:
        print(f"  -> id={servo_id} position={pos}us")
        set_servo_position(client, servo_id, pos, duration=0.6)
        time.sleep(0.4)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=int, required=True, help="หมายเลข PWM servo id (ใช้ได้เฉพาะ 9, 10, 11 — ช่องขยายว่าง)")
    parser.add_argument("--position", type=int, help="pulse width (us) ทั่วไป 500-2500")
    parser.add_argument("--duration", type=float, default=0.5, help="เวลาที่ใช้เคลื่อนที่ (วินาที)")
    parser.add_argument("--sweep", action="store_true", help="ทดสอบขอบเขตแบบค่อยเป็นค่อยไป (ปลอดภัยกว่า)")
    args = parser.parse_args()

    if args.id not in (9, 10, 11):
        print(f"⚠️  id={args.id} ไม่ถูกต้อง — id 1-8 คือขาหุ่นยนต์จริง ห้ามสั่งเล่น ใช้ได้เฉพาะ 9, 10, 11 เท่านั้น")
        return

    client = connect()
    try:
        if args.sweep:
            print(f"ทดสอบขอบเขตเซอร์โว id={args.id} แบบค่อยเป็นค่อยไป")
            sweep_test(client, args.id)
        else:
            if args.position is None:
                print("ต้องระบุ --position หรือใช้ --sweep")
                return
            print(f"สั่งเซอร์โว id={args.id} ไปตำแหน่ง {args.position}us ในเวลา {args.duration}s")
            set_servo_position(client, args.id, args.position, args.duration)
    finally:
        client.terminate()


if __name__ == "__main__":
    main()
