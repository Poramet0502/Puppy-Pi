"""
สั่งหุ่นยนต์ PuppyPi เดินหน้าประมาณ 1 เมตร แล้วทำท่า "pee" (ท่าสำเร็จรูปของโรงงาน)
คุมผ่าน rosbridge (roslibpy) จาก host เท่านั้น — ไม่แตะโค้ดเดิมของโรงงานเลย

หมายเหตุสำคัญ: หุ่นยนต์ไม่มีเซนเซอร์วัดระยะทางที่เดินจริง (ไม่มี wheel/leg odometry)
ระยะ 1 เมตรจึงเป็นการประมาณแบบ dead-reckoning จาก (ความเร็ว x หน่วย cm/s) x (เวลา)
ระยะจริงจะคลาดเคลื่อนได้ตามพื้นผิว/แรงเสียดทาน/การลื่นไถลของขา ควรวัดระยะจริงเทียบด้วยตาเปล่า
แล้วปรับค่า FORWARD_SPEED_CMS ให้ตรงกับพื้นผิวที่ใช้งานจริงภายหลัง

วิธีใช้:
    python3 walk_and_pee.py
"""

import time

import roslibpy

ROSBRIDGE_HOST = "127.0.0.1"
ROSBRIDGE_PORT = 9090

FORWARD_SPEED_CMS = 15.0  # cm/s — ค่าเดียวกับที่ demo ของโรงงานใช้ (puppy_demo.py), max ที่ระบบรับคือ 35 cm/s
TARGET_DISTANCE_CM = 100.0  # 1 เมตร
WALK_DURATION_SEC = TARGET_DISTANCE_CM / FORWARD_SPEED_CMS

# ค่า gait/pose อ้างอิงจาก puppy_demo.py ของโรงงาน (ท่าเดินแบบ Trot มาตรฐาน)
GAIT = {"overlap_time": 0.2, "swing_time": 0.3, "clearance_time": 0.0, "z_clearance": 5}
POSE = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "height": -10.0,
    "x_shift": 2.0,
    "stance_x": 0.0,
    "stance_y": 0.0,
    "run_time": 500,
}
STOP_VELOCITY = {"x": 0.0, "y": 0.0, "yaw_rate": 0.0}


def main():
    client = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    client.run()
    if not client.is_connected:
        print("เชื่อมต่อ rosbridge ไม่สำเร็จ ตรวจสอบว่า ROS ของหุ่นยนต์ทำงานอยู่หรือไม่")
        return
    print("เชื่อมต่อ rosbridge สำเร็จ")

    gait_pub = roslibpy.Topic(client, "/puppy_control/gait", "puppy_control/Gait")
    pose_pub = roslibpy.Topic(client, "/puppy_control/pose", "puppy_control/Pose")
    vel_pub = roslibpy.Topic(client, "/puppy_control/velocity", "puppy_control/Velocity")
    action_srv = roslibpy.Service(client, "/puppy_control/runActionGroup", "puppy_control/SetRunActionName")
    go_home_srv = roslibpy.Service(client, "/puppy_control/go_home", "std_srvs/Empty")

    try:
        print("สั่งยืนกลับตำแหน่งเริ่มต้น (go_home) ก่อนเดิน")
        go_home_srv.call(roslibpy.ServiceRequest({}))
        time.sleep(1.5)

        gait_pub.publish(roslibpy.Message(GAIT))
        time.sleep(0.3)
        pose_pub.publish(roslibpy.Message(POSE))
        time.sleep(0.5)

        print(
            f"เดินหน้าด้วยความเร็ว {FORWARD_SPEED_CMS} cm/s เป็นเวลา {WALK_DURATION_SEC:.1f} วินาที "
            f"(ระยะประมาณ {TARGET_DISTANCE_CM / 100:.1f} เมตร)"
        )
        vel_pub.publish(roslibpy.Message({"x": FORWARD_SPEED_CMS, "y": 0.0, "yaw_rate": 0.0}))
        time.sleep(WALK_DURATION_SEC)

        print("หยุดเดิน")
        vel_pub.publish(roslibpy.Message(STOP_VELOCITY))
        time.sleep(1.0)  # รอให้ขาหยุดนิ่งสนิทก่อนทำท่าถัดไป

        print("สั่งทำท่า 'pee'")
        # ชื่อต้องตรงกับไฟล์จริงใน ActionGroups/ (มีนามสกุล .d6ac ต่อท้ายเสมอ) ไม่งั้นระบบจะหาไฟล์ไม่เจอ
        # แต่ service ของโรงงานจะตอบ success:True เสมอแม้หาไฟล์ไม่เจอ (บั๊กของโค้ดเดิม) ต้องระวังจุดนี้
        request = roslibpy.ServiceRequest({"name": "pee.d6ac", "wait": True})
        result = action_srv.call(request)
        print("ผลลัพธ์การทำท่า:", result)

    finally:
        vel_pub.publish(roslibpy.Message(STOP_VELOCITY))
        time.sleep(0.2)
        gait_pub.unadvertise()
        pose_pub.unadvertise()
        vel_pub.unadvertise()
        client.terminate()
        print("ปิดการเชื่อมต่อแล้ว")


if __name__ == "__main__":
    main()
