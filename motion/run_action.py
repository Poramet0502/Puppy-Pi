"""
สั่งให้ PuppyPi เล่นท่า action group ใดก็ได้ตามชื่อไฟล์จริง (ใช้ทดสอบทีละท่าโดยไม่ต้องเดินก่อน)
คุมผ่าน rosbridge (roslibpy) จาก host เท่านั้น — ไม่แตะโค้ดเดิมของโรงงาน

⚠️ ต้องใส่ชื่อไฟล์เต็มพร้อมนามสกุล (.d6a หรือ .d6ac) ให้ตรงกับที่มีจริงใน
   /home/ubuntu/software/puppypi_control/ActionGroups/ ในคอนเทนเนอร์ ไม่งั้นจะหาไฟล์ไม่เจอ
   แต่ service จะตอบ success:True เสมอแม้หาไฟล์ไม่เจอ (บั๊กของโค้ดเดิม) จึงต้องเช็คด้วยตาว่าหุ่นขยับจริงไหม

ท่าที่มีให้เลือก (ตัวอย่าง): stand.d6ac, sit.d6ac, lie_down.d6ac, bow.d6ac, nod.d6ac,
    shake_head.d6ac, shake_hands.d6ac, wave.d6ac, jump.d6ac, push-up.d6ac, boxing.d6ac,
    boxing2.d6ac, moonwalk.d6ac, spacewalk.d6ac, stretch.d6ac, pee.d6ac, 2_legs_stand.d6ac,
    kick_ball_left.d6ac, kick_ball_right.d6ac, up_stairs_2cm.d6ac, up_stairs_3.5cm.d6ac,
    grab.d6a, place.d6a, look_down.d6a

วิธีใช้:
    python3 run_action.py pee.d6ac
    python3 run_action.py wave.d6ac
"""

import sys

import roslibpy

ROSBRIDGE_HOST = "127.0.0.1"
ROSBRIDGE_PORT = 9090


def run_action(name, wait=True):
    client = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    client.run()
    if not client.is_connected:
        print("เชื่อมต่อ rosbridge ไม่สำเร็จ")
        return None

    action_srv = roslibpy.Service(client, "/puppy_control/runActionGroup", "puppy_control/SetRunActionName")
    try:
        print(f"สั่งเล่นท่า: {name}")
        result = action_srv.call(roslibpy.ServiceRequest({"name": name, "wait": wait}))
        print("ผลลัพธ์ (หมายเหตุ: service เดิมตอบ success:True เสมอ ไม่ได้ยืนยันว่าไฟล์มีจริง):", result)
        return result
    finally:
        client.terminate()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("วิธีใช้: python3 run_action.py <ชื่อไฟล์ท่า เช่น pee.d6ac>")
        sys.exit(1)
    run_action(sys.argv[1])
