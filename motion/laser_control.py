"""
คุมโมดูลเลเซอร์ (KY-008 650nm 5V) ที่ต่อเข้าขา GPIO7 ของ Raspberry Pi โดยตรง
(ใช้ขั้วต่อ "5V GND IO7 IO8" บนบอร์ดควบคุม — IO7=GPIO7 ตรงตัว, ว่างอยู่เพราะไม่ได้ใช้จอ dot matrix)
ไม่เกี่ยวข้องกับ ROS/rosbridge เลย เพราะคุมผ่าน GPIO ของ Pi ตรงๆ แยกขาดจากบอร์ดควบคุมเดิม 100%
ไม่แตะโค้ดเดิมของโรงงาน

การต่อสาย:
    เลเซอร์ + -> 5V (ขั้วต่อเดียวกับ IO7/IO8 บนบอร์ด)
    เลเซอร์ - -> GND
    เลเซอร์ S -> IO7 (GPIO7)

วิธีใช้:
    python3 laser_control.py on
    python3 laser_control.py off
    python3 laser_control.py blink   # กระพริบทดสอบ 3 ครั้ง
"""

import sys
import time

try:
    from gpiozero import LED
except ImportError:
    print("❌ ต้องติดตั้ง gpiozero ก่อน:")
    print("   pip3 install --user gpiozero")
    sys.exit(1)

LASER_PIN = 7  # GPIO7 (BCM) ตรงกับป้าย "IO7" บนบอร์ด


class LaserControl:
    def __init__(self, pin=LASER_PIN):
        try:
            self.laser = LED(pin)
            self.pin = pin
            print(f"✓ เชื่อมต่อเลเซอร์ที่ GPIO{self.pin} สำเร็จ")
        except Exception as e:
            print(f"❌ เกิดข้อผิดพลาด: {e}")
            sys.exit(1)
    
    def on(self):
        """เปิดเลเซอร์"""
        self.laser.on()
        print(f"🔴 เลเซอร์: ON")
    
    def off(self):
        """ปิดเลเซอร์"""
        self.laser.off()
        print(f"⚫ เลเซอร์: OFF")
    
    def test(self):
        """ทดสอบเลเซอร์"""
        print(f"\n{'='*50}")
        print(f"ทดสอบเลเซอร์ Ky-008 650nm (GPIO{self.pin})")
        print(f"{'='*50}\n")
        
        # ทดสอบ 1: กระพริบ 5 ครั้ง
        print("📌 ทดสอบที่ 1: กระพริบเลเซอร์ 5 ครั้ง")
        for i in range(5):
            self.on()
            time.sleep(0.3)
            self.off()
            time.sleep(0.3)
        print("✓ เสร็จ\n")
        
        # ทดสอบ 2: เปิด 2 วินาที
        print("📌 ทดสอบที่ 2: เปิดเลเซอร์ 2 วินาที")
        self.on()
        time.sleep(2)
        self.off()
        print("✓ เสร็จ\n")
        
        # ทดสอบ 3: พัลส์เร็ว 10 ครั้ง
        print("📌 ทดสอบที่ 3: พัลส์เร็ว 10 ครั้ง")
        for i in range(10):
            self.laser.toggle()
            time.sleep(0.1)
        self.off()
        print("✓ เสร็จ\n")
        
        print(f"{'='*50}")
        print("✅ ทดสอบเสร็จทั้งหมด")
        print(f"{'='*50}\n")
    
    def blink(self, times=3):
        """กระพริบ N ครั้ง"""
        print(f"💫 กระพริบเลเซอร์ {times} ครั้ง...")
        for i in range(times):
            self.on()
            time.sleep(0.5)
            self.off()
            time.sleep(0.5)
        print("✓ เสร็จ")
    
    def pulse(self, times=20):
        """พัลส์เลเซอร์"""
        print(f"🔄 พัลส์เลเซอร์ {times} ครั้ง...")
        for i in range(times):
            self.laser.toggle()
            time.sleep(0.1)
        self.off()
        print("✓ เสร็จ")
    
    def cleanup(self):
        """ปิดและล้าง GPIO"""
        self.laser.close()
        print("🔌 ปิดการเชื่อมต่อ GPIO")


def main():
    if len(sys.argv) < 2:
        print("วิธีใช้:")
        print("  python3 laser_control.py on              # เปิดเลเซอร์")
        print("  python3 laser_control.py off             # ปิดเลเซอร์")
        print("  python3 laser_control.py blink [จำนวน]  # กระพริบ (default 3 ครั้ง)")
        print("  python3 laser_control.py pulse [จำนวน]  # พัลส์ (default 20 ครั้ง)")
        print("  python3 laser_control.py test            # ทดสอบแบบสมบูรณ์")
        sys.exit(1)
    
    laser = LaserControl()
    action = sys.argv[1].lower()
    
    try:
        if action == "on":
            laser.on()
            print("\nกด Ctrl+C เพื่อปิดเลเซอร์...")
            while True:
                time.sleep(1)
        
        elif action == "off":
            laser.off()
        
        elif action == "blink":
            times = int(sys.argv[2]) if len(sys.argv) > 2 else 3
            laser.blink(times)
        
        elif action == "pulse":
            times = int(sys.argv[2]) if len(sys.argv) > 2 else 20
            laser.pulse(times)
        
        elif action == "test":
            laser.test()
        
        else:
            print(f"❌ คำสั่งไม่รู้จัก: {action}")
            print("ใช้: python3 laser_control.py [on|off|blink|pulse|test]")
    
    except KeyboardInterrupt:
        print("\n\n⏹️  ขัดจังหวะ")
    
    except Exception as e:
        print(f"❌ เกิดข้อผิดพลาด: {e}")
    
    finally:
        laser.off()
        laser.cleanup()


if __name__ == "__main__":
    main()
