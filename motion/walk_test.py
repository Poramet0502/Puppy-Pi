#!/usr/bin/env python3
"""
walk_test.py — หน้าทดสอบการเดิน /walk-test (เพิ่ม 2026-10-28)

ขอบเขตงานข้อ 1: ติดตั้งฐานเล็งเป้า 2 แกนบนหลัง PuppyPi โดยไม่กระทบสมดุล/การเดิน
→ สั่งเดินหน้าตามระยะ/ความเร็วที่ตั้ง หยุดเอง แล้วผู้ทดลองวัดระยะจริง ระยะเบี่ยง และล้มหรือไม่ แยกเงื่อนไข มี/ไม่มีฐานเล็งเป้า

⚠️ /puppy_control/velocity เป็นแบบค้างค่า: ส่งความเร็วครั้งเดียว หุ่นจะเดินไปเรื่อยๆ จนกว่าจะได้ความเร็ว 0
(เคยเกิดเหตุหุ่นเดินไม่หยุด 2026-09-15) จึงต้องสั่งหยุดใน finally เสมอ + ปุ่มหยุดฉุกเฉิน + สั่งหยุดเมื่อต่อ ROS ติดใหม่
ห้ามทดสอบด้วย rostopic pub/docker exec ตรงๆ

แอปหลักเรียก register(app, globals()) ก่อนบรรทัด if __name__ == '__main__':
ถอดออก: ลบ 2 บรรทัดที่เรียก register ในแอปหลัก (ไฟล์ข้อมูล walk_test_data.json ไม่ถูกลบ)
"""
import math
import os
import threading
import time

import roslibpy

DATA_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'walk_test_data.json')

# เดินแบบเดียวกับจอยของโรงงาน (remote_control_joystick.py): go_home (ท่ายืน Stand) แล้วส่งความเร็วเข้า
# /puppy_control/velocity/autogait — puppy.py ปรับจังหวะก้าวตามความเร็วเอง ที่ ≤10 cm/s ก้าวช้า มีช่วงวางเท้า
# (clearance_time) นุ่มและกินไฟน้อย ส่วน >15 cm/s ใช้จังหวะเร็วคงที่ (0.1/0.15 วิ) — เดิมใช้ /velocity + gait/pose ของ
# puppy_demo.py (x_shift 2.0 เอนหลัง) ผู้ใช้แจ้ง 2026-10-28 ว่าเดินแรง/เร็ว แบตหมดไว เทียบกับแอปโรงงาน
MAX_SPEED_CMS = 15.0      # จอยโรงงานเริ่มที่ 15 ปรับได้ 5–25
DEFAULT_SPEED_CMS = 8.0
MAX_DURATION_SEC = 25.0   # ที่ 5–8 cm/s เดิน 100 cm ใช้ 12–20 วิ
MIN_BATTERY_PCT = 30      # แบตต่ำ เซอร์โวขาอาจแรงไม่พอ หุ่นทรุดระหว่างเดิน
CONDITIONS = ('with_mount', 'without_mount')
WALK_MODE = 'autogait'
STOP = {'x': 0.0, 'y': 0.0, 'yaw_rate': 0.0}


def register(app, g):
    from flask import jsonify, request, render_template_string

    records = g['load_json_records'](DATA_FILE)
    lock = threading.Lock()
    stop_event = threading.Event()
    state = {'walking': False, 'phase': 'idle', 'started': None, 'duration': 0.0, 'message': 'พร้อม'}
    last_walk = {'info': None}  # บันทึกผลได้เฉพาะหลังเดินจริง 1 ครั้งต่อ 1 แถว
    pubs = {'ros': None}

    def topics():
        ros = g['ros']
        if ros is None or not ros.is_connected:
            raise RuntimeError('ROS ไม่ได้เชื่อมต่อ')
        if pubs['ros'] is not ros:
            pubs.update({
                'ros': ros,
                'vel': roslibpy.Topic(ros, '/puppy_control/velocity/autogait', 'puppy_control/Velocity'),
                'vel_plain': roslibpy.Topic(ros, '/puppy_control/velocity', 'puppy_control/Velocity'),
                'home': roslibpy.Service(ros, '/puppy_control/go_home', 'std_srvs/Empty'),
            })
        return pubs

    def send_stop(times=3):
        for _ in range(times):
            try:
                t = topics()
                t['vel'].publish(roslibpy.Message(STOP))
                t['vel_plain'].publish(roslibpy.Message(STOP))  # เผื่อมีอะไรสั่งผ่าน /velocity ค้างไว้
            except Exception as e:
                print(f'⚠️ walk_test: สั่งหยุดไม่สำเร็จ: {e}')
            time.sleep(0.1)

    def stop_on_connect():
        """ถ้าแอปตายกลางการเดินแล้ว systemd รันใหม่ หุ่นยังเดินค้างอยู่ — ต่อ ROS ติดเมื่อไหร่สั่งหยุดทันที"""
        while True:
            if g['ros'] is not None and g['ros'].is_connected and g['servo_pub'] is not None:
                send_stop()
                print('✓ walk_test: สั่งความเร็ว 0 หลังเชื่อมต่อ ROS แล้ว')
                return
            time.sleep(1)

    threading.Thread(target=stop_on_connect, daemon=True).start()

    def walk_thread(speed, duration, cmd_distance, condition, battery):
        completed = False
        try:
            state.update({'phase': 'prepare', 'message': 'เตรียมท่ายืน (go_home)...'})
            g['aim_go_home']()
            t = topics()
            t['home'].call(roslibpy.ServiceRequest({}), timeout=5)
            if stop_event.wait(1.5):
                return
            state.update({'phase': 'walking', 'started': time.time(), 'message': f'กำลังเดิน {speed} cm/s'})
            t['vel'].publish(roslibpy.Message({'x': float(speed), 'y': 0.0, 'yaw_rate': 0.0}))
            completed = not stop_event.wait(duration)
        except Exception as e:
            state['message'] = f'❌ error: {e}'
            print(f'❌ walk_test: {e}')
        finally:
            send_stop()
            last_walk['info'] = {'condition': condition, 'walk_mode': WALK_MODE, 'speed_cms': speed, 'duration_s': round(duration, 2),
                                 'cmd_distance_cm': cmd_distance, 'completed': completed,
                                 'battery_pct_start': battery, 'walk_time': time.strftime('%Y-%m-%d %H:%M:%S')}
            state.update({'walking': False, 'phase': 'idle', 'started': None,
                          'message': '✅ เดินครบแล้ว หยุดแล้ว — วัดระยะแล้วบันทึก' if completed
                          else (state['message'] if state['message'].startswith('❌') else '⏹️ หยุดก่อนครบ')})
            lock.release()

    @app.route('/api/walktest/start', methods=['POST'])
    def walktest_start():
        d = request.json or {}
        try:
            speed = float(d.get('speed_cms'))
            cmd_distance = float(d.get('distance_cm'))
        except (TypeError, ValueError):
            return jsonify({'success': False, 'error': 'กรอกความเร็วและระยะเป็นตัวเลข'})
        condition = d.get('condition')
        if condition not in CONDITIONS:
            return jsonify({'success': False, 'error': 'เลือกเงื่อนไขการทดสอบก่อน'})
        if not (0 < speed <= MAX_SPEED_CMS):
            return jsonify({'success': False, 'error': f'ความเร็วต้องอยู่ระหว่าง 0–{MAX_SPEED_CMS:g} cm/s'})
        duration = cmd_distance / speed
        if not (0 < duration <= MAX_DURATION_SEC):
            return jsonify({'success': False, 'error': f'ระยะ/ความเร็ว = {duration:.1f} วิ เกิน {MAX_DURATION_SEC:g} วิ ลดระยะหรือเพิ่มความเร็ว'})
        battery = g['battery_state'].get('percent')
        if battery is None:
            return jsonify({'success': False, 'error': 'ไม่มีข้อมูลแบตเตอรี่ (ROS อาจหลุด) — ไม่สั่งเดิน'})
        if battery < MIN_BATTERY_PCT:
            return jsonify({'success': False, 'error': f'แบตเตอรี่ {battery}% ต่ำกว่า {MIN_BATTERY_PCT}% — ชาร์จก่อน'})
        if g['autotrack_running'].is_set():
            return jsonify({'success': False, 'error': 'ปิด Auto-Track ในหน้าหลักก่อน'})
        if not lock.acquire(blocking=False):
            return jsonify({'success': False, 'error': 'กำลังเดินอยู่'})
        try:
            topics()
        except Exception as e:
            lock.release()
            return jsonify({'success': False, 'error': str(e)})
        stop_event.clear()
        state.update({'walking': True, 'duration': duration})
        threading.Thread(target=walk_thread, args=(speed, duration, cmd_distance, condition, battery), daemon=True).start()
        return jsonify({'success': True, 'duration': round(duration, 2)})

    @app.route('/api/walktest/stop', methods=['POST'])
    def walktest_stop():
        stop_event.set()
        send_stop()
        return jsonify({'success': True})

    @app.route('/api/walktest/status')
    def walktest_status():
        elapsed = time.time() - state['started'] if state['started'] else 0
        return jsonify({**{k: state[k] for k in ('walking', 'phase', 'duration', 'message')},
                        'elapsed': round(elapsed, 1), 'battery_pct': g['battery_state'].get('percent'),
                        'can_save': last_walk['info'] is not None, 'last_walk': last_walk['info']})

    @app.route('/api/walktest/save', methods=['POST'])
    def walktest_save():
        info = last_walk['info']
        if info is None:
            return jsonify({'success': False, 'error': 'ต้องกดเริ่มเดินก่อน 1 ครั้งต่อ 1 แถว'})
        d = request.json or {}
        fell = bool(d.get('fell'))
        rec = {**info, 'fell': fell, 'note': d.get('note', ''), 'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')}
        if not fell:
            try:
                actual = float(d.get('actual_distance_cm'))
                lateral = float(d.get('lateral_cm'))
            except (TypeError, ValueError):
                return jsonify({'success': False, 'error': 'กรอกระยะที่เดินได้จริงและระยะเบี่ยง (ตัวเลข)'})
            err = actual - info['cmd_distance_cm']
            rec.update({'actual_distance_cm': actual, 'lateral_cm': lateral, 'error_cm': round(err, 2),
                        'error_pct': round(err / info['cmd_distance_cm'] * 100, 2)})
        records.append(rec)
        g['save_json_records'](DATA_FILE, records)
        last_walk['info'] = None
        return jsonify({'success': True})

    @app.route('/api/walktest/list')
    def walktest_list():
        return jsonify(records)

    @app.route('/api/walktest/delete', methods=['POST'])
    def walktest_delete():
        i = (request.json or {}).get('index')
        if not isinstance(i, int) or not (0 <= i < len(records)):
            return jsonify({'success': False})
        records.pop(i)
        g['save_json_records'](DATA_FILE, records)
        return jsonify({'success': True})

    def stats():
        mean, sd = g['_mean'], g['_sd']
        out = []
        for cond in CONDITIONS:
            rows = [r for r in records if r['condition'] == cond]
            if not rows:
                continue
            ok = [r for r in rows if not r['fell'] and 'actual_distance_cm' in r]
            s = {'condition': cond, 'n': len(rows), 'n_fell': sum(r['fell'] for r in rows),
                 'n_stopped_early': sum(not r['completed'] for r in rows),
                 'stability_pct': round((len(rows) - sum(r['fell'] for r in rows)) / len(rows) * 100, 2)}
            if ok:
                act = [r['actual_distance_cm'] for r in ok]
                lat = [r['lateral_cm'] for r in ok]
                abs_pct = [abs(r['error_pct']) for r in ok]
                s.update({'n_measured': len(ok), 'mean_actual_cm': round(mean(act), 2), 'sd_actual_cm': round(sd(act), 2),
                          'mean_error_cm': round(mean([r['error_cm'] for r in ok]), 2),
                          'mean_abs_error_pct': round(mean(abs_pct), 2),
                          'mean_lateral_cm': round(mean(lat), 2), 'mean_abs_lateral_cm': round(mean([abs(x) for x in lat]), 2),
                          'sd_lateral_cm': round(sd(lat), 2)})
            out.append(s)
        return out

    @app.route('/api/walktest/stats')
    def walktest_stats():
        return jsonify(stats())

    @app.route('/api/walktest/export.csv')
    def walktest_export():
        fields = ['condition', 'walk_mode', 'speed_cms', 'duration_s', 'cmd_distance_cm', 'actual_distance_cm', 'error_cm', 'error_pct',
                  'lateral_cm', 'fell', 'completed', 'battery_pct_start', 'note', 'walk_time', 'timestamp']
        return g['_csv_response'](records, fields, 'walk_test_data.csv')

    @app.route('/walk-test')
    def walktest_page():
        return render_template_string(WALK_HTML, max_speed=MAX_SPEED_CMS, max_dur=MAX_DURATION_SEC, min_bat=MIN_BATTERY_PCT,
                                      default_speed=DEFAULT_SPEED_CMS)


WALK_HTML = """<!DOCTYPE html><html lang="th"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0"><title>Walk Test</title>
<style>
body{font-family:Tahoma,sans-serif;background:linear-gradient(135deg,#667eea,#764ba2);margin:0;padding:16px;color:#222;min-height:100vh}
h1{text-align:center;color:#fff;font-size:1.5em;margin:6px 0}.nav{text-align:center;margin-bottom:14px}.nav a{color:#fff;margin:0 8px}
.wrap{max-width:1000px;margin:0 auto;display:grid;grid-template-columns:1fr 1fr;gap:16px}@media(max-width:800px){.wrap{grid-template-columns:1fr}}
.card{background:#fff;border-radius:14px;padding:16px;box-shadow:0 6px 18px #0002}h2{font-size:1.05em;color:#4b4fd8;margin:0 0 10px}
label{display:block;font-size:.85em;color:#555;margin-top:8px}input,select{width:100%;box-sizing:border-box;padding:8px;border:1px solid #ccc;border-radius:8px;margin-top:3px}
button{width:100%;padding:12px;border:0;border-radius:10px;font-weight:700;color:#fff;cursor:pointer;margin-top:10px;font-size:1em}
.go{background:#2ecc71}.stop{background:#e74c3c;font-size:1.3em;padding:18px}.save{background:#4b4fd8}button:disabled{opacity:.45;cursor:not-allowed}
.hint{color:#888;font-size:.78em;margin-top:6px}.status{background:#f5f6fa;border-radius:8px;padding:10px;margin-top:10px;font-weight:600}
.stat{background:#eef0ff;border-radius:8px;padding:10px;margin-bottom:8px;font-size:.88em}
table{width:100%;border-collapse:collapse;font-size:.8em}td,th{border-bottom:1px solid #eee;padding:5px;text-align:center}
.del{background:#ff6b6b;padding:4px 8px;width:auto;margin:0;font-size:.8em}
</style></head><body>
<h1>🐕 ทดสอบการเดิน (ฐานเล็งเป้าไม่กระทบสมดุล/การเดิน)</h1>
<div class="nav"><a href="/">← หน้าควบคุมหลัก</a> | <a href="/distance-test">📏 ทดสอบระยะทาง</a> | <a href="/aiming-test">🎯 ทดสอบการชี้เป้า</a></div>
<div class="wrap">
<div>
 <div class="card"><h2>1. ตั้งค่าแล้วเริ่มเดิน</h2>
  <label>เงื่อนไข<select id="cond"><option value="">— เลือก —</option><option value="with_mount">มีฐานเล็งเป้า</option><option value="without_mount">ไม่มีฐานเล็งเป้า</option></select></label>
  <label>ความเร็ว (cm/s, ไม่เกิน {{max_speed}})<input id="speed" type="number" step="1" value="{{default_speed|int}}"></label>
  <div style="display:flex;gap:6px">
   <button type="button" style="background:#57606f;margin-top:6px;padding:8px" onclick="setSpeed(6)">นุ่ม 6</button>
   <button type="button" style="background:#57606f;margin-top:6px;padding:8px" onclick="setSpeed(8)">ปกติ 8</button>
   <button type="button" style="background:#57606f;margin-top:6px;padding:8px" onclick="setSpeed(12)">เร็ว 12</button>
  </div>
  <label>ระยะที่สั่งเดิน (cm)<input id="dist" type="number" step="1" value="100"></label>
  <div class="hint">เดินแบบเดียวกับจอยโรงงาน (autogait): ≤10 cm/s ก้าวช้าและนุ่ม กินไฟน้อย — เวลาเดิน = ระยะ ÷ ความเร็ว ไม่เกิน {{max_dur}} วิ หุ่นไม่มีเซนเซอร์วัดระยะ จึงเดินตามเวลา (dead-reckoning) ไม่ให้เดินถ้าแบตต่ำกว่า {{min_bat}}%<br>
  ⚠️ วางหุ่นบนพื้น ไม่ใช่บนโต๊ะ เว้นที่ว่างด้านหน้า และยืนใกล้ๆ พร้อมยกหุ่นได้</div>
  <button class="go" id="go" onclick="start()">▶️ เริ่มเดิน</button>
  <button class="stop" onclick="stopNow()">⏹️ หยุดฉุกเฉิน</button>
  <div class="status" id="status">-</div>
 </div>
 <div class="card" style="margin-top:16px"><h2>2. วัดแล้วบันทึกผล</h2>
  <div class="hint">วัดด้วยตลับเมตรจากจุดเริ่ม (เทปที่พื้น) ถึงตำแหน่งที่หยุด — ระยะตามแนวตรงที่สั่ง และระยะเบี่ยงออกด้านข้าง (ขวา +, ซ้าย −)</div>
  <label>ระยะที่เดินได้จริง (cm)<input id="actual" type="number" step="0.5"></label>
  <label>ระยะเบี่ยงด้านข้าง (cm, ขวา +)<input id="lat" type="number" step="0.5"></label>
  <label><input id="fell" type="checkbox" style="width:auto"> หุ่นล้ม/เสียสมดุลจนเดินต่อไม่ได้</label>
  <label>หมายเหตุ<input id="note"></label>
  <button class="save" id="save" onclick="save()" disabled>💾 บันทึกผล</button>
  <div class="hint">บันทึกได้ 1 แถวต่อการเดิน 1 ครั้ง (ต้องกดเริ่มเดินก่อนทุกครั้ง)</div>
 </div>
</div>
<div class="card"><h2>📋 ผลการทดสอบ (<span id="n">0</span> ครั้ง) <a href="/api/walktest/export.csv" style="font-size:.75em">⬇ CSV</a></h2>
 <div id="stats"></div>
 <div style="overflow-x:auto"><table><thead><tr><th>เงื่อนไข</th><th>v</th><th>สั่ง</th><th>จริง</th><th>คลาด%</th><th>เบี่ยง</th><th>ล้ม</th><th></th></tr></thead><tbody id="rows"></tbody></table></div>
</div>
</div>
<script>
const COND={with_mount:'มีฐานเล็งเป้า',without_mount:'ไม่มีฐานเล็งเป้า'};
const j=(u,b)=>fetch(u,b?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)}:{}).then(r=>r.json());
function start(){
 const b={condition:document.getElementById('cond').value,speed_cms:+document.getElementById('speed').value,distance_cm:+document.getElementById('dist').value};
 j('/api/walktest/start',b).then(d=>{if(!d.success)alert(d.error);poll();});
}
function setSpeed(v){document.getElementById('speed').value=v;}
function stopNow(){j('/api/walktest/stop',{}).then(poll);}
window.addEventListener('beforeunload',()=>navigator.sendBeacon('/api/walktest/stop'));
function poll(){j('/api/walktest/status').then(s=>{
 let t=s.message;
 if(s.phase==='walking')t+=` (${s.elapsed}/${s.duration.toFixed(1)} วิ)`;
 document.getElementById('status').textContent=`${t} | แบต ${s.battery_pct??'-'}%`;
 document.getElementById('go').disabled=s.walking;
 document.getElementById('save').disabled=s.walking||!s.can_save;
});}
function save(){
 const b={actual_distance_cm:document.getElementById('actual').value,lateral_cm:document.getElementById('lat').value,
  fell:document.getElementById('fell').checked,note:document.getElementById('note').value};
 j('/api/walktest/save',b).then(d=>{if(!d.success){alert(d.error);return;}
  ['actual','lat','note'].forEach(i=>document.getElementById(i).value='');document.getElementById('fell').checked=false;load();poll();});
}
function del(i){if(confirm('ลบแถวนี้?'))j('/api/walktest/delete',{index:i}).then(load);}
function load(){
 j('/api/walktest/list').then(rs=>{document.getElementById('n').textContent=rs.length;
  document.getElementById('rows').innerHTML=rs.map((r,i)=>`<tr><td>${COND[r.condition]}</td><td>${r.speed_cms}</td><td>${r.cmd_distance_cm}</td><td>${r.actual_distance_cm??'-'}</td><td>${r.error_pct??'-'}</td><td>${r.lateral_cm??'-'}</td><td>${r.fell?'❌':'✅'}${r.completed?'':' ⏹️'}</td><td><button class="del" onclick="del(${i})">ลบ</button></td></tr>`).join('');});
 j('/api/walktest/stats').then(ss=>{document.getElementById('stats').innerHTML=ss.map(s=>`<div class="stat"><b>${COND[s.condition]}</b> n=${s.n} | ไม่ล้ม ${s.stability_pct}% (ล้ม ${s.n_fell}, หยุดก่อนครบ ${s.n_stopped_early})`+
  (s.n_measured?`<br>ระยะจริงเฉลี่ย ${s.mean_actual_cm} cm (S.D. ${s.sd_actual_cm}) | คลาดเฉลี่ย ${s.mean_error_cm} cm, |คลาด| ${s.mean_abs_error_pct}%<br>เบี่ยงเฉลี่ย ${s.mean_lateral_cm} cm (|เบี่ยง| ${s.mean_abs_lateral_cm}, S.D. ${s.sd_lateral_cm})`:'')+`</div>`).join('');});
}
load();poll();setInterval(poll,500);
</script></body></html>"""
