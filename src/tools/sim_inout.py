# -*- coding: utf-8 -*-
"""
诊断"往内侧移动到不了点，往外推能经过"现象：
1) 用当前 main.cpp 的精确 IK(120/52.5偏移, minR=11.5) 扫描工作空间，找出舵机角度被钳位的区域
2) 验证用户5个示教点
3) 模拟齿隙(回程差)：同一目标点从内侧/外侧接近，落点差多少毫米
"""
import math

# ---- 与 main.cpp 完全一致的常量 ----
UPPERARM, FORARM = 8.0, 8.0
BASEHEIGHT, CLAWLEN = 6.0, 6.3
PENHEIGHT = 5.0
CLAWOFFSET, BASE_ZERO = 0.4, 4.0
MIN_R, MAX_R = 11.5, 22.0
# 播放时舵机限位(loop(): isplay -> elbow[40,145])
SH_MIN, SH_MAX = 20, 120
EL_MIN, EL_MAX = 40, 145

def ik(x, y, z):
    """返回 (base, shoulder_servo, elbow_servo) 或 None(超工作空间)"""
    d = math.hypot(x, y)
    if d < MIN_R or d > MAX_R:
        return None
    L = math.sqrt(d * d - CLAWOFFSET ** 2) - CLAWLEN
    h = z - BASEHEIGHT
    s = math.sqrt(L * L + h * h)
    sh_geo = math.acos(max(-1, min(1, (UPPERARM * UPPERARM + L * L + h * h - FORARM * FORARM) /
                       (2 * UPPERARM * s)))) + math.atan2(h, L)
    el_geo = sh_geo + math.acos(max(-1, min(1, (UPPERARM * UPPERARM + FORARM * FORARM - s * s) /
                                   (2 * UPPERARM * FORARM))))
    base = math.degrees(math.atan2(y, x) + math.asin(CLAWOFFSET / d)) + BASE_ZERO
    return base, 120.0 - math.degrees(sh_geo), math.degrees(el_geo) - 52.5

def fk(base, sh_servo, el_servo):
    """舵机角 -> 笔尖 (x,y,z)，与 AngleToPos 一致"""
    b = math.radians(120.0 - sh_servo)
    c = math.radians(el_servo + 52.5)
    L = math.sqrt(UPPERARM ** 2 + FORARM ** 2 - 2 * UPPERARM * FORARM * math.cos(c - b))
    a1 = math.acos(max(-1, min(1, (UPPERARM ** 2 + L ** 2 - FORARM ** 2) / (2 * UPPERARM * L))))
    a2 = b - a1
    D = L * math.cos(a2)
    h = L * math.sin(a2)
    a = math.radians(base - BASE_ZERO) - math.atan2(CLAWOFFSET, D + CLAWLEN)
    length = math.sqrt(CLAWOFFSET ** 2 + (D + CLAWLEN) ** 2)
    return length * math.cos(a), length * math.sin(a), BASEHEIGHT + h

def clamp_note(x, y, z):
    """返回 (ok, 说明)"""
    ang = ik(x, y, z)
    if ang is None:
        d = math.hypot(x, y)
        return False, "r=%.1f 超出[11.5,22] -> 返回init角, 根本不动" % d
    b, s, e = ang
    msgs = []
    if s < SH_MIN or s > SH_MAX: msgs.append("肩舵机%.1f越界[%d,%d]->钳位" % (s, SH_MIN, SH_MAX))
    if e < EL_MIN or e > EL_MAX: msgs.append("肘舵机%.1f越界[%d,%d]->钳位" % (e, EL_MIN, EL_MAX))
    if msgs: return False, "; ".join(msgs)
    return True, "ok"

# ================= 1. 用户5点 =================
print("=" * 62)
print("【1】你的5个示教点 (z=PENHEIGHT=5):")
pts = [(10, 15), (7, 18), (0, 15), (-6, 13), (-10, 17)]
for p in pts:
    ok, note = clamp_note(p[0], p[1], PENHEIGHT)
    ang = ik(p[0], p[1], PENHEIGHT)
    if ang:
        b, s, e = ang
        print("  (%5.1f,%5.1f) r=%5.2f -> 基%6.1f 肩%6.1f 肘%6.1f  %s" % (p[0], p[1], math.hypot(*p), b, s, e, note))
    else:
        print("  (%5.1f,%5.1f) %s" % (p[0], p[1], note))

# ================= 2. 钳位区扫描 =================
print("=" * 62)
print("【2】钳位区扫描 (z=5, 不同半径 r，方向任意):")
print("  r     肩舵机   肘舵机   状态")
for r in [11.0, 11.5, 12, 13, 14, 15, 16, 18, 20, 21.5, 22, 22.5]:
    x, y, z = r, 0.0, PENHEIGHT
    ang = ik(x, y, z)
    if ang is None:
        print("  %5.1f  %s" % (r, "超出工作空间(距离门)"))
        continue
    b, s, e = ang
    bad = []
    if s < SH_MIN or s > SH_MAX: bad.append("肩钳位")
    if e < EL_MIN or e > EL_MAX: bad.append("肘钳位")
    tag = "OK" if not bad else "<-- " + ",".join(bad)
    print("  %5.1f  %6.1f  %7.1f   %s" % (r, s, e, tag))

print("\n  同一半径 r=13，不同落笔高度 z:")
for z in [3.0, 4.0, 5.0, 6.0, 7.0]:
    ang = ik(13.0, 0.0, z)
    if ang is None:
        print("    z=%.1f  超出工作空间" % z); continue
    b, s, e = ang
    bad = []
    if s < SH_MIN or s > SH_MAX: bad.append("肩钳位")
    if e < EL_MIN or e > EL_MAX: bad.append("肘钳位(差%.1f度)" % (EL_MIN - e))
    print("    z=%.1f  肩%6.1f 肘%6.1f  %s" % (z, s, e, "OK" if not bad else "<-- " + ",".join(bad)))

# ================= 3. 齿隙模拟 =================
print("=" * 62)
print("【3】齿隙(回程差)模拟: 同一目标 (0,14)，分别从内、外接近")
print("  模型: 肩/肘舵机各带 delta 度齿隙; 运动时关节滞后舵机 delta,")
print("        到位后重力负载把关节再压回 delta -> 两方向落点不同")
tx, ty, tz = 0.0, 14.0, PENHEIGHT
tgt = ik(tx, ty, tz)
print("  目标舵机角: 基%.1f 肩%.1f 肘%.1f" % tgt)
for delta in [1.0, 1.5, 2.0]:
    print("  --- 齿隙 delta = %.1f 度 (肩+肘各) ---" % delta)
    # 从外侧来(先在外, 再向内移动到目标): 向内运动=折叠, 重力同向 -> 关节滞后(少折叠) delta
    sh_in, el_in = tgt[1] - delta, tgt[2] - delta   # 少折叠 = 角度偏小(肩/肘舵机角都偏小)
    p_in = fk(tgt[0], sh_in, el_in)
    # 从内侧来(先在内, 再向外移动到目标): 向外=展臂, 重力反向 -> 舵机推关节, 关节精确跟随
    p_out = fk(tgt[0], tgt[1], tgt[2])
    err_in = math.hypot(p_in[0] - tx, p_in[1] - ty)
    err_out = math.hypot(p_out[0] - tx, p_out[1] - ty)
    print("    向内移动到目标: 实际落点 (%.2f, %.2f) 偏差 %.1f mm  (到不了点)" % (p_in[0], p_in[1], err_in * 10))
    print("    向外移动到目标: 实际落点 (%.2f, %.2f) 偏差 %.1f mm  (经过)" % (p_out[0], p_out[1], err_out * 10))
print("=" * 62)
