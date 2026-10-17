# -*- coding: utf-8 -*-
"""
按关节拆解"向内不到点/向外能经过":
- 重力对肩关节: 把大臂往下压 -> b_geo减小 -> 展臂(向外)  => 向外移动=重力助攻肩
- 重力对肘关节: 把前臂往下压 -> c_geo减小 -> 折叠(向内)  => 向内移动=重力助攻肘
- 舵机硬顶负载方向时关节精确跟随; 被重力"拖着走"时关节滞后 delta
"""
import math
UPPERARM, FORARM = 8.0, 8.0
BASEHEIGHT, CLAWLEN = 6.0, 6.3
CLAWOFFSET, BASE_ZERO = 0.4, 4.0

def ik(x, y, z):
    d = math.hypot(x, y)
    L = math.sqrt(d*d - CLAWOFFSET**2) - CLAWLEN
    h = z - BASEHEIGHT
    s = math.sqrt(L*L + h*h)
    sh = math.acos((UPPERARM**2 + s*s - FORARM**2)/(2*UPPERARM*s)) + math.atan2(h, L)
    el = sh + math.acos((UPPERARM**2 + FORARM**2 - s*s)/(2*UPPERARM*FORARM))
    base = math.degrees(math.atan2(y, x) + math.asin(CLAWOFFSET/d)) + BASE_ZERO
    return base, 120.0-math.degrees(sh), math.degrees(el)-52.5

def fk(base, sh_s, el_s):
    b = math.radians(120.0 - sh_s); c = math.radians(el_s + 52.5)
    L = math.sqrt(UPPERARM**2 + FORARM**2 - 2*UPPERARM*FORARM*math.cos(c-b))
    a1 = math.acos((UPPERARM**2 + L**2 - FORARM**2)/(2*UPPERARM*L))
    a2 = b - a1
    D, h = L*math.cos(a2), L*math.sin(a2)
    a = math.radians(base - BASE_ZERO) - math.atan2(CLAWOFFSET, D+CLAWLEN)
    ln = math.sqrt(CLAWOFFSET**2 + (D+CLAWLEN)**2)
    return ln*math.cos(a), ln*math.sin(a), BASEHEIGHT+h

def tip_err(p, tx, ty, tz):
    return math.hypot(p[0]-tx, p[1]-ty)*10, abs(p[2]-tz)*10  # 平面偏差mm, 高度偏差mm

tx, ty, tz = 0.0, 14.0, 5.0
tgt = ik(tx, ty, tz)
print("目标(0,14): 基%.1f 肩%.1f 肘%.1f" % tgt)
print("(肩舵机减小=展臂/向外, 肘舵机减小=折叠/向内)\n")

for d in [1.0, 1.5, 2.0]:
    print("--- 齿隙 %.1f 度 ---" % d)
    # 向内移动: 肘被重力拖着走 -> 肘关节滞后(少折叠d) = 肘舵机角偏大 d; 肩被硬顶 -> 精确
    p = fk(tgt[0], tgt[1], tgt[2] + d)
    e, ez = tip_err(p, tx, ty, tz)
    print("  向内移动(仅肘滞后): 平面偏差 %.1f mm, 高度偏差 %.1f mm" % (e, ez))
    # 向外移动: 肩被重力拖着走 -> 肩滞后(少展臂d) = 肩舵机角偏小 d; 肘被硬顶 -> 精确
    p = fk(tgt[0], tgt[1] - d, tgt[2])
    e, ez = tip_err(p, tx, ty, tz)
    print("  向外移动(仅肩滞后): 平面偏差 %.1f mm, 高度偏差 %.1f mm" % (e, ez))
    # 补偿验证: 向内移动时肘舵机多减 d (命令多折叠d), 抵消滞后
    p = fk(tgt[0], tgt[1], tgt[2] + d - d)
    e, ez = tip_err(p, tx, ty, tz)
    print("  向内+补偿(肘命令减%.1f): 平面偏差 %.1f mm" % (d, e))
