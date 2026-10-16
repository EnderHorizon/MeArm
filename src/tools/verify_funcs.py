# -*- coding: utf-8 -*-
"""
verify_funcs.py — 复刻 main.cpp 的 PosToAngle / AngleToPos（摆平面侧向偏心模型），数值验证：
1. 五个示教点的正逆解往返误差（应≈0）
2. 边界用例（旧模型的 NaN 区/折叠区现在应回待机角而非 NaN）
3. 旧模型（无偏心）在这台机器上的系统误差（应≈w，证明改的值）
4. 用当前 samples 跑折线/曲线，统计舵机钳位与轨迹偏差
"""
import math

U = F = 8.0
BH = 6.0
CL = 6.3
PEN_H = 3.0
INIT = (90.0, 20.0, 90.0)

# 与 main.cpp 一致
W = 0.4          # CLAWOFFSET：摆平面与底座转轴的侧向距离/cm
S0 = 0.0         # SHOULDER_S：肩关节在摆平面内的切向偏移/cm
BASE_ZERO = 90.0 # BASE_ZERO：底座舵机零位补偿/度
MIN_R = 10.0
MAX_R = 25.0
LIM = dict(base=(0.0, 180.0), shoulder=(20.0, 120.0), elbow=(70.0, 145.0))


def pos_to_angle(x, y, z):
    """复刻 main.cpp 的 PosToAngle（偏心模型，分支1：a = φ − arccos(w/r)）"""
    r = math.hypot(x, y)
    if r <= W or r < MIN_R or r > MAX_R:
        return INIT, "禁区/环外"
    phi = math.atan2(y, x)
    a = phi - math.acos(W / r)
    s = math.sqrt(r * r - W * W)
    h = z - BH
    L = s - S0 - CL
    d = math.hypot(L, h)
    if d <= 8.0 or d >= U + F:
        return INIT, "折叠/不可达"
    beta = math.acos((U * U + d * d - F * F) / (2 * U * d))
    gamma = math.acos((U * U + F * F - d * d) / (2 * U * F))
    sh = beta + math.atan2(h, L)
    el = sh + gamma
    return (math.degrees(a) + BASE_ZERO, 135.0 - math.degrees(sh), math.degrees(el) - 50.0), "ok"


def angle_to_pos(a, b, c):
    """复刻 main.cpp 的 AngleToPos（偏心模型）"""
    b = 135.0 - b
    c = c + 50.0
    a, b, c = math.radians(a - BASE_ZERO), math.radians(b), math.radians(c)
    d = math.sqrt(max(0.0, U * U + F * F - 2 * U * F * math.cos(c - b)))
    if d == 0.0:
        return None
    arg1 = (U * U + d * d - F * F) / (2 * U * d)
    if not (-1.0 <= arg1 <= 1.0):
        return None
    angle1 = math.acos(arg1)
    angle2 = b - angle1
    D = d * math.cos(angle2)
    h = d * math.sin(angle2)
    s = S0 + D + CL
    return (W * math.cos(a) - s * math.sin(a),
            W * math.sin(a) + s * math.cos(a), BH + h)


def old_pos_to_angle(x, y, z):
    """旧模型（无偏心）：底座角 = φ，L = r − 钳长"""
    r = math.hypot(x, y)
    L = r - CL
    h = z - BH
    d = math.hypot(L, h)
    if d == 0.0:
        return None
    beta = math.acos(max(-1.0, min(1.0, (U * U + d * d - F * F) / (2 * U * d))))
    gamma = math.acos(max(-1.0, min(1.0, (U * U + F * F - d * d) / (2 * U * F))))
    sh = beta + math.atan2(h, L)
    el = sh + gamma
    return (math.degrees(math.atan2(y, x)), 135.0 - math.degrees(sh), math.degrees(el) - 50.0)


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


SAMPLES = [None,
           (11.8, 14.8, PEN_H),
           (12.6, 11.4, PEN_H),
           (16.1, 10.4, PEN_H),
           (17.5, 8.2, PEN_H),
           (17.6, 4.0, PEN_H)]

print("=" * 74)
print("1) 正逆解往返（5 个示教点，z=3，w=0.4，BASE_ZERO=90）")
print("=" * 74)
for i in range(1, 6):
    x, y, z = SAMPLES[i]
    ang, st = pos_to_angle(x, y, z)
    if st != "ok":
        print(f"  P{i}: status={st}")
        continue
    rec = angle_to_pos(*ang)
    err = math.hypot(rec[0] - x, rec[1] - y)
    b_ok = LIM["base"][0] <= ang[0] <= LIM["base"][1]
    sh_ok = LIM["shoulder"][0] <= ang[1] <= LIM["shoulder"][1]
    el_ok = LIM["elbow"][0] <= ang[2] <= LIM["elbow"][1]
    phi = math.degrees(math.atan2(y, x))
    print(f"  P{i}({x:5.1f},{y:5.1f}): 方位φ={phi:5.1f}°  底座={ang[0]:6.1f}°{'✓' if b_ok else ' ✗超量程'} "
          f"肩={ang[1]:6.1f}°{'✓' if sh_ok else ' ✗'} 肘={ang[2]:6.1f}°{'✓' if el_ok else ' ✗'} "
          f"往返误差={err:.4f}cm")

print()
print("=" * 74)
print("2) 边界用例（偏心模型 + d 检查）")
print("=" * 74)
cases = [
    (15.0, 10.0, 3.0, "正常点（z=3）"),
    (15.0, 10.0, 6.0, "z=底座高度"),
    (15.0, 10.0, 12.0, "z=12（d 仍在连杆内）"),
    (15.0, 10.0, 25.0, "z=25（旧模型 NaN 区）"),
    (23.0, 5.0, 3.0, "r=23.5（旧模型 NaN 区）"),
    (24.0, 5.0, 6.0, "r=24.5（旧模型 NaN 区）"),
    (10.0, 10.0, 6.0, "r=14.1,z=6（旧模型折叠区）"),
    (10.5, 4.0, 3.0, "r=11.2（旧模型深度折叠）"),
    (0.5, 0.5, 3.0, "r=0.7 ≤ w（圆柱禁区）"),
    (5.0, 5.0, 3.0, "r=7.1 < minR（环外）"),
    (26.0, 5.0, 3.0, "r=26.5 > maxR（环外）"),
]
for x, y, z, desc in cases:
    ang, st = pos_to_angle(x, y, z)
    if st == "ok":
        sh_ok = LIM["shoulder"][0] <= ang[1] <= LIM["shoulder"][1]
        el_ok = LIM["elbow"][0] <= ang[2] <= LIM["elbow"][1]
        flag = "" if (sh_ok and el_ok) else "  ← 超舵机量程，会被 MoveTo 静默钳位"
        print(f"  ({x:5.1f},{y:5.1f},{z:5.1f}) {desc:26s} → 底座={ang[0]:6.1f}° 肩={ang[1]:6.1f}° 肘={ang[2]:6.1f}°{flag}")
    else:
        print(f"  ({x:5.1f},{y:5.1f},{z:5.1f}) {desc:26s} → {st}（回待机角 ✓ 不再 NaN）")

print()
print("=" * 74)
print("3) 旧模型（无偏心）在这台机器上的系统误差 = 这次修复消除的误差")
print("=" * 74)
for i in range(1, 6):
    x, y, z = SAMPLES[i]
    old_ang = old_pos_to_angle(x, y, z)
    actual = angle_to_pos(*old_ang)  # 旧角度放到真实（偏心）机器上
    err = math.hypot(actual[0] - x, actual[1] - y)
    print(f"  P{i}({x:5.1f},{y:5.1f}): 旧模型落点偏差 = {err:.3f}cm")

print()
print("=" * 74)
print("4) 用当前 samples 跑折线/曲线（新模型 + 舵机限位钳制后的实际轨迹）")
print("=" * 74)


def run_straight():
    pts = []
    for i in range(1, 5):
        x0, y0 = SAMPLES[i][0], SAMPLES[i][1]
        x1, y1 = SAMPLES[i + 1][0], SAMPLES[i + 1][1]
        for k in range(48):
            t = k / 48.0
            pts.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    pts.append((SAMPLES[5][0], SAMPLES[5][1]))
    return pts


def run_smooth():
    S = [None] * 7
    for i in range(1, 6):
        S[i] = SAMPLES[i]
    S[0] = tuple(2 * S[1][k] - S[2][k] for k in range(3))
    S[6] = tuple(2 * S[5][k] - S[4][k] for k in range(3))

    def vsub(a, b): return tuple(a[k] - b[k] for k in range(3))
    def vadd(a, b): return tuple(a[k] + b[k] for k in range(3))
    def vmul(a, s): return tuple(a[k] * s for k in range(3))
    def vdiv(a, s): return tuple(a[k] / s for k in range(3))
    def vlen(a): return math.sqrt(sum(a[k] * a[k] for k in range(3)))

    t = [0.0] * 7
    for i in range(6):
        t[i + 1] = t[i] + math.sqrt(vlen(vsub(S[i + 1], S[i])))
    tang = [None] * 7
    tang[0] = vdiv(vsub(S[1], S[0]), t[1] - t[0])
    tang[6] = vdiv(vsub(S[6], S[5]), t[6] - t[5])
    for i in range(1, 6):
        tang[i] = vadd(vmul(vdiv(vsub(S[i], S[i - 1]), t[i] - t[i - 1]), (t[i + 1] - t[i]) / (t[i + 1] - t[i - 1])),
                       vmul(vdiv(vsub(S[i + 1], S[i]), t[i + 1] - t[i]), (t[i] - t[i - 1]) / (t[i + 1] - t[i - 1])))
    pts = []
    for i in range(4):
        pts.append((S[i + 1][0], S[i + 1][1]))
        dt = t[i + 2] - t[i + 1]
        for k in range(1, 48):
            u = k / 48.0
            u2, u3 = u * u, u * u * u
            h00 = 2 * u3 - 3 * u2 + 1
            h10 = u3 - 2 * u2 + u
            h01 = -2 * u3 + 3 * u2
            h11 = u3 - u2
            res = vadd(vadd(vmul(S[i + 1], h00), vmul(tang[i + 1], h10 * dt)),
                       vadd(vmul(S[i + 2], h01), vmul(tang[i + 2], h11 * dt)))
            pts.append((res[0], res[1]))
    pts.append((S[5][0], S[5][1]))
    return pts


for name, pts in (("折线", run_straight()), ("曲线", run_smooth())):
    n_clamp = 0
    dev = 0.0
    zs = []
    for (px, py) in pts:
        ang, st = pos_to_angle(px, py, PEN_H)
        if st != "ok":
            continue
        b = clamp(ang[0], *LIM["base"])
        sh = clamp(ang[1], *LIM["shoulder"])
        el = clamp(ang[2], *LIM["elbow"])
        if (b, sh, el) != ang:
            n_clamp += 1
        fp = angle_to_pos(b, sh, el)
        dev = max(dev, math.hypot(fp[0] - px, fp[1] - py))
        zs.append(fp[2])
    print(f"  {name}: {len(pts)} 点，被舵机限位钳位 {n_clamp} 点，"
          f"与预期轨迹最大偏差 {dev:.2f}cm，爪尖高度 [{min(zs):.2f},{max(zs):.2f}]cm")
